package main

import (
	"fmt"
	"log"
	"math"
	"math/cmplx"
	"os"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/hashicorp/go-hclog"
	"github.com/hashicorp/go-plugin"

	"github.com/openconfig/gnmic/pkg/api/types"
	"github.com/openconfig/gnmic/pkg/formatters"
	"github.com/openconfig/gnmic/pkg/formatters/event_plugin"
)

const processorType = "event-birefringence-tracker"

func toFloat64(v interface{}) (float64, bool) {
	switch i := v.(type) {
	case float64:
		return i, true
	case int:
		return float64(i), true
	case int64:
		return float64(i), true
	case float32:
		return float64(i), true
	case string:
		f, err := strconv.ParseFloat(i, 64)
		return f, err == nil
	default:
		return 0, false
	}
}

// Parses stringified or numeric timestamps to int64
func parseTimestamp(v interface{}) (int64, bool) {
	switch i := v.(type) {
	case string:
		ts, err := strconv.ParseInt(i, 10, 64)
		return ts, err == nil
	case float64:
		return int64(i), true
	case int64:
		return i, true
	case int:
		return int64(i), true
	default:
		return 0, false
	}
}

type refState struct {
	matrix [4]complex128
	lastTs int64
}

type birefTrackerProcessor struct {
	TapRateHz float64 `mapstructure:"tap-rate-hz"`
	CutoffHz  float64 `mapstructure:"cutoff-hz"`

	mu          sync.Mutex
	refCache    map[string]*refState
	lastCleanup int64

	logger hclog.Logger
}

func (p *birefTrackerProcessor) Init(cfg interface{}, opts ...formatters.Option) error {
	err := formatters.DecodeConfig(cfg, p)
	if err != nil {
		return err
	}

	if p.TapRateHz <= 0 {
		return fmt.Errorf("tap-rate-hz must be strictly positive")
	}
	if p.CutoffHz >= p.TapRateHz/2.0 {
		return fmt.Errorf("cutoff-hz (%f) must be less than Nyquist (%f) of the nominal tap rate", p.CutoffHz, p.TapRateHz/2.0)
	}

	p.refCache = make(map[string]*refState)
	p.lastCleanup = time.Now().Unix()

	for _, o := range opts {
		o(p)
	}
	return nil
}

// shatter explodes Samples.XX arrays into distinct events using exact hardware timestamps.
// It discards any subcarrier data that is NOT data.0.
func (p *birefTrackerProcessor) shatter(events []*formatters.EventMsg) []*formatters.EventMsg {
	var res []*formatters.EventMsg

	for _, ev := range events {
		if ev == nil || ev.Values == nil {
			continue
		}

		// Pass 1: Extract exact hardware timestamps for each Sample
		timestamps := make(map[int]int64)
		for k, v := range ev.Values {
			if idx := strings.Index(k, "Samples."); idx != -1 {
				rest := k[idx+8:]
				parts := strings.Split(rest, "/")
				if len(parts) == 2 && parts[1] == "timestamp" {
					if sampleID, err := strconv.Atoi(parts[0]); err == nil {
						if ts, ok := parseTimestamp(v); ok {
							timestamps[sampleID] = ts
						}
					}
				}
			}
		}

		// Pass 2: Group metrics by sample, applying dimensionality reduction
		groups := make(map[int]map[string]interface{})
		nonSopValues := make(map[string]interface{})
		hasSOP := false

		for k, v := range ev.Values {
			idx := strings.Index(k, "Samples.")
			if idx == -1 {
				nonSopValues[k] = v // Non-SOP meta data passes through unharmed
				continue
			}

			rest := k[idx+8:]
			parts := strings.Split(rest, "/")
			sampleID, err := strconv.Atoi(parts[0])
			if err != nil {
				continue
			}

			hasSOP = true

			// We only process data payload arrays
			if len(parts) >= 3 && strings.HasPrefix(parts[1], "data.") {
				dataID := strings.TrimPrefix(parts[1], "data.")

				// DIMENSIONALITY REDUCTION: Keep only subcarrier 0. Drop the rest.
				if dataID != "0" {
					continue
				}

				metricName := strings.Join(parts[2:], "/")
				if groups[sampleID] == nil {
					groups[sampleID] = make(map[string]interface{})
				}
				// Append raw Stokes, Jones, and rate vectors verbatim
				groups[sampleID][metricName] = v
			}
		}

		if len(nonSopValues) > 0 || !hasSOP {
			res = append(res, &formatters.EventMsg{
				Name:      ev.Name,
				Timestamp: ev.Timestamp,
				Tags:      copyTags(ev.Tags),
				Values:    nonSopValues,
			})
		}

		// Distribute shattered metrics bound to their exact hardware temporal origin
		for sampleID, vals := range groups {
			ts, ok := timestamps[sampleID]
			if !ok {
				ts = ev.Timestamp // Fallback safety net
			}

			newTags := copyTags(ev.Tags)
			if newTags == nil {
				newTags = make(map[string]string)
			}
			newTags["data_id"] = "0"

			res = append(res, &formatters.EventMsg{
				Name:      ev.Name,
				Timestamp: ts,
				Tags:      newTags,
				Values:    vals, // Carries ALL context including stokes & rates
			})
		}
	}

	// Ensure chronological stability within the batch
	sort.SliceStable(res, func(i, j int) bool {
		return res[i].Timestamp < res[j].Timestamp
	})

	return res
}

func (p *birefTrackerProcessor) Apply(events ...*formatters.EventMsg) []*formatters.EventMsg {
	nowSec := time.Now().Unix()

	// 1. FAST LOCK: GC stale state matrices periodically
	p.mu.Lock()
	if nowSec-p.lastCleanup > 60 {
		for k, v := range p.refCache {
			if nowSec-(v.lastTs/1e9) > 60 { // lastTs is in nanoseconds
				delete(p.refCache, k)
			}
		}
		p.lastCleanup = nowSec
	}
	p.mu.Unlock()

	// 2. PARALLELIZABLE: Shatter the JSON arrays statelessly
	shattered := p.shatter(events)
	var outEvents []*formatters.EventMsg

	reqKeys := []string{
		"jones-matrix/xx/real", "jones-matrix/xx/imaginary",
		"jones-matrix/xy/real", "jones-matrix/xy/imaginary",
		"jones-matrix/yx/real", "jones-matrix/yx/imaginary",
		"jones-matrix/yy/real", "jones-matrix/yy/imaginary",
	}

	for _, ev := range shattered {
		if ev == nil || len(ev.Values) == 0 {
			continue
		}

		// Due to JSON_IETF atomicity, checking the map guarantees sample completeness.
		ready := true
		for _, rk := range reqKeys {
			if _, ok := ev.Values[rk]; !ok {
				ready = false
				break
			}
		}

		if !ready {
			outEvents = append(outEvents, ev)
			continue
		}

		vXXr, _ := toFloat64(ev.Values["jones-matrix/xx/real"])
		vXXi, _ := toFloat64(ev.Values["jones-matrix/xx/imaginary"])
		vXYr, _ := toFloat64(ev.Values["jones-matrix/xy/real"])
		vXYi, _ := toFloat64(ev.Values["jones-matrix/xy/imaginary"])
		vYXr, _ := toFloat64(ev.Values["jones-matrix/yx/real"])
		vYXi, _ := toFloat64(ev.Values["jones-matrix/yx/imaginary"])
		vYYr, _ := toFloat64(ev.Values["jones-matrix/yy/real"])
		vYYi, _ := toFloat64(ev.Values["jones-matrix/yy/imaginary"])

		A00 := complex(vXXr, vXXi)
		A01 := complex(vXYr, vXYi)
		A10 := complex(vYXr, vYXi)
		A11 := complex(vYYr, vYYi)

		source := ev.Tags["source"]
		comp := ev.Tags["component_name"]
		key := fmt.Sprintf("%s|%s|0", source, comp)

		// 3. MATH ENGINE: Lock is handled internally around cache interactions only
		angle, v0, v1, v2, ok := p.processJones(key, A00, A01, A10, A11, ev.Timestamp)

		if ok {
			ev.Values["polarization/rotation_angle_rad"] = angle
			ev.Values["polarization/rotation_axis/S3_sigma_x"] = v0
			ev.Values["polarization/rotation_axis/S2_sigma_y"] = v1
			ev.Values["polarization/rotation_axis/S1_sigma_z"] = v2
		}

		outEvents = append(outEvents, ev)
	}

	return outEvents
}

func (p *birefTrackerProcessor) processJones(key string, A00, A01, A10, A11 complex128, timestampNs int64) (float64, float64, float64, float64, bool) {
	// =========================================================
	// STATELESS PHASE: Parallelizes across gNMIc worker threads
	// =========================================================

	// 1. Polar Decomposition A = U*R -> extract unitary part U
	m00 := real(A00*cmplx.Conj(A00) + A10*cmplx.Conj(A10))
	m11 := real(A01*cmplx.Conj(A01) + A11*cmplx.Conj(A11))
	m01 := cmplx.Conj(A00)*A01 + cmplx.Conj(A10)*A11

	detM := m00*m11 - real(m01*cmplx.Conj(m01))
	if detM < 1e-12 {
		return 0, 0, 0, 0, false
	}

	sqrtDetM := math.Sqrt(detM)
	denom := complex(sqrtDetM, 0) * cmplx.Sqrt(complex(m00+m11+2.0*sqrtDetM, 0))

	ri00 := (complex(m11+sqrtDetM, 0)) / denom
	ri11 := (complex(m00+sqrtDetM, 0)) / denom
	ri01 := -m01 / denom
	ri10 := cmplx.Conj(ri01)

	U00 := A00*ri00 + A01*ri10
	U01 := A00*ri01 + A01*ri11
	U10 := A10*ri00 + A11*ri10
	U11 := A10*ri01 + A11*ri11

	// 2. SU(2) Projection: force det(U) = 1
	detU := U00*U11 - U01*U10
	sqDetU := cmplx.Sqrt(detU)
	if cmplx.Abs(sqDetU) < 1e-12 {
		return 0, 0, 0, 0, false
	}
	U00 /= sqDetU
	U01 /= sqDetU
	U10 /= sqDetU
	U11 /= sqDetU

	// =========================================================
	// STATEFUL PHASE: Critical section for memory integrity
	// =========================================================
	p.mu.Lock()
	defer p.mu.Unlock()

	state, exists := p.refCache[key]
	if !exists {
		p.refCache[key] = &refState{
			matrix: [4]complex128{U00, U01, U10, U11},
			lastTs: timestampNs,
		}
		return 0, 0, 0, 0, true
	}

	ref := state.matrix

	// Calculate exact delta time in seconds using hardware timestamps
	dt := float64(timestampNs-state.lastTs) / 1e9

	// Guard against exact duplicates causing negative/zero dt
	if dt <= 0 {
		dt = 1.0 / p.TapRateHz // Fallback to nominal rate
	}

	// Dynamic time-aware exponential decay weights
	dynAlpha := 1.0 - math.Exp(-2.0*math.Pi*p.CutoffHz*dt)
	dynBeta := 1.0 - dynAlpha

	// 3. SU(2) Branch-Cut Resolution (180-degree flip)
	// Keeps relative rotation contiguous in standard bounds, preventing 2π jumps.
	tr := real(U00*cmplx.Conj(ref[0]) + U01*cmplx.Conj(ref[1]) +
		U10*cmplx.Conj(ref[2]) + U11*cmplx.Conj(ref[3]))

	if tr < 0 {
		U00 = -U00
		U01 = -U01
		U10 = -U10
		U11 = -U11
	}

	// 4. Relative rotation D = U_now * U_ref^H
	D00 := U00*cmplx.Conj(ref[0]) + U01*cmplx.Conj(ref[1])
	D01 := U00*cmplx.Conj(ref[2]) + U01*cmplx.Conj(ref[3])
	D10 := U10*cmplx.Conj(ref[0]) + U11*cmplx.Conj(ref[1])
	D11 := U10*cmplx.Conj(ref[2]) + U11*cmplx.Conj(ref[3])

	// 5. Pauli extraction -> physical rotation vector extraction
	v1 := imag(D01 + D10) // sigma_x (horizontal/vertical)
	v2 := real(D01 - D10) // sigma_y (±45°)
	v3 := imag(D00 - D11) // sigma_z (circular)

	normV := math.Sqrt(v1*v1 + v2*v2 + v3*v3)
	angle := 2.0 * math.Atan2(normV, real(D00+D11))

	var sv1, sv2, sv3 float64
	if normV > 1e-12 {
		sv1 = (v1 / normV)
		sv2 = (v2 / normV)
		sv3 = (v3 / normV)
	}

	// 6. NLERP reference update using dynamic weights
	n00 := ref[0]*complex(dynBeta, 0) + U00*complex(dynAlpha, 0)
	n01 := ref[1]*complex(dynBeta, 0) + U01*complex(dynAlpha, 0)

	mag := math.Sqrt(real(n00*cmplx.Conj(n00) + n01*cmplx.Conj(n01)))
	if mag > 1e-12 {
		n00 /= complex(mag, 0)
		n01 /= complex(mag, 0)

		// Update reference matrix preserving SU(2) structure, log exact hardware TS
		state.matrix = [4]complex128{n00, n01, -cmplx.Conj(n01), cmplx.Conj(n00)}
		state.lastTs = timestampNs
	}

	return angle, sv1, sv2, sv3, true
}

func copyTags(t map[string]string) map[string]string {
	if t == nil {
		return nil
	}
	res := make(map[string]string, len(t))
	for k, v := range t {
		res[k] = v
	}
	return res
}

func (p *birefTrackerProcessor) WithActions(act map[string]map[string]interface{}) {}
func (p *birefTrackerProcessor) WithTargets(tcs map[string]*types.TargetConfig)    {}
func (p *birefTrackerProcessor) WithProcessors(procs map[string]map[string]any)    {}
func (p *birefTrackerProcessor) WithLogger(l *log.Logger)                          {}

func main() {
	logger := hclog.New(&hclog.LoggerOptions{
		Output:      os.Stderr,
		DisableTime: true,
		Level:       hclog.Info,
	})

	plug := &birefTrackerProcessor{
		logger: logger,
	}

	plugin.Serve(&plugin.ServeConfig{
		HandshakeConfig: plugin.HandshakeConfig{
			ProtocolVersion:  1,
			MagicCookieKey:   "GNMIC_PLUGIN",
			MagicCookieValue: "gnmic",
		},
		Plugins: map[string]plugin.Plugin{
			processorType: &event_plugin.EventProcessorPlugin{Impl: plug},
		},
		Logger: logger,
	})
}
