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

type subcarrierBuffer struct {
	values    map[string]float64
	timestamp int64
	tags      map[string]string
	name      string
	lastTouch int64
}

// refState holds the reference matrix and the exact timestamp of the last update
type refState struct {
	matrix [4]complex128
	lastTs int64
}

type birefTrackerProcessor struct {
	TapRateHz float64 `mapstructure:"tap-rate-hz"`
	CutoffHz  float64 `mapstructure:"cutoff-hz"`

	mu          sync.Mutex
	bufferCache map[string]*subcarrierBuffer
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

	p.bufferCache = make(map[string]*subcarrierBuffer)
	p.refCache = make(map[string]*refState)
	p.lastCleanup = time.Now().Unix()

	for _, o := range opts {
		o(p)
	}
	return nil
}

// shatter natively explodes the samples arrays into distinct, chronologically spaced events.
func (p *birefTrackerProcessor) shatter(events []*formatters.EventMsg) []*formatters.EventMsg {
	var res []*formatters.EventMsg

	for _, ev := range events {
		if ev == nil || ev.Values == nil {
			continue
		}

		maxSampleID := -1
		hasSOP := false

		// Pass 1: Find max sample_id to infer temporal spacing
		for k := range ev.Values {
			if idx := strings.Index(k, "Samples."); idx != -1 {
				rest := k[idx+8:]
				slashIdx := strings.IndexByte(rest, '/')
				if slashIdx != -1 {
					idStr := rest[:slashIdx]
					if id, err := strconv.Atoi(idStr); err == nil {
						if id > maxSampleID {
							maxSampleID = id
						}
						hasSOP = true
					}
				}
			}
		}

		// Fast-path: bypass events with no sample arrays
		if !hasSOP {
			res = append(res, ev)
			continue
		}

		numSamples := int64(maxSampleID + 1)
		nsIncrement := int64(1e9) / numSamples

		type groupKey struct {
			sampleID int
			dataID   string
		}
		groups := make(map[groupKey]map[string]interface{})
		nonSopValues := make(map[string]interface{})

		// Pass 2: Shatter and re-map
		for k, v := range ev.Values {
			idx := strings.Index(k, "Samples.")
			if idx == -1 {
				nonSopValues[k] = v
				continue
			}

			rest := k[idx+8:]
			parts := strings.Split(rest, "/")
			if len(parts) < 3 {
				nonSopValues[k] = v // fallback
				continue
			}

			sampleID, err := strconv.Atoi(parts[0])
			if err != nil {
				nonSopValues[k] = v
				continue
			}

			dataIDParts := strings.Split(parts[1], ".")
			dataID := "0"
			if len(dataIDParts) > 1 {
				dataID = dataIDParts[1]
			}

			metricName := strings.Join(parts[2:], "/")

			gk := groupKey{sampleID, dataID}
			if groups[gk] == nil {
				groups[gk] = make(map[string]interface{})
			}
			groups[gk][metricName] = v
		}

		// Keep non-SOP values at base timestamp
		if len(nonSopValues) > 0 {
			res = append(res, &formatters.EventMsg{
				Name:      ev.Name,
				Timestamp: ev.Timestamp,
				Tags:      copyTags(ev.Tags),
				Values:    nonSopValues,
			})
		}

		// Distribute shattered metrics over the timeline
		for gk, vals := range groups {
			newTags := copyTags(ev.Tags)
			if newTags == nil {
				newTags = make(map[string]string)
			}
			newTags["data_id"] = gk.dataID

			res = append(res, &formatters.EventMsg{
				Name:      ev.Name,
				Timestamp: ev.Timestamp + (int64(gk.sampleID) * nsIncrement),
				Tags:      newTags,
				Values:    vals,
			})
		}
	}

	sort.SliceStable(res, func(i, j int) bool {
		return res[i].Timestamp < res[j].Timestamp
	})

	return res
}

func (p *birefTrackerProcessor) Apply(events ...*formatters.EventMsg) []*formatters.EventMsg {
	p.mu.Lock()
	defer p.mu.Unlock()

	nowSec := time.Now().Unix()

	// Garbage collection for stale fragments
	if nowSec-p.lastCleanup > 60 {
		for k, v := range p.bufferCache {
			if nowSec-v.lastTouch > 60 {
				delete(p.bufferCache, k)
				delete(p.refCache, k)
			}
		}
		p.lastCleanup = nowSec
	}

	// Explode samples inline (replaces Starlark)
	shattered := p.shatter(events)

	var outEvents []*formatters.EventMsg

	reqKeys := []string{
		"jones-matrix/xx/real", "jones-matrix/xx/imaginary",
		"jones-matrix/xy/real", "jones-matrix/xy/imaginary",
		"jones-matrix/yx/real", "jones-matrix/yx/imaginary",
		"jones-matrix/yy/real", "jones-matrix/yy/imaginary",
	}

	for _, ev := range shattered {
		if ev == nil || ev.Values == nil {
			continue
		}

		jonesValues := make(map[string]float64)
		otherValues := make(map[string]interface{})

		for k, v := range ev.Values {
			isReq := false
			for _, rk := range reqKeys {
				if k == rk {
					isReq = true
					break
				}
			}
			if isReq {
				if fv, ok := toFloat64(v); ok {
					jonesValues[k] = fv
				}
			} else {
				otherValues[k] = v
			}
		}

		if len(otherValues) > 0 {
			evOther := &formatters.EventMsg{
				Name:      ev.Name,
				Timestamp: ev.Timestamp,
				Tags:      copyTags(ev.Tags),
				Values:    otherValues,
			}
			outEvents = append(outEvents, evOther)
		}

		if len(jonesValues) == 0 {
			continue
		}

		if ev.Tags == nil {
			continue
		}
		source := ev.Tags["source"]
		comp := ev.Tags["component_name"]
		dataID := ev.Tags["data_id"]

		if source == "" || comp == "" || dataID == "" {
			continue
		}

		key := fmt.Sprintf("%s|%s|%s", source, comp, dataID)

		b, exists := p.bufferCache[key]
		if !exists {
			b = &subcarrierBuffer{
				values: make(map[string]float64),
				tags:   copyTags(ev.Tags),
			}
			p.bufferCache[key] = b
		}

		b.timestamp = ev.Timestamp
		b.name = ev.Name
		b.lastTouch = nowSec
		for k, v := range jonesValues {
			b.values[k] = v
		}

		ready := true
		for _, rk := range reqKeys {
			if _, ok := b.values[rk]; !ok {
				ready = false
				break
			}
		}

		if ready {
			A00 := complex(b.values["jones-matrix/xx/real"], b.values["jones-matrix/xx/imaginary"])
			A01 := complex(b.values["jones-matrix/xy/real"], b.values["jones-matrix/xy/imaginary"])
			A10 := complex(b.values["jones-matrix/yx/real"], b.values["jones-matrix/yx/imaginary"])
			A11 := complex(b.values["jones-matrix/yy/real"], b.values["jones-matrix/yy/imaginary"])

			angle, v0, v1, v2, ok := p.processJones(key, A00, A01, A10, A11, b.timestamp)
			if ok {
				outEv := &formatters.EventMsg{
					Name:      b.name,
					Timestamp: b.timestamp,
					Tags:      copyTags(b.tags),
					Values: map[string]interface{}{
						"polarization/rotation_axis/S3_sigma_x":       v0,
						"polarization/rotation_axis/S2_sigma_y":       v1,
						"polarization/rotation_axis/S1_sigma_z":       v2,
						"polarization/rotation_angle_rad":   angle,
						"jones-matrix/xx/real":                        b.values["jones-matrix/xx/real"],
						"jones-matrix/xx/imaginary":                   b.values["jones-matrix/xx/imaginary"],
						"jones-matrix/xy/real":                        b.values["jones-matrix/xy/real"],
						"jones-matrix/xy/imaginary":                   b.values["jones-matrix/xy/imaginary"],
						"jones-matrix/yx/real":                        b.values["jones-matrix/yx/real"],
						"jones-matrix/yx/imaginary":                   b.values["jones-matrix/yx/imaginary"],
						"jones-matrix/yy/real":                        b.values["jones-matrix/yy/real"],
						"jones-matrix/yy/imaginary":                   b.values["jones-matrix/yy/imaginary"],
					},
				}
				outEvents = append(outEvents, outEv)
			}

			// Clear matrix to prepare for next sample fragment
			for _, rk := range reqKeys {
				delete(b.values, rk)
			}
		}
	}

	return outEvents
}

func (p *birefTrackerProcessor) processJones(key string, A00, A01, A10, A11 complex128, timestampNs int64) (float64, float64, float64, float64, bool) {
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

	state, exists := p.refCache[key]
	if !exists {
		p.refCache[key] = &refState{
			matrix: [4]complex128{U00, U01, U10, U11},
			lastTs: timestampNs,
		}
		return 0, 0, 0, 0, true
	}

	ref := state.matrix

	// Calculate exact delta time in seconds (gNMI timestamps are nanoseconds)
	dt := float64(timestampNs-state.lastTs) / 1e9

	// Guard against exact duplicates or out-of-order packets causing negative dt
	if dt <= 0 {
		dt = 1.0 / p.TapRateHz // Fallback to nominal rate
	}

	// Dynamic time-aware exponential decay weights
	dynAlpha := 1.0 - math.Exp(-2.0*math.Pi*p.CutoffHz*dt)
	dynBeta := 1.0 - dynAlpha

	// 3. SU(2) Branch-Cut Resolution (180-degree flip)
	// Keeps relative rotation contiguous in standard bounds, preventing 2π jumps in resulting vectors.
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

	// 5. Pauli extraction -> rotation vector
	v1 := imag(D01 + D10) // sigma_x
	v2 := real(D01 - D10) // sigma_y
	v3 := imag(D00 - D11) // sigma_z

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

		// Update reference matrix preserving SU(2) structure, and log exact timestamp
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
