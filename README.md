# Optical Fiber Birefringence High-pass Filter

## How to use it

```bash
cd src/
go mod tidy
CGO_ENABLED=0 go build -o ../gnmic_plugins/event-birefringence-tracker main.go
chmod +x ../gnmic_plugins/event-birefringence-tracker
```

Copy `/gnmic_plugins/event-birefringence-tracker` into gnmic custom plugins directory as defined in your `gnmic.yaml` configuration file.

```yml
plugins:
  path: /app/plugins/ # this is the custom plugins directory
  glob: "*"
  start-timeout: 5s


processors:
  birefringence-tracker:
    # This key MUST match the `processorType` string constant in the Go code.
    event-birefringence-tracker:
      tap-rate-hz: 100.0 # nominal frequency of SOP-data sampling
      cutoff-hz: 0.01 # frequency above which changes are considered 'fast' 

# the rest of the gnmic.yaml configuration file

# in the outputs you should include the processor in the list of event-processors:
outputs:
  your-output:
    event-processors:
      - birefringence-tracker

```

## Processor's pseudo-code

```
Initialize:
    cutoff_hz = High-pass filter cutoff (e.g., 0.01 Hz)
    U_ref = None
    last_timestamp = None

For each incoming 2x2 Jones Matrix A(t) at time t:
    
    // --- 1. Polar Decomposition & SU(2) Projection ---
    // Separate the matrix A into a purely rotational part (U) and a loss/gain part
    Compute U = A * (A^H * A)^(-1/2) 
    
    // Force determinant to 1 to project perfectly onto SU(2) group
    detU = det(U)
    U_now = U / sqrt(detU)
    
    // --- 2. State Initialization ---
    If U_ref is None:
        U_ref = U_now
        last_timestamp = t
        Continue

    // --- 3. Branch-Cut Resolution (The 180-degree flip) ---
    // In SU(2), U and -U represent the EXACT SAME physical rotation in SO(3).
    // To prevent the math from taking the "long way around" the sphere (a 2pi phase jump):
    trace = real( U_now * U_ref^H )
    If trace < 0:
        U_now = -U_now

    // --- 4. Relative Rotation Extraction ---
    // Compute the rotation operator that moves U_ref to U_now
    D = U_now * U_ref^H

    // --- 5. Pauli Vector Projection (The Physics) ---
    // Any SU(2) matrix can be written as cos(θ)I + i*sin(θ)(v_x*σ_x + v_y*σ_y + v_z*σ_z)
    // Extract the physical 3D strain vector (v_x, v_y, v_z) using Pauli matrices
    v_x = imag(D01 + D10)
    v_y = real(D01 - D10)
    v_z = imag(D00 - D11)
    
    norm_v = sqrt(v_x^2 + v_y^2 + v_z^2)
    rotation_angle_rad = 2 * atan2(norm_v, real(D00 + D11))
    
    rotation_axis = (v_x, v_y, v_z) / norm_v

    // --- 6. Time-Aware NLERP (Non-Linear Exponential Rolling Prediction) ---
    dt = t - last_timestamp
    alpha = 1 - exp(-2 * pi * cutoff_hz * dt)
    beta = 1 - alpha

    // Linearly interpolate the reference, then re-normalize to stay on the SU(2) manifold
    U_ref_raw = (beta * U_ref) + (alpha * U_now)
    mag = sqrt(|U_ref_raw_00|^2 + |U_ref_raw_01|^2)
    U_ref = [ U_ref_raw_00 / mag, U_ref_raw_01 / mag ]
            [ -conj(U_ref_raw_01)/mag, conj(U_ref_raw_00)/mag ]
    
    last_timestamp = t

    Return rotation_angle_rad, rotation_axis
```

## How to interpret the outputs

The `rotation_axis` and `rotation_angle_rad` quantify the **high-frequency differential perturbation** (the "fast transient") applied *on top* of the fiber's baseline birefringence. They do not represent the total birefringence of the optical link. Because this processor acts as a high-pass filter, the outputs isolate sudden mechanical or acoustic impacts (e.g., trains, vibrations, aerial cable wind-whip) while ignoring slow thermal drifts.

If the receiver is currently tracking a stable, low-pass-filtered polarization state $\vec{S}\_{ref}$, a transient event will cause the incoming light to suddenly whip away from this baseline into a new polarization state $\vec{S}\_{last}$. The `rotation_axis` and `rotation_angle_rad` tell you around what axis and by how many radians to rotate $\vec{S}\_{ref}$ on the Poincaré sphere to obtain $\vec{S}\_{last}$.

**`rotation_axis`**: The two polarizations found by intersecting this axis with the Poincaré sphere are the eigenvectors of the *transient perturbation*. If the light arriving at the perturbation point happens to be polarized with one of these two polarizations, the transient will not induce a change in the polarization.

**`rotation_angle_rad`**: This is the clockwise magnitude of the sudden rotation around the `rotation_axis`. Physically, it is the retardance induced by the transient perturbation of one eigen-polarization with respect to the other. For example, suppose the sudden rotation is around the S1-axis by $\theta$. The input field can be written as a linear combination of H and V polarizations (eigenvectors). Then, after the rotation, the phase difference of H and V components is changed by $\theta$. 

Mathematically, the sudden rotation matrix $D$ around S1-axis is:

$$ D = \begin{pmatrix} 
e^{i\theta/2} & 0 \\
0 & e^{-i\theta/2} 
\end{pmatrix} $$

Let the input electric field be an arbitrary mix of H and V:

$$ \vec{E}_{in} = \begin{pmatrix} 
|E_H| e^{i\phi_H} \\
|E_V| e^{i\phi_V}
\end{pmatrix} $$

Applying the birefringence operator $D$:

$$ \vec{E}_{out} = D \vec{E}_{in} = \begin{pmatrix} 
|E_H| e^{i(\phi_H + \theta/2)} \\ 
|E_V| e^{i(\phi_V - \theta/2)} 
\end{pmatrix} $$

The final phase difference between the H and V fields is:

$$ \Delta\phi' = \phi_H' - \phi_V' = \left(\phi_H + \frac{\theta}{2}\right) - \left(\phi_V - \frac{\theta}{2}\right) = (\phi_H - \phi_V) + \theta $$
