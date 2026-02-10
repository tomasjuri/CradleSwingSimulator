# KolebkaSim -- Physics & Control Theory

## 1. System overview

A rocking cradle (kolebka) is a rigid body with a curved bottom surface
-- the **rocker** -- that rolls without slipping on a flat ground.
Inside the cradle a **weight** slides back and forth on a horizontal rail
driven by a stepper motor.  By moving the weight at the right phase and
frequency the controller pumps energy into the rocking motion, much like
pumping a playground swing.

```
            ┌──────────────────┐
            │    cradle body   │   CoM at height h
            │   ┌─[weight]──┐ │   weight on rail
            │   └───rail────┘ │
            └───────┬──┬──────┘
               rocker legs
                 ╲    ╱        ← rocker profile y(x)
                  ╲  ╱
                   ╲╱
        ───────────●──────────── ground
              contact point
```

### Degrees of freedom

| Symbol | Description |
|--------|-------------|
| \(\theta\) | Tilt angle of the cradle (positive = tilt right / CW) |
| \(s\) | Weight position along the rail in the body frame (m) |

\(\theta\) is a free (dynamic) coordinate; \(s\) is driven by the motor.


## 2. Rocker profile

The bottom surface of the cradle is described in the **body frame** by

\[
  y(x) \;=\; R\,\left|\frac{x}{W}\right|^{\!p}
\]

| Parameter | Meaning | Default |
|-----------|---------|---------|
| \(R\) | Depth of the rocker curve | 0.08 m |
| \(W\) | Half-width of the rocker contact zone | 0.20 m |
| \(p\) | Shape exponent | 2.0 |

The exponent \(p\) controls the leg shape:

* **\(p = 2\)** -- parabolic / circular (profile A in the sketch).
  The radius of curvature at the bottom is \(R_c = W^2 / (2R)\).
* **\(p = 1\)** -- V-shape / pointed peak (profile B in the sketch).
  The radius of curvature at the bottom is zero.
* **\(1 < p < 2\)** -- intermediate shapes.

### Stability condition

The system is stable when the total centre of mass (CoM) is **below**
the centre of curvature of the rocker at the contact point:

\[
  h_{\text{cm}} \;<\; R_c \;=\; \frac{W^2}{2R}
  \quad\text{(for } p = 2\text{)}
\]

With the defaults: \(R_c = 0.04 / 0.16 = 0.25\) m and
\(h_{\text{cm}} = 0.15\) m, so the metacentric height is
\(d = R_c - h_{\text{cm}} = 0.10\) m > 0 -- stable.


## 3. Contact geometry

When the cradle tilts by angle \(\theta\) the contact point moves along
the rocker curve.  Two geometric steps determine the configuration:

1. **Find the contact point** on the curve where the tangent angle equals
   \(\theta\):

\[
  \frac{dy}{dx}\bigg|_{x_c} = \tan\theta
  \quad\Longrightarrow\quad
  x_c = W\!\left(\frac{W\tan\theta}{Rp}\right)^{\!\frac{1}{p-1}}
\]

2. **Rolling constraint** -- no slipping.  The arc length along the
   curve from \(x = 0\) to \(x = x_c\) equals the horizontal distance
   the contact has travelled on the ground:

\[
  x_{\text{ground}} = \operatorname{sgn}(\theta)\;\int_0^{|x_c|}
      \sqrt{1 + \left(\frac{dy}{dx}\right)^{\!2}}\;dx
\]

From the contact point the body-frame origin (and therefore every
point in the body) is located using the clockwise rotation matrix:

\[
  \mathbf{R}_{\text{cw}}(\theta)
  = \begin{pmatrix} \cos\theta & \sin\theta \\
                    -\sin\theta & \cos\theta \end{pmatrix}
\]


## 4. Equations of motion

The tilt angle evolves according to angular momentum balance about the
instantaneous contact point \(C\):

\[
  I_{\text{total}}\;\ddot\theta
  = \tau_{\text{gravity}}
  + \tau_{\text{coriolis}}
  + \tau_{\text{motor}}
  + \tau_{\text{damping}}
\]

### 4.1 Gravitational torque

For each mass (cradle body \(M\) and weight \(m\)) the CW gravitational
torque about \(C\) is:

\[
  \tau_{\text{grav}} = M g\, r_x^{\text{cradle}} + m g\, r_x^{\text{weight}}
\]

where \(r_x\) is the **horizontal world-frame distance** from the
contact point to the mass.  When the CoM is to the right of the
contact, \(r_x > 0\) and the torque is positive (CW, increasing
\(\theta\)) -- this is the destabilising direction.  For a stable
rocker the CoM ends up to the **left** of the contact when tilted
right, giving a restoring (negative) torque.

### 4.2 Moment of inertia

\[
  I_{\text{total}} = I_{\text{cm,cradle}} + M\,d_{\text{cradle}}^2
                    + m\,d_{\text{weight}}^2
\]

where \(d\) is the distance from \(C\) to each mass (parallel-axis
theorem) and the cradle's own moment of inertia is approximated as an
ellipse: \(I_{\text{cm}} = \frac{M}{4}(a^2 + b^2)\).

### 4.3 Coriolis coupling

When the weight moves on the rail at velocity \(\dot s\), it changes
the moment of inertia.  Conservation of angular momentum produces a
coupling torque analogous to an ice skater extending their arms:

\[
  \tau_{\text{coriolis}} = -2\,m\,s\,\dot s\,\dot\theta
\]

### 4.4 Motor reaction torque

The motor exerts a force \(F = m\,\ddot s\) on the weight along the
rail.  The equal-and-opposite reaction on the cradle creates a torque
about \(C\) proportional to the perpendicular distance from \(C\) to
the rail:

\[
  \tau_{\text{motor}} = -m\,\ddot s\;\bigl(h_{\text{rail}} - y_c\bigr)
\]

where \(y_c = R\,|x_c/W|^p\) is the height of the contact point on
the rocker curve and \(h_{\text{rail}}\) is the rail height in the body
frame.

### 4.5 Damping

Surface rolling resistance is modelled as viscous damping:

\[
  \tau_{\text{damp}} = -b\,\dot\theta
\]

The coefficient \(b\) is high for carpet (~0.5) and low for hard
flooring (~0.05).

### 4.6 Integration

The ODE is integrated with the classical **4th-order Runge-Kutta**
method at a timestep of 0.5 ms.  Elastic-wall boundary conditions
clamp \(\theta\) and \(\dot\theta\) at the configurable maximum tilt
angle.


## 5. Control algorithm

### 5.1 Goal

Maintain a target rocking amplitude while **minimising the weight
displacement**.  At steady state the weight should move as little as
possible -- just enough to replace the energy lost to damping.

### 5.2 Amplitude estimation

Instead of noisy peak detection, the controller uses a **continuous
energy-based amplitude estimate**:

\[
  A_{\text{est}} = \sqrt{\theta^2 + \left(\frac{\dot\theta}{\omega_0}\right)^{\!2}}
\]

This is proportional to the square root of the total oscillation energy
and is smooth at every timestep.  The natural frequency \(\omega_0\) is
estimated from zero-crossings of \(\theta\).

### 5.3 Displacement fraction

The key output of the controller is the **displacement fraction** \(f\)
(\(0 \ldots 1\)), which sets how far the weight travels relative to the
full rail span.

At **steady state**, the fraction is derived from energy balance.  The
energy dissipated per cycle by damping is:

\[
  \Delta E_{\text{damp}} = \pi\,b\,\omega_0\,A^2
\]

The energy injected per cycle by the weight is approximately:

\[
  \Delta E_{\text{inject}} \approx \pi\,m\,g\,f\,L_{\text{rail}}\,A
\]

Setting \(\Delta E_{\text{inject}} = \Delta E_{\text{damp}}\) and
solving for the sustain fraction:

\[
  f_{\text{sustain}} = \frac{b\,\omega_0\,A}{m\,g\,L_{\text{rail}}}
\]

With the defaults (\(b = 0.15\), \(\omega_0 \approx 3\) rad/s,
\(A = 0.15\) rad, \(m = 1\) kg, \(L = 0.15\) m):

\[
  f_{\text{sustain}} \approx \frac{0.15 \times 3 \times 0.15}{1 \times 9.81 \times 0.15}
  \approx 0.046 \;\;(\approx 5\%)
\]

So at steady state the weight moves only about **5 % of the rail**
(~7 mm amplitude).

During **ramp-up** an additional proportional term is added based on
the amplitude deficit, capped at 50 % of the rail.

### 5.4 Phase-locked drive

The weight target position is set proportional to \(\dot\theta\):

\[
  s_{\text{target}} = f \cdot \frac{\dot\theta}{\max|\dot\theta|}
      \cdot L_{\text{rail}}
\]

This automatically:

* **Locks to the natural frequency** -- the weight oscillates at the
  same frequency as the cradle.
* **Locks to the optimal phase** -- the weight is at maximum
  displacement when \(|\dot\theta|\) is maximum (i.e. when \(\theta\)
  passes through zero), which is exactly when energy injection is most
  efficient.
* **Scales smoothly** -- no discontinuities or bang-bang switching.

### 5.5 Motor constraints

A PD position controller tracks \(s_{\text{target}}\) while respecting
the stepper motor limits:

* Maximum acceleration: 2.0 m/s\(^2\)
* Maximum velocity: 0.3 m/s
* Rail span: \(\pm\)15 cm


## 6. Configurable parameters

| Group | Parameter | Default | Description |
|-------|-----------|---------|-------------|
| Cradle | mass | 5.0 kg | Empty cradle mass |
| | cm_height | 0.15 m | CoM height above rocker origin |
| | body_width | 0.40 m | Half-width (for inertia) |
| | body_height | 0.25 m | Half-height (for inertia) |
| Rocker | R | 0.08 m | Curve depth |
| | W | 0.20 m | Curve half-width |
| | p | 2.0 | Shape exponent (2 = round, 1 = pointed) |
| Weight | mass | 1.0 kg | Moving weight mass |
| | rail_height | 0.15 m | Rail height in body frame |
| | rail_half_span | 0.15 m | Max weight displacement |
| Motor | max_acceleration | 2.0 m/s^2 | Stepper limit |
| | max_velocity | 0.3 m/s | Noise constraint |
| Surface | damping | 0.15 N m s/rad | Rolling resistance |
| Control | target_amplitude | 0.15 rad (~8.6 deg) | Desired swing |
| | gain | 1.0 | Controller gain (0..1) |
