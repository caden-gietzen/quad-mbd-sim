# Plant Verification Requirements (open-loop)

Requirements for validating the 6DOF quadrotor **plant** (`plant/`) and its
Simulink simulation wiring, independent of any controller. The goal is to
confirm that the model obeys rigid-body physics and that the actuator/mixing
path is wired correctly, before the plant is trusted as the truth model for
closed-loop control design and V&V.

Each requirement is verified open-loop by a `matlab.unittest` case in
`vnv/tPlant.m` driven through the `vnv/run_plant_case.m` harness. Every test
cites the `REQ-PLANT-xxx` it verifies with a `% Verifies REQ-PLANT-xxx`
comment, and the matrix at the bottom traces requirements to tests.

**Conventions** (authoritative source: `docs/conventions.md`): NED world
(+z = down, gravity `[0,0,+9.81]`), FRD body (x-fwd, y-right, z-down; thrust
along body −z), Hamilton scalar-first quaternions `[w x y z]`. State vector is
`[r(3) v(3) q(4) omega(3)]` (see `vnv/unpack_state.m`). Tolerances on the
analytic requirements are set above the fixed variable-step solver's own error
(`ode45`, default `RelTol 1e-3`); tolerances on the conservation requirements
are drift bounds over the stated horizon.

**Status summary:** the rigid-body dynamics group (REQ-PLANT-001…006) is
**verified** — implemented in `vnv/tPlant.m` and passing. The actuator/wiring
group (REQ-PLANT-007…010) is **planned** — specified here but not yet
implemented.

---

## A. Rigid-body dynamics

### REQ-PLANT-001 — Ballistic trajectory
With zero motor thrust and no aerodynamic forces, the plant shall propagate the
CoM along the analytic projectile solution `r(t) = r0 + v0*t + 0.5*g*t^2`
(g = `[0,0,+9.81]`, NED).
- **Preconditions:** `T = 0`; identity attitude; initial velocity `v0`.
- **Acceptance:** `norm(r_sim(t) - r_analytic(t)) < 1e-3 m` for all `t ∈ [0, 1] s`.
- **Verification:** `tPlant/freeFall` (vertical) and `tPlant/ballisticTrajectory`
  (angled launch, all three position channels). **Status: verified.**
- **Uniquely catches:** translational integrator wiring and gravity direction,
  against an independent analytic oracle.

### REQ-PLANT-002 — Mechanical energy conservation
With zero motor thrust and no aerodynamic forces (no non-conservative forces),
total mechanical energy `E = 0.5*m*|v|^2 + m*g*h` (h = altitude = −z) shall be
conserved over the run.
- **Preconditions:** `T = 0`; started at altitude so `E(0)` is a nonzero scale.
- **Acceptance:** `|E(t) - E(0)| / E(0) < 1e-3` over the run.
- **Verification:** `tPlant/zeroTorqueAttitude`. **Status: verified.**
- **Uniquely catches:** systematic numerical dissipation or energy injection
  (a damping/sign error that adds or removes energy) that a short analytic
  position check would not reveal.

### REQ-PLANT-003 — Angular momentum conservation (torque-free)
Under zero net torque, the magnitude of the angular momentum `L = I*omega`
shall remain constant (gravity acts at the CoM and applies no moment). Note the
body *rates* precess for an off-principal-axis spin — it is `|L|` that is
invariant, not `omega`.
- **Preconditions:** zero net moment; off-principal-axis `omega0`.
- **Acceptance:** `| |L(t)| - |L(0)| | / |L(0)| < 1e-3` over the run.
- **Verification:** `tPlant/angularMomentumConservation`. **Status: verified.**
- **Uniquely catches:** rotational integrator and the `cross(omega, I*omega)`
  gyroscopic term producing a spurious torque.

### REQ-PLANT-004 — Torque-free principal-axis rotation
Given an initial body rate about a single principal (body) axis and zero net
torque, the rate about that axis shall remain constant and the off-axis rates
shall remain zero.
- **Preconditions:** zero net moment; `omega0 = e_k * w0` for principal axis k.
- **Acceptance:** `|omega(t) - omega0| / |omega0| < 1e-3` over the run.
- **Verification:** `tPlant/singleAxisSpin`. **Status: verified.**
- **Uniquely catches:** spurious inter-axis coupling in the rotational dynamics.

### REQ-PLANT-005 — Gyroscopic coupling
A constant moment applied about one body axis of a body spinning about an
orthogonal axis shall produce precession about the third axis of the sign and
magnitude predicted by Euler's equations.
- **Preconditions:** constant spin `omega_z = Omega`; small constant moment
  `M_x`; `I = diag(Ix, Iy, Iz)`.
- **Acceptance:** `omega_y` develops with the sign of `M_x/((Iz-Iy)*Omega)` and
  its mean over whole nutation periods is within 10% of `M_x/((Iz-Iy)*Omega)`;
  `omega_x` remains bounded (nutation only).
- **Verification:** `tPlant/gyroscopicCoupling`. **Status: verified.**
- **Uniquely catches:** the sign and scale of the `cross(omega, I*omega)` term.

### REQ-PLANT-006 — Quaternion norm integrity
The attitude quaternion norm shall remain unit throughout any maneuver.
- **Preconditions:** any input; a maneuver that rotates the body (e.g. a
  torque-free spin) to exercise `quat_derivative` integration.
- **Acceptance:** `max_t | norm(q(t)) - 1 | < 1e-6`.
- **Verification:** `tPlant/quaternionNormCheck`. **Status: verified.**
- **Uniquely catches:** attitude-kinematics drift / a missing in-loop
  normalization.

---

## B. Actuator and control allocation (wiring)

*Planned — specified but not yet implemented. These validate the actuator/mixing
path rather than the rigid-body dynamics.*

### REQ-PLANT-007 — Actuator thrust mapping / hover balance
The motor chain (`pwm_to_thrust` → lag → delay → saturation) shall deliver, in
steady state, the thrust commanded by the PWM input. At the hover command
(`u = thrust_to_pwm(m*g/4)`) the delivered total thrust shall balance weight.
- **Preconditions:** constant hover PWM; motor states settled (or seeded at
  hover); vehicle at rest.
- **Acceptance:** settled vertical specific force `|a_z| < 1e-2 m/s^2`
  (equivalently `|T_total - m*g| < 0.01 N`).
- **Verification:** `tPlant/hoverThrustBalancesWeight` *(planned)*.
- **Uniquely catches:** the pwm→thrust DC-gain error (the 1000× gain bug this
  requirement was written after catching).

### REQ-PLANT-008 — Motor lag and transport delay
The delivered per-motor thrust response to a PWM step shall exhibit the modeled
first-order lag and transport delay.
- **Preconditions:** step PWM command; test-only per-motor thrust outport.
- **Acceptance:** delivered thrust stays at its initial value until `t = delay`,
  then reaches `0.632 * dT` at `t = delay + tau` within 5%.
- **Verification:** `tPlant/motorLagStep` *(planned — needs the thrust outport)*.
- **Uniquely catches:** wrong `tau`/`delay`, or a lag/delay block bypassed in
  the wiring.

### REQ-PLANT-009 — Control-allocation moment signs
A single-motor thrust excess shall produce body moments of the sign dictated by
`motor_mixing_matrix` for the FRD frame.
- **Preconditions:** three motors at hover, one bumped; start from rest.
- **Acceptance:** for a motor-1 (front-right) excess,
  `omega_x < 0`, `omega_y > 0`, `omega_z < 0` (τ_x = −y_pos, τ_y = +x_pos,
  τ_z = −k*yaw_sign), each of magnitude above `1e-2 rad/s`.
- **Verification:** `tPlant/differentialThrustSigns` *(planned)*.
- **Uniquely catches:** a flipped sign in the mixing matrix — passes every
  hover/altitude check yet destabilizes the closed loop. *Note: this confirms
  internal consistency of the mixing convention; physical validation of the
  signs comes from closed-loop stability.*

### REQ-PLANT-010 — Actuation symmetry
Equal thrust on all four motors shall produce pure heave — zero net body moment
and specific force along body −z only.
- **Preconditions:** all motors at equal thrust; identity attitude; start at rest.
- **Acceptance:** `|omega(t)| < 1e-6 rad/s` (no net moment) and horizontal
  velocity `|v_xy(t)| < 1e-6 m/s` for all `t ∈ [0, 2] s`.
- **Verification:** `tPlant/actuationSymmetry` *(planned)*.
- **Uniquely catches:** an asymmetric wiring/sign error in the mixing matrix or
  motor chain that a single-motor test might miss.

---

## Requirements-verification matrix

| REQ | Title | Method | Test (`tPlant/…`) | Status |
|-----|-------|--------|-------------------|--------|
| REQ-PLANT-001 | Ballistic trajectory | Sim (analytic oracle) | `freeFall`, `ballisticTrajectory` | verified |
| REQ-PLANT-002 | Energy conservation | Sim (drift bound) | `zeroTorqueAttitude` | verified |
| REQ-PLANT-003 | Angular momentum conservation | Sim (drift bound) | `angularMomentumConservation` | verified |
| REQ-PLANT-004 | Torque-free principal-axis rotation | Sim | `singleAxisSpin` | verified |
| REQ-PLANT-005 | Gyroscopic coupling | Sim (vs Euler eqns) | `gyroscopicCoupling` | verified |
| REQ-PLANT-006 | Quaternion norm integrity | Sim | `quaternionNormCheck` | verified |
| REQ-PLANT-007 | Actuator thrust mapping / hover | Sim | `hoverThrustBalancesWeight` | planned |
| REQ-PLANT-008 | Motor lag and transport delay | Sim | `motorLagStep` | planned |
| REQ-PLANT-009 | Control-allocation moment signs | Sim | `differentialThrustSigns` | planned |
| REQ-PLANT-010 | Actuation symmetry | Sim | `actuationSymmetry` | planned |

**Status legend:** `verified` = test implemented in `vnv/tPlant.m` and passing.
`planned` = requirement defined, test not yet written. The rigid-body group
(001–006) is verified; the actuator/wiring group (007–010) is the next block of
work.
