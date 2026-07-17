# Conventions (frames, units, interfaces)

Authoritative definition of the coordinate frames, units, quaternion
convention, state layout, and actuator interface used across `quad-mbd-sim`.
This is the project's interface-control document: every model, script, and
generated file must conform, and code comments should **reference** this
document rather than restate it.

## Quick reference

| Quantity | Convention |
|---|---|
| World frame | **NED** — North, East, Down. `+z = down`, altitude = `−z`. |
| Gravity | `g_world = [0, 0, +9.81] m/s^2` (points down, `+z`). |
| Body frame | **FRD** — x-forward, y-right, z-down. Right-handed. |
| Thrust | Along body `−z` (up), opposing gravity. |
| Attitude | Unit quaternion, **scalar-first `[w x y z]`, Hamilton**, body→world. |
| Angles / rates | radians, rad/s. |
| Linear units | metres, m/s, newtons, N·m, kg, seconds (SI). |
| PWM | normalized `0..1` per motor. |
| Vector shape | MATLAB row vectors `(1,N)`, enforced by `arguments` blocks. |

## 1. Reference frames

### World — NED
North-East-Down inertial frame. `+z` points **down**, so a vehicle at altitude
`h` has world position `z = −h`. Gravity is therefore `[0, 0, +9.81] m/s^2`.
World position `r` and world velocity `v` are expressed here.

### Body — FRD
Forward-Right-Down, fixed to the airframe:

- `+x` forward (nose)
- `+y` right
- `+z` down

This is right-handed: `x̂ × ŷ = ẑ` ⇒ forward × right = down ✓. Body angular
rate `omega` is expressed here. Rotor thrust acts along body **`−z`** (up),
which is why `plant()` applies `quat_rotate(q, [0,0,−T_total])`.

This is the standard aerospace / PX4-internal convention. NED world + FRD body
means the world and body z-axes are aligned (both down) at level hover.

## 2. Attitude / quaternion convention

- **Scalar-first, `[w, x, y, z]`, Hamilton convention**, everywhere — models,
  scripts, and generated C. No mixed conventions between plant, control, or
  estimation.
- The attitude quaternion `q` is the **body→world** rotation:
  `quat_rotate(q, v_body)` returns the vector expressed in the world (NED)
  frame. Equivalently `q` is the orientation of the body frame as seen from
  the world frame.
- `q` shall stay unit-norm; `quat_derivative(q, omega)` uses the body-frame
  rate `omega` (see REQ-PLANT-006 for the norm-integrity requirement).
- **Euler angles**, where used, are stored as the vector `[roll, pitch, yaw]`
  (`[φ, θ, ψ]`, the standard aerospace storage order) and compose in the
  **3-2-1 / Z-Y-X sequence**: yaw about z, then pitch about y, then roll about x
  (the standard aerospace "yaw-pitch-roll" Tait-Bryan sequence). Storage order
  and rotation sequence are independent — the angle *vector* lists roll first,
  while the *rotation* applies yaw first. `common/euler_to_quat.m` /
  `common/quat_to_euler.m` implement this sequence and are authoritative.
- Quaternion math is hand-written and codegen-safe in `common/`
  (`quat_multiply`, `quat_conjugate`, `quat_inverse`, `quat_normalize`,
  `quat_rotate`, `quat_error`, `quat_derivative`, `quat_to_rotvec`,
  `quat_to_rotmat`/`quat_from_rotmat`, `quat_to_euler`/`euler_to_quat`).
  Aerospace Toolbox quaternion functions are **not** used inside any MATLAB
  Function block (not code-generation compatible).

## 3. State vector

The plant state is the 13-element vector `[r(3) v(3) q(4) omega(3)]`:

| Index | Symbol | Meaning | Frame |
|-------|--------|---------|-------|
| 1:3 | `r` | position | world (NED), `+z` down |
| 4:6 | `v` | linear velocity | world (NED) |
| 7:10 | `q` | attitude `[w x y z]` | body→world |
| 11:13 | `omega` | angular velocity | body (FRD) |

`vnv/unpack_state.m` is the single point that maps this layout to named fields;
consumers index state through it, not by raw column number.

## 4. Actuator and mixing conventions

### PWM input
Each motor is commanded by a normalized PWM value `u ∈ [0, 1]`; `u = [u1 u2 u3
u4]`. The static map to thrust is the inverse pair
`plant/pwm_to_thrust.m` / `control/thrust_to_pwm.m`:
`T = min_thrust + (max_thrust − min_thrust) * u`. The pwm→thrust DC gain of the
motor chain must equal `(max_thrust − min_thrust)`; see REQ-PLANT-007.

### Motor chain (in `plant/`, vehicle hardware)
`u (1,4)` PWM → static pwm→thrust map → first-order lag (`tau`) → transport
delay (`delay`) → saturation (`min_thrust`/`max_thrust`) → `T (1,4)` delivered
thrust. The rigid-body dynamics take `T`, never raw `u`.

### Motor numbering and X-layout
X configuration, PX4 quad-X numbering and spin:

- motor **1** = front-right, CW
- motor **2** = rear-left, CW
- motor **3** = front-left, CCW
- motor **4** = rear-right, CCW

`plant/motor_mixing_matrix.m` is the authoritative source for the numeric
allocation matrix `A`, mapping per-motor thrust `[T1;T2;T3;T4]` to the body
wrench `[F; tau_x; tau_y; tau_z]`. In FRD with thrust along `−z`, the torque
rows are `tau_x = −y_pos`, `tau_y = +x_pos`, and the yaw-reaction row is
sign-flipped versus a z-up build (`−k*yaw_sign`).

## 5. Where the conventions are enforced

| Convention | Enforced in |
|---|---|
| NED gravity / FRD thrust direction | `plant/plant.m` (`F_gravity`, `F_thrust_world`) |
| FRD mixing (motor layout, torque signs) | `plant/motor_mixing_matrix.m` |
| pwm↔thrust map | `plant/pwm_to_thrust.m`, `control/thrust_to_pwm.m` |
| State layout | `vnv/unpack_state.m` |
| Quaternion math | `common/*.m` |
| Frame-dependent test oracles | `vnv/tPlant.m` |

## 6. Current conformance

All modules conform to NED/FRD:

- **Plant:** `plant/plant.m`, `plant/motor_mixing_matrix.m`, `control/control_allocator.m`.
- **Verification:** `vnv/tPlant.m`, `vnv/unpack_state.m`.
- **References:** `vnv/reference_trajectory.m`, `vnv/reference_params.m` — altitudes
  are stored as positive heights and converted at use (`z = -altitude`).
- **Controllers:** `control/position_controller.m`, `control/attitude_from_thrust.m`
  are frame-converted (NED gravity compensation, FRD thrust axis) but remain
  **functional placeholders** — the control logic (integrator states, attitude
  error) is not yet implemented. Frame-correctness is not the same as a working
  controller; these are not closed-loop-validated.
