# quad-mbd-sim

Model-based design of a quadrotor flight control system in MATLAB/Simulink — a 6DOF plant with attitude/rate control laws to be autocoded to C via Embedded Coder and SIL-verified against the model, with parsim Monte Carlo V&V and closed-loop system identification planned.

## Overview

This project implements the full model-based design (MBD) pipeline used in production flight software — model, autocode, software-in-the-loop (SIL) verification, and Monte Carlo validation — applied to a quadrotor flight control system. It is built to mirror a UAV flight-software workflow end to end, not just to simulate a controller.

Control laws will be designed as isolated Simulink models, separated from the plant and estimation subsystems so they can be autocoded independently. The attitude/rate control law will be autocoded to C using Embedded Coder, with the generated code verified for model-vs-generated-code equivalence in SIL.

## Key Features

- 6DOF rigid-body quadrotor plant modeled in Simulink
- Quaternion-based attitude representation throughout the model
- Attitude/rate control laws to be designed as an isolated Simulink model, serving as the autocode target
- Embedded Coder C code generation with model-vs-generated-code equivalence testing in SIL (planned)
- `parsim`-based Monte Carlo dispersion analysis with P99 requirement tracking (planned)
- Closed-loop system identification via two-stage ARX (planned)

## Repository Structure

```
quad-mbd-sim/
├── common/        # shared, codegen-safe math utilities (quaternion library) used across all model folders
├── plant/         # 6DOF quadrotor Simulink model + parameter init scripts
├── control/       # attitude/rate controller model (the Embedded Coder autocode target)
├── estimation/    # quaternion EKF for attitude estimation
├── codegen/       # Embedded Coder configuration, generated C, and model-vs-code equivalence tests
├── vnv/           # reference trajectory generator, parsim Monte Carlo harness, and trim/linearization workflow
└── sysid/         # closed-loop system identification procedure
```

## Requirements

- MATLAB / Simulink
- Embedded Coder (C code generation and model-vs-code equivalence testing)
- Parallel Computing Toolbox (required for `parsim`-based Monte Carlo runs)
- Aerospace Toolbox (used for scalar-first `[w,x,y,z]` quaternion utilities in analysis/scripting code only — `common/` hand-rolls codegen-safe quaternion math for use inside MATLAB Function blocks, since Aerospace Toolbox's `quatmultiply`, `quatconj`, etc. are not code-generation compatible)

## Status

### Development Roadmap

- [x] Minimal 6DOF plant
- [ ] Attitude/rate controller as an isolated Simulink model
- [ ] Embedded Coder autocode + model-vs-code equivalence test
- [ ] `parsim` Monte Carlo V&V
- [ ] Quaternion EKF and closed-loop system identification (later)

## Conventions

**Quaternion convention:** scalar-first `[w, x, y, z]`, Hamilton convention, used consistently across all models and code in this repository. Any block, script, or generated code that consumes or produces a quaternion must follow this ordering.

## Verification Artifact

_Placeholder: model-vs-generated-C equivalence overlay plot._

Overlay of Simulink model output vs. SIL-executed generated C code output for the attitude/rate control law across the Monte Carlo dispersion set. This is the key verification artifact for the autocode pipeline — it demonstrates that the generated C code is numerically equivalent to the model it was generated from.
