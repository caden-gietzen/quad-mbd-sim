# quad-mbd-sim

Model-based design of a quadrotor flight control system in MATLAB/Simulink — 6DOF plant, attitude/rate control laws autocoded to C via Embedded Coder and SIL-verified against the model, with parsim Monte Carlo V&V and closed-loop system identification.

## Overview

This project implements the full model-based design (MBD) pipeline used in production flight software — model, autocode, software-in-the-loop (SIL) verification, and Monte Carlo validation — applied to a quadrotor flight control system. It is built to mirror a UAV flight-software workflow end to end, not just to simulate a controller.

Control laws are designed as isolated Simulink models, separated from the plant and estimation subsystems so they can be autocoded independently. The attitude/rate control law is autocoded to C using Embedded Coder, and the generated code is verified for model-vs-generated-code equivalence in SIL.

## Key Features

- 6DOF rigid-body quadrotor plant modeled in Simulink
- Quaternion-based attitude representation throughout the model
- Attitude/rate control laws designed as an isolated Simulink model, serving as the autocode target
- Embedded Coder C code generation with model-vs-generated-code equivalence testing in SIL
- `parsim`-based Monte Carlo dispersion analysis with P99 requirement tracking
- Closed-loop system identification via two-stage ARX

## Repository Structure

```
quad-mbd-sim/
├── plant/         # 6DOF quadrotor Simulink model + parameter init scripts (quaternion math via Aerospace Toolbox)
├── control/       # attitude/rate controller model (the Embedded Coder autocode target)
├── estimation/    # quaternion EKF for attitude estimation
├── codegen/       # Embedded Coder configuration, generated C, and model-vs-code equivalence tests
├── vnv/           # parsim Monte Carlo harness and trim/linearization workflow
└── sysid/         # closed-loop system identification procedure
```

## Requirements

- MATLAB / Simulink
- Embedded Coder (C code generation and model-vs-code equivalence testing)
- Parallel Computing Toolbox (required for `parsim`-based Monte Carlo runs)
- Aerospace Toolbox (scalar-first `[w,x,y,z]` quaternion functions — `quatmultiply`, `quatconj`, `quatnorm`, `quatinv`, etc. — used in place of hand-rolled quaternion math)

## Status

### Development Roadmap

- [ ] Minimal 6DOF plant
- [ ] Attitude/rate controller as an isolated Simulink model
- [ ] Embedded Coder autocode + model-vs-code equivalence test
- [ ] `parsim` Monte Carlo V&V
- [ ] Quaternion EKF and closed-loop system identification (later)

## Conventions

**Quaternion convention:** scalar-first `[w, x, y, z]`, Hamilton convention, used consistently across all models and code in this repository. Any block, script, or generated code that consumes or produces a quaternion must follow this ordering.

## Verification Artifact

_Placeholder: model-vs-generated-C equivalence overlay plot._

Overlay of Simulink model output vs. SIL-executed generated C code output for the attitude/rate control law across the Monte Carlo dispersion set. This is the key verification artifact for the autocode pipeline — it demonstrates that the generated C code is numerically equivalent to the model it was generated from.
