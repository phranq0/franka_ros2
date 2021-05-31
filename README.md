# A ROS2 Package for Continuous Franka Panda Control

## Features (TODOs)
- [X] Add `VELOCITY_PRIOR` and `TIME_LIMIT` control mode for continuos loop
- [ ] Add `auto-recovery` and `motion/wrench threshold` to avoid damaging robot or reaching singularity
- [ ] Allow different types of control request: single `destination` command for simple linear motion; `waypoints` command for path following (via interpolation)
- [ ] Allow hybrid torque control
- [ ] Allow customized physical contact behaviours (only for inbuilt sensors)
- [ ] Add `lock` and `unlock` robot joints functions
- [ ] Enable runtime robot-ip change

## Remarks
- Motion planning should be achieved outside of the library, this library only aims at smooth low-level control
- Sensory fusion should be done outside of this library, as with any other perceptual/cognitive unit