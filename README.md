# RCSIM

A free, realistic, opensource, cross platform remote controlled models flight simulator for both model helicopters and model airplane.

## Key Features

* Supports both Desktop and Web environments.

* Supports game RC controllers / Joysticks.

* For model helicopters, models a veriatiy of aearodynamic effects, including dissimetry of lift, ground effects, heuristics for torbulation, autorotation and more. In addition, it allows turning off the 6dof gyro system during flight.

* For model airplanes, the modular design allows experimenting with unorthdox wing configurations, while modeling complex aerodynamic behaviour.

* Models the entire stack of RC components, including the input channel mixers, flight controller, gyro, servo filters and telemetry.

## How to Compile Locally

To complie and run for linux environment, just

```bash
make run_linux
```

To complie and run for the web environment, just

```bash
make run_web
```

 Note:  The build platform uses CMake, where the head `Makefile` is only used for caching the build commands.

