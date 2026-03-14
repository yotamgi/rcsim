# [RCSIM](https://yotamgi.github.io/rcsim/)

[![Build & publish WASM game](https://github.com/yotamgi/rcsim/actions/workflows/pages.yml/badge.svg)](https://github.com/yotamgi/rcsim/actions/workflows/pages.yml)

A free, open-source, cross-platform flight simulator for remote-controlled (RC) helicopters and airplanes.

The simulator is available to play directly in your web browser [here](https://yotamgi.github.io/rcsim/).

## Key Features

* **Cross-Platform**: Supports both desktop and web environments.

* **Hardware Support**: Compatible with RC controllers and joysticks.

* **Realistic Helicopter Physics**: Models a variety of aerodynamic effects, including dissymmetry of lift, ground effect, turbulence heuristics, autorotation, and more. It also features a toggleable 6-DOF gyro system.

* **Modular Airplane Design**: Allows for experimentation with unorthodox wing configurations while maintaining complex aerodynamic modeling.

* **Comprehensive RC Stack**: Simulates the entire electronics chain, including input channel mixers, flight controllers, gyros, servo filters, and telemetry.

## Compiling and running locally

To complie and run for linux environment, just

```bash
make run_linux
```

To complie and run for the web environment, just

```bash
make run_web
```

 Note: The project uses CMake for builds; the top-level Makefile is provided as a convenient wrapper for caching build commands.

