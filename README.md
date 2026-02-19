# Hypersonic_Simulation_GHAME

Simulink implementation of the 6-DOF Generic Hypersonic Aerodynamic Model Example (GHAME).

The original GHAME implementation was developed by Peter H. Zipfel as C++ code accompanying *Modeling and Simulation of Aerospace Vehicle Dynamics, Fourth Edition*. The reference implementation can be obtained from the AIAA website:  
https://arc.aiaa.org/doi/suppl/10.2514/4.107535  

This Simulink model reproduces the original formulation. Simulation results have been cross-verified against the reference C++ implementation to ensure consistency and correctness.



## Overview

This repository contains a nonlinear six-degree-of-freedom flight dynamics simulation of the Generic Hypersonic Aerodynamic Model Example (GHAME), implemented entirely in Simulink and MATLAB.

The model represents a rigid hypersonic vehicle including full translational and rotational dynamics, Earth kinematics, aerodynamic force and moment modeling, propulsion effects, actuator dynamics, and a configurable inertial navigation system (INS). It is designed as a research and control development platform tool.

The simulation architecture is modular and structured to allow systematic analysis, controller design, uncertainty studies, and verification.

---
