# Hypersonic_Simulation_GHAME

Simulink implementation of the 6-DOF Generic Hypersonic Aerodynamic Model Example (GHAME), a hypersonic aerodynamic model originally developed by NASA in 1989 as a simulation platform for the National AeroSpace Plane (NASP) program.

The original GHAME implementation was developed by Peter H. Zipfel as C++ code accompanying *Modeling and Simulation of Aerospace Vehicle Dynamics, Fourth Edition*. The reference implementation can be obtained from the AIAA website:  
https://arc.aiaa.org/doi/suppl/10.2514/4.107535  

This Simulink model reproduces the original formulation. Simulation results have been cross-verified against the reference C++ implementation to ensure consistency and correctness.

The trimming procedure is based on the open-source **TrimMod** package and does not require the *Simulink Control Design Toolbox*.  

The *Aerospace Blockset Toolbox* is only required for optional wind modeling functionality. This feature is commented out by default and can be activated if the toolbox is available.

The code has been tested and verified to run in MATLAB versions down to **MATLAB R2018b**.

## Overview

This repository contains a nonlinear six-degree-of-freedom flight dynamics simulation of the Generic Hypersonic Aerodynamic Model Example (GHAME), implemented entirely in Simulink and MATLAB.

The model represents a rigid hypersonic vehicle including full translational and rotational dynamics, Earth kinematics, aerodynamic force and moment modeling, propulsion effects, actuator dynamics, and a configurable inertial navigation system (INS). It is designed as a research and control development platform tool.

The simulation architecture is modular and structured to allow systematic analysis, controller design, uncertainty studies, and verification.

---
