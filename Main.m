%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main.m
%
% Master script for nonlinear aircraft simulation.
% This script initializes the environment, adds required helper functions,
% and runs the simulation with optional trimming and uncertainty modeling.
% Results are collected and plotted automatically.
%
% Written by Thomas H. Mueller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear

addpath('Helper Functions','Helper Functions/Miscellaneous', 'Simulink', 'TrimMod', 'Plotting')

% Figure saving options
SAVE_EPS = true;                       % Enable or disable figure export

% Load aeropropulsive data
Data = Load_Aero_Data();
    
% Function handles 
F = struct( ...
  'mat3tr',        @mat3tr, ...
  'cad_tdi84',     @cad_tdi84, ...
  'cad_tgi84',     @cad_tgi84, ...
  'cad_in_geo84',  @cad_in_geo84, ...
  'init_ins',      @init_ins, ...
  'pol_from_cart', @pol_from_cart, ...
  'absolute',      @absolute);

% Compute initial condition for trimming.
[Sim, IC] = Initialize_Simulation(F); 

% Verify if mission description fits your case
Print_Mission_Description(Sim, IC);

% Execute steady-state trim procedure.
% Argument 1: enable console output
% Argument 2: force retrim regardless of cached conditions
[Sim, IC] = Trim_Run_jj(true, false, Sim, IC);

% Recalculate Initial condition based on trimmed alpha and theta
IC = Reinitialize_Simulation(IC, F);

% Build Commands for the simulation
CMD = Build_Command_Profile(Sim, IC);

% Simulate uncertainty
UNC = Create_Uncertainty(Data, Sim.uncertainty.Delta, 67);
OUT = Run_Simulations(Sim, Data, 'GHAME_Model');

%%
% ================== Select output directory ==================
if Sim.uncertainty.N >= 1
    OUTDIR = fullfile(pwd, 'MonteCarloResults');
else
    OUTDIR = fullfile(pwd, 'SimulationResults');
end

if ~exist(OUTDIR, 'dir')
    mkdir(OUTDIR);
end

%%================== Define filename suffix ==================
suffix = sprintf('_U%d_W%d_N%d_T%d_P%d', ...
    Sim.mode.muncertainty * Sim.uncertainty.N, ...  % Number of uncertainty runs, 0 if off
    Sim.mode.mwind, ...                             % Wind model flag
    Sim.mode.mnoise, ...                            % Sensor noise flag
    Sim.mode.mturb, ...                             % Turbulence flag
    Sim.mode.mperturbations);                       % Perturbation flag

% ================== Plot and save ==================
Plot_Responses(OUT, Sim, SAVE_EPS, OUTDIR, suffix);

clearvars suffix OUTDIR





