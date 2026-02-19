function [Sim, IC] = Initialize_Simulation(F)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% InitializeSimulation.m
%
% Builds the full simulation configuration and corresponding
% initial conditions for the nonlinear aircraft model.
%
% Outputs:
%   Sim : simulation configuration struct
%   IC  : initial condition struct
%
% Dependencies:
%   F : helper function container
%
% Written by Thomas H. Mueller, 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ========================================================================
% USER-DEFINED SIMULATION INPUTS
%
% These parameters define the simulation scenario, configuration,
% and initial conditions. Users are expected to modify ONLY this section.
% ========================================================================

% ---------------- Simulation and integration ----------------
Sim.sample_time             = 0.01;         % Integration time step [s]
Sim.simulation.duration     = 10;           % Simulation duration

% ---------------- Controller and model toggles ----------------
Sim.controller.K_sep        = 4;            % Separation factor
Sim.mode.mEarth             = 1;            % 0: spherical Earth, 1: oblate Earth

Sim.mode.mwind              = 0;            % Enable wind model
Sim.mode.mturb              = 0;            % Enable turbulence
Sim.mode.mnoise             = 0;            % Enable sensor noise
Sim.mode.mINS               = 0;            % Enable INS, NOT functional so far
Sim.mode.mperturbations     = 0;            % Enable perturbations
Sim.mode.muncertainty       = 1;            % Enable uncertain simulations

% ---------------- Actuator model ----------------
Sim.actuator.mact                       = 2;        % Actuator model selector
Sim.actuator.zeta                       = 0.707;    % Actuator damping ratio
Sim.actuator.omega                      = 50;       % Actuator natural frequency [rad/s]
Sim.actuator.deflection_limit_deg       = 20;       % Actuator deflcetion limit  [deg]
Sim.actuator.deflection_rate_limit_deg  = 150;      % Actuator rate limit        [deg/s]

% ------------------ Wind model ------------------
Sim.wind.turb_sigma         = 2.0;          % Wind turbulence intensity
Sim.wind.turb_length        = 150;          % Wind trubulence length    [m]

% ---------------- Filter models ----------------
Sim.filters.omega_AA        = 80;           % Anti-aliasing cutoff frequency [rad/s]
Sim.filters.omega_N         = 30;           % Noise filter natural frequency [rad/s]
Sim.filters.zeta_N          = 0.707;        % Noise filter damping ratio 
Sim.filters.delay_step      = 0;            % Effective actuator delay [s]

% ----------------- Uncertainty ------------------
Sim.uncertainty.N           = 5;            % Number of uncertainty runs
Sim.uncertainty.Delta       = 0.3;          % Max uniform uncertainty 
Sim.uncertainty.seed        = 67;           % Seed for reproduceablity

% ---------------- Launch location ----------------
Sim.launch.lonx             = 0;            % Geodetic longitude [deg]
Sim.launch.latx             = 0;            % Geodetic latitude  [deg]

% ---------------- Initial flight condition ----------------
Sim.initial.velocity.dvbe   = 885.1392;     % Initial airspeed [m/s]
Sim.initial.altitude.alt    = 14288;        % Initial altitude [m]

Sim.initial.attitude.psibdx = 0;            % Initial yaw   [deg]
Sim.initial.attitude.thtbdx = 2.5;          % Initial pitch [deg]
Sim.initial.attitude.phibdx = 0;            % Initial roll  [deg]

Sim.initial.aero.alpha_deg  = 2.5;          % Initial angle of attack [deg]
Sim.initial.aero.beta_deg   = 0;            % Initial sideslip angle [deg]

Sim.initial.rates.ppx       = 0;            % Initial roll rate  [deg/s]
Sim.initial.rates.qqx       = 0;            % Initial pitch rate [deg/s]
Sim.initial.rates.rrx       = 0;            % Initial yaw rate   [deg/s]

% ---------------- Vehicle mass state ----------------
Sim.vehicle.fuel.fraction   = 0.5;          % Fuel fraction (0 = empty, 1 = full)

% ----------------- Uncertainty Consistency Check ------------------

if Sim.uncertainty.N > 0 && Sim.mode.muncertainty == 0
    warning(['Uncertainty runs requested (N = %d) but ', ...
             'Sim.mode.muncertainty is disabled. ', ...
             'No Monte Carlo simulations will be executed.'], ...
             Sim.uncertainty.N);
end

if Sim.uncertainty.N == 0 && Sim.mode.muncertainty == 1
    warning(['Sim.mode.muncertainty is enabled but N = 0. ', ...
             'Only nominal simulation will be executed.']);
end

%%% Warning DO NOT change values from here onwards %%%

%% ========================================================================
% Constants
Sim.constants.PI    = 3.1415926536;
Sim.constants.DEG   = 57.2957795130823;
Sim.constants.RAD   = pi / 180;
Sim.constants.EPS   = 1e-10;
Sim.constants.SMALL = 1e-7;

RAD = Sim.constants.RAD;
DEG = Sim.constants.DEG;

Sim.simulation.Sim_Extender = 1;

%% Earth and environment model
Sim.earth.AGRAV    = 9.80675445;     % Standard gravity WGS-84
Sim.earth.C20      = -4.8416685e-4;  % Second zonal harmonic coefficient
Sim.earth.GM       = 3.9860044e14;   % Earth gravitational parameter [m^3/s^2]
Sim.earth.WEII3    = 0;              % Rotation of Earth is zero for trim
Sim.earth.GW_CLONG = 0;              % Celestial longitude 

if Sim.mode.mEarth == 0
    Sim.earth.SMAJOR_AXIS = 6370987.308;   % Approximated to REARTH [m]
    Sim.earth.REARTH      = 6370987.308;   % Mean Earth radius [m]
    Sim.earth.FLATTENING  = 0;             % No flattening for spherical Earth model
else
    Sim.earth.SMAJOR_AXIS = 6378137.0;     % WGS-84 semi-major axis (equatorial radius) [m]
    Sim.earth.REARTH      = 6370987.308;   % Mean Earth radius [m]
    Sim.earth.FLATTENING  = 3.33528106e-3; % WGS-84 flattening factor (f = (a-b)/a)
end

%% ========================================================================
% Vehicle properties

% Vehicle mass properties
Sim.vehicle.mass.total_full = 136077;  % Maximum total vehicle mass
Sim.vehicle.mass.fuel_full  = 81646;   % Maximum fuel mass

Sim.vehicle.mass.structural = ...
    Sim.vehicle.mass.total_full - Sim.vehicle.mass.fuel_full; % Structural vehicle mass

% Current fuel and vehicle mass
Sim.vehicle.mass.fuel = ...
    Sim.vehicle.fuel.fraction * Sim.vehicle.mass.fuel_full;  % Actual fuel mass

Sim.vehicle.mass.current = ...
    Sim.vehicle.mass.structural + Sim.vehicle.mass.fuel;     % Actual vehicle mass

Sim.vehicle.inertia.IBBB0 = [ ...
    1.5730e+06  0            3.8000e+05;
    0           3.1600e+07   0;
    3.8000e+05  0            3.2540e+07 ]; % MOI at full fuel

Sim.vehicle.inertia.IBBB1 = [ ...
    1.1800e+06  0            2.4000e+05;
    0           1.9625e+07   0;
    2.4000e+05  0            2.0200e+07 ]; % MOI at empty fuel

Sim.vehicle.inertia.IBBBr = ...
    Sim.vehicle.fuel.fraction     * Sim.vehicle.inertia.IBBB0 + ...
    (1-Sim.vehicle.fuel.fraction) * Sim.vehicle.inertia.IBBB1; % Actual MOI

Sim.vehicle.geometry.refa = 557.42; % Reference surface area [m]
Sim.vehicle.geometry.refb = 24.38;  % Reference span         [m]
Sim.vehicle.geometry.refc = 22.86;  % Reference chord        [m]

Sim.vehicle.acowl = 27.27; % Engine cowl area


%% ========================================================================
% ===================== INITIAL CONDITIONS ===============================
% ========================================================================

% Environment
IC.dvba = Sim.initial.velocity.dvbe;
IC.alt  = Sim.initial.altitude.alt;
IC.GW_CLONG = Sim.earth.GW_CLONG;

% Kinematics
IC.psibd = Sim.initial.attitude.psibdx * RAD;
IC.thtbd = Sim.initial.attitude.thtbdx * RAD;
IC.phibd = Sim.initial.attitude.phibdx * RAD;

IC.lon = Sim.launch.lonx*RAD;
IC.lat = Sim.launch.latx*RAD;

IC.TBD = F.mat3tr(IC.psibd, IC.thtbd, IC.phibd);
IC.TDI = F.cad_tdi84(IC.lon, IC.lat, IC.GW_CLONG);
IC.TBI = IC.TBD * IC.TDI;

% Mass and inertia
IC.vmassr = Sim.vehicle.mass.current;
IC.IBBBr = Sim.vehicle.inertia.IBBBr;

Sim.actuator.deflection_limit_rad      = Sim.actuator.deflection_limit_deg * RAD;
Sim.actuator.deflection_rate_limit_rad = Sim.actuator.deflection_rate_limit_deg * RAD;

% INS
[IC.ESBI, IC.EVBI, IC.RICI] = F.init_ins(Sim.mode.mINS, 0);

% Position and velocity
IC.WEII3 = Sim.earth.WEII3;

IC.WEII_skew = [ ...
    0 -IC.WEII3 0;
    IC.WEII3 0 0;
    0 0 0 ];

IC.SBII = F.cad_in_geo84( ...
    IC.lon, IC.lat, IC.alt, ...
    Sim.earth.SMAJOR_AXIS, Sim.earth.FLATTENING,...
    IC.GW_CLONG);

IC.alpha = Sim.initial.aero.alpha_deg * RAD;
IC.beta  = Sim.initial.aero.beta_deg  * RAD;
IC.dvbe  = Sim.initial.velocity.dvbe;

IC.VBEB = [ ...
    cos(IC.alpha)*cos(IC.beta)*IC.dvbe;
    sin(IC.beta)*IC.dvbe;
    sin(IC.alpha)*cos(IC.beta)*IC.dvbe];

IC.VBED = IC.TBD' * IC.VBEB;

IC.VBII = IC.TDI' * IC.VBED + IC.WEII_skew * IC.SBII;
IC.dvbi = F.absolute(IC.VBII);

% Flight-path angles
POLAR = F.pol_from_cart(IC.VBED);
IC.psivd = POLAR(2);
IC.thtvd = POLAR(3);

% Angular rates
IC.WBEB = [ ...
    Sim.initial.rates.ppx;
    Sim.initial.rates.qqx;
    Sim.initial.rates.rrx ] * RAD;

IC.WEII = [0; 0; IC.WEII3];

IC.WBIB = IC.WBEB + IC.TBI * IC.WEII;

% Initialize trim values
IC.dp_trim = 0;
IC.dq_trim = 0;
IC.dr_trim = 0;
IC.dT_trim = 0.05;


end
