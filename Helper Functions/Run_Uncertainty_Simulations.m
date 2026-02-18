function OUT = Run_Uncertainty_Simulations(Sim, Data, modelName)
% Run_Uncertainty_Simulations
%
% Runs nominal + Monte Carlo uncertainty simulations.
%
% Inputs:
%   Sim        : simulation configuration struct
%   CTRL       : controller struct
%   UNC        : uncertainty struct returned by Create_Uncertainty
%   modelName  : Simulink model name (string)
%
% Output:
%   OUT        : struct containing nominal and uncertain results
%

%% ------------------------------------------------------------------------
% Extract settings
warningState = warning;
warning('off','all')

N        = Sim.uncertainty.N;
Delta    = Sim.uncertainty.Delta;
seed     = Sim.uncertainty.seed;
dt       = Sim.sample_time;
Tfinal   = Sim.simulation.duration;

time_vec = 0:dt:Tfinal;
Nt       = numel(time_vec);

rng(seed)
load_system(modelName)
set_param(modelName,'FastRestart','on')
%% ------------------------------------------------------------------------
% Nominal simulation

UNC_i = Create_Uncertainty(Data.AeroCoefficients, 0, seed);

% Inject perturbed coefficients into Simulation
assignin('base','UNC',UNC_i)

NLout_nom = sim(modelName);

OUT.nom.h        = NLout_nom.Altitude(:)';
OUT.nom.alpha    = NLout_nom.Alpha(:)';
OUT.nom.speed    = NLout_nom.Speed(:)';
OUT.nom.nz       = NLout_nom.nz(:)';
OUT.nom.heading  = NLout_nom.Heading(:)';
OUT.nom.beta     = NLout_nom.Sideslip(:)';
OUT.nom.flight   = NLout_nom.Flight_Path(:)';
OUT.nom.bank     = NLout_nom.Bank(:)';
OUT.nom.throttle = NLout_nom.Throttle(:)';
OUT.nom.rates    = NLout_nom.Rates(:,:);
OUT.nom.aileron  = NLout_nom.Aileron(:)';
OUT.nom.elevator = NLout_nom.Elevator(:)';
OUT.nom.rudder   = NLout_nom.Rudder(:)';
OUT.nom.time     = time_vec;

%% ------------------------------------------------------------------------
% If uncertainty disabled → exit early

if N == 0
    OUT.unc = [];
    OUT.time = time_vec;

    % Disable Fast Restart before exiting
    set_param(modelName,'FastRestart','off')
    warning(warningState)

    return
end

%% ------------------------------------------------------------------------
% Preallocate uncertain storage

OUT.unc.h        = zeros(N,Nt);
OUT.unc.alpha    = zeros(N,Nt);
OUT.unc.speed    = zeros(N,Nt);
OUT.unc.nz       = zeros(N,Nt);
OUT.unc.heading  = zeros(N,Nt);
OUT.unc.beta     = zeros(N,Nt);
OUT.unc.flight   = zeros(N,Nt);
OUT.unc.bank     = zeros(N,Nt);
OUT.unc.throttle = zeros(N,Nt);
OUT.unc.rates    = zeros(N,3,Nt);
OUT.unc.aileron  = zeros(N,Nt);
OUT.unc.elevator = zeros(N,Nt);
OUT.unc.rudder   = zeros(N,Nt);

%% ------------------------------------------------------------------------
% Monte Carlo loop
fprintf('\nStarting Monte Carlo simulation (%d runs)\n', N);

t_total_start = tic;

for i = 1:N
    
    t_iter = tic;
    
    fprintf('Run %d / %d ... ', i, N);
    
    % Create new perturbed dataset
    UNC_i = Create_Uncertainty(Data.AeroCoefficients, Delta, seed+i);
    
    % Inject perturbed coefficients into Simulation
    assignin('base','UNC',UNC_i)
    
    NLout = sim(modelName);
    
    OUT.unc.h(i,:)        = NLout.Altitude(:)';
    OUT.unc.alpha(i,:)    = NLout.Alpha(:)';
    OUT.unc.speed(i,:)    = NLout.Speed(:)';
    OUT.unc.nz(i,:)       = NLout.nz(:)';
    OUT.unc.heading(i,:)  = NLout.Heading(:)';
    OUT.unc.beta(i,:)     = NLout.Sideslip(:)';
    OUT.unc.flight(i,:)   = NLout.Flight_Path(:)';
    OUT.unc.bank(i,:)     = NLout.Bank(:)';
    OUT.unc.throttle(i,:) = NLout.Throttle(:)';
    OUT.unc.rates(i,:,:)  = NLout.Rates(:,:);
    OUT.unc.aileron(i,:)  = NLout.Aileron(:)';
    OUT.unc.elevator(i,:) = NLout.Elevator(:)';
    OUT.unc.rudder(i,:)   = NLout.Rudder(:)';

    % ---- Timing info ----
    iter_time = toc(t_iter);
    elapsed   = toc(t_total_start);
    avg_time  = elapsed / i;
    remaining = avg_time * (N - i);
    
    fprintf('%.2f s | Est. remaining: %.1f s\n', iter_time, remaining);
    
end
total_time = toc(t_total_start);

fprintf('\nMonte Carlo completed.\n');
fprintf('Total time: %.2f seconds\n', total_time);
fprintf('Average time per run: %.2f seconds\n\n', total_time/N);

OUT.time = time_vec;

%% ------------------------------------------------------------------------
% Disable Fast Restart

set_param(modelName,'FastRestart','off')
warning(warningState)
end
