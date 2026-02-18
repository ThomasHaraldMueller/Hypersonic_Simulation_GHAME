function CMD = Build_Command_Profile(Sim, IC)
% Build_Command_Profile
%
% Constructs altitude commands, heading command,
% simulation time vector, and elevator perturbations.
%
% Input:
%   Sim : simulation structure
%   IC  : initial condition structure
%
% Output:
%   CMD : struct containing time vector, command signals,
%         and perturbation signals formatted for Simulink

%% ------------------------------------------------------------------------
% Time settings

dt           = Sim.sample_time;
sim_seconds  = Sim.simulation.duration;

t = 0:dt:sim_seconds;

CMD.t            = t;
CMD.dt           = dt;
CMD.sim_seconds  = sim_seconds;
CMD.time_steps   = numel(t);

%% ------------------------------------------------------------------------
% Altitude command profile

CMD.altitude.values = [
    IC.alt
    1524
   -1524
    0
    0
    0
];

CMD.altitude.step_times = [
    0
    25
    155
    0
    0
    0
];

%% ------------------------------------------------------------------------
% Heading command

CMD.heading.enable     = 0;
CMD.heading.value      = 0;
CMD.heading.step_time  = 15;

%% ------------------------------------------------------------------------
% Elevator perturbations

sigma = 0.4;

perturbTime = [sim_seconds/3 140 210 250];
ampImpulse  = [0.2 -0.24 -0.2 0];

ampConst1 = -3 * pi/180;
ampConst2 =  7 * pi/180;
ampConst3 = -4 * pi/180;

perturbElevator = zeros(size(t));

%% --- Impulse-type perturbations ----------------------------------------

if Sim.mode.mperturbations == 1
    for k = 1:length(perturbTime)
        perturbElevator = perturbElevator + ...
            ampImpulse(k) * exp(-((t - perturbTime(k)).^2)/(2*sigma^2));
    end
end

%% ------------------------------------------------------------------------
% Simulink-compatible formatting

CMD.perturbElevator.time_signal = [t(:), perturbElevator(:)];
CMD.perturbElevator.signal      = perturbElevator;

end

