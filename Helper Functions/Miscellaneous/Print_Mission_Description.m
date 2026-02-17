function Print_Mission_Description(Sim, IC)
%PRINT_MISSION_DESCRIPTION  Print a formatted mission description to console
%
% Inputs:
%   Sim : simulation configuration struct
%   IC  : initial condition struct

    % Start description with flight conditions
    desc = sprintf("Simulation mission setup:\n");
    desc = desc + sprintf("  - Airspeed: %.1f m/s\n", IC.dvba);
    desc = desc + sprintf("  - Altitude: %.1f m\n", IC.alt);

    % Wind model
    switch Sim.mode.mwind
        case 1
            desc = desc + sprintf("  - Wind: Enabled (constant wind model)\n");
        case 2
            desc = desc + sprintf("  - Wind: Enabled (linear shear model)\n");
        otherwise
            desc = desc + sprintf("  - Wind: Disabled\n");
    end

    % Turbulence
    if Sim.mode.mturb == 1
        desc = desc + sprintf("  - Turbulence: Enabled\n");
    else
        desc = desc + sprintf("  - Turbulence: Disabled\n");
    end

    % Sensor noise
    if Sim.mode.mnoise == 1
        desc = desc + sprintf("  - Sensor noise: Enabled\n");
    else
        desc = desc + sprintf("  - Sensor noise: Disabled\n");
    end

    % Control perturbations
    if Sim.mode.mperturbations == 1
        desc = desc + sprintf("  - Control perturbations: Enabled\n");
    else
        desc = desc + sprintf("  - Control perturbations: Disabled\n");
    end

    % Print block
    fprintf("\n============================================================\n");
    fprintf("%s", desc);
    fprintf("============================================================\n\n");
end
