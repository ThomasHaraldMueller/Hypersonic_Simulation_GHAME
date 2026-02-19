function Print_Mission_Description(Sim, IC)
%PRINT_MISSION_DESCRIPTION  Print a formatted mission description to console
%
% Inputs:
%   Sim : simulation configuration struct
%   IC  : initial condition struct

    % Start description with flight conditions
    desc = sprintf("Simulation mission setup:\n");
    desc =     desc + sprintf("  - Airspeed:           %.1f m/s\n", IC.dvba);
    desc =     desc + sprintf("  - Altitude:           %.1f m\n", IC.alt);
    desc =     desc + sprintf("  - Fuel fraction:      %.1f \n\n", Sim.vehicle.fuel.fraction); 

    desc = desc + sprintf("Simulation modes:\n");
    % Wind model
    if Sim.mode.mwind == 1
        desc = desc + sprintf("  - Wind:               Enabled (constant wind model)\n");
    else
        desc = desc + sprintf("  - Wind:               Disabled\n");
    end

    % Turbulence
    if Sim.mode.mturb == 1
        desc = desc + sprintf("  - Turbulence:         Enabled\n");
    else
        desc = desc + sprintf("  - Turbulence:         Disabled\n");
    end

    % Sensor noise
    if Sim.mode.mnoise == 1
        desc = desc + sprintf("  - Sensor noise:       Enabled\n");
    else
        desc = desc + sprintf("  - Sensor noise:       Disabled\n");
    end 

    % Control perturbations
    if Sim.mode.mperturbations == 1
        desc = desc + sprintf("  - Act. perturbations: Enabled\n");
    else
        desc = desc + sprintf("  - Act. perturbations: Disabled\n");
    end

    if Sim.mode.muncertainty == 1
        desc = desc + sprintf("  - Uncertainty:        Enabled\n");
        desc = desc + sprintf("  - Monte Carlo:        %d runs\n", Sim.uncertainty.N);
    else
        desc = desc + sprintf("  - Uncertainty:        Disabled\n");
    end


    % Print block
    fprintf("\n============================================================\n");
    fprintf("%s", desc);
    fprintf("============================================================\n\n");
end
