function [Sim, IC] = Trim_Run_jj(doPrint, forceTrim, Sim, IC)
% Trim_Run_jj
% Steady-state aircraft trim using jj_trim.
%
% Inputs:
%   Sim : simulation configuration struct
%   IC  : initial condition struct
%   doPrint : enable console output
%
% Outputs:
%   Sim, IC updated with trim results

% Trim_Run_jj
% Executes trimming only if relevant initial conditions have changed,
% unless forceTrim is true.

if nargin < 1 || isempty(doPrint)
    doPrint = true;
end

trimFileState = 'last_trim_state.mat';
trimFileCond  = 'last_trim_conditions.mat';

% Current trim-defining conditions

current.alt      = Sim.initial.altitude.alt;
current.dvbe     = Sim.initial.velocity.dvbe;
current.fuelFrac = Sim.vehicle.fuel.fraction;


runTrim = true;   % default: run trim

if ~forceTrim
    if exist(trimFileCond,'file') && exist(trimFileState,'file')
        
        load(trimFileCond,'lastCond')
        
        if isequal(current,lastCond)
            runTrim = false;
        
            tmp = load(trimFileState,'IC');
        
            % Only copy trim-related fields
            IC.alpha_trim = tmp.IC.alpha_trim;
            IC.thtbd_trim = tmp.IC.thtbd_trim;
            IC.dp_trim    = tmp.IC.dp_trim;
            IC.dq_trim    = tmp.IC.dq_trim;
            IC.dr_trim    = tmp.IC.dr_trim;
            IC.dT_trim    = tmp.IC.dT_trim;
        
            fprintf('Trim skipped. Using cached trim results.\n');
        end
    end
end



if runTrim
    fprintf('Running trim routine...\n');
    model = 'Trim';
    
    % ------------------------------------------------------------------------
    % Extract compiled states and ordering
    sys = linmod(model);
    
    
    stateNames = sys.StateName;
    
    % Detect state indices robustly
    
    idx_rot = find(contains(stateNames,'Inertial Body Rotation'));
    idx_pos = find(contains(stateNames,'Inertial Position'));
    idx_vel = find(contains(stateNames,'Inertial Velocity'));
    idx_vab = find(contains(stateNames,'V_alpha_beta'));
    idx_eul = find(contains(stateNames,'Euler Angles'));
    
    % Assign explicitly (no +1 assumptions)
    idx_p     = idx_rot(1);
    idx_q     = idx_rot(2);
    idx_r     = idx_rot(3);
    
    idx_z     = idx_pos(1);
    idx_y     = idx_pos(2);
    idx_x     = idx_pos(3);
    
    idx_vz    = idx_vel(1);
    idx_vy    = idx_vel(2);
    idx_vx    = idx_vel(3);
    
    idx_V     = idx_vab(1);
    idx_alpha = idx_vab(2);
    idx_beta  = idx_vab(3);
    
    idx_phi   = idx_eul(1);
    idx_theta = idx_eul(2);
    idx_psi   = idx_eul(3);
    
    
    % ------------------------------------------------------------
    % Initial conditions
    % ------------------------------------------------------------
    x0(idx_p)     = 0;
    x0(idx_q)     = 0;
    x0(idx_r)     = 0;
    
    x0(idx_z)     = IC.SBII(1);
    x0(idx_y)     = 0;
    x0(idx_x)     = 0;
    
    x0(idx_vz)    = 0;
    x0(idx_vy)    = 0;
    x0(idx_vx)    = IC.VBII(3);
    
    x0(idx_V)     = IC.dvba;
    x0(idx_alpha) = IC.alpha;
    x0(idx_beta)  = 0;
    
    x0(idx_phi)   = 0;
    x0(idx_theta) = IC.thtbd;
    x0(idx_psi)   = 0;
    
    x0 = x0';
    nx = length(x0);
    
    % ------------------------------------------------------------
    % Initial input guess
    % ------------------------------------------------------------
    u0 = [0; 0; 0; 0.05];
    nu = length(u0);
    
    % ------------------------------------------------------------
    % Desired derivatives (steady trim)
    % ------------------------------------------------------------
    d0 = zeros(nx,1);
    
    % ------------------------------------------------------------
    % Outputs (unused for trim, but required by jj_trim)
    % ------------------------------------------------------------
    ny = 1;
    y0 = zeros(ny,1);
    iy_trim = [];
    
    % ------------------------------------------------------------
    % Trim variables
    % ------------------------------------------------------------
    ix = [
        idx_alpha % alpha
        idx_theta % theta
    ];
    
    iu = [
        2      % elevator
        4      % throttle
    ];
    
    id = [
        idx_alpha     % alpha_dot = 0
        idx_q         % q_dot     = 0
        idx_vz        % Vz_dot    = 0
        idx_V         % V_dot     = 0   
    ];
    
    % Sanity check
    if length(ix) + length(iu) ~= length(id)
        error('Trim problem ill-posed');
    end
    
    % ------------------------------------------------------------
    % Names (diagnostics)
    % ------------------------------------------------------------
    x_nam = cellstr(stateNames);
    u_nam = {'Aileron_Com';'Elevator_Com';'Rudder_Com';'Throttle_Com'};
    d_nam = strcat(x_nam,'_dot');
    
    % ------------------------------------------------------------
    % Input limits
    % ------------------------------------------------------------
    ctrlLim = Sim.actuator.deflection_limit_deg * pi/180;
    u_min = [-ctrlLim; -ctrlLim; -ctrlLim; 0.05];
    u_max = [ ctrlLim;  ctrlLim;  ctrlLim; 2];
    
    % ------------------------------------------------------------
    % State bounds
    % ------------------------------------------------------------
    x_min = -inf(nx,1);
    x_max =  inf(nx,1);
    x_min(2) = -3  * pi/180;
    x_max(2) = 21  * pi/180;
    
    % ------------------------------------------------------------
    % jj_trim options
    % ------------------------------------------------------------
    options.n_iter_max   = 50;
    options.cost_tbg     = 1e-9;
    options.CompileFlag  = 1;
    options.EnableMessages = 1;
    
    % ------------------------------------------------------------
    % Execute trim
    % ------------------------------------------------------------
    if doPrint
        fprintf('jj_trim: %s\n', model);
        fprintf('nx = %d, nu = %d, ny = %d\n', nx, nu, ny);
    end
    
    [x_tr, u_tr, ~, ~, err, info] = jj_trim( ...
        model, ...
        x0, u0, d0, y0, ...
        ix, iu, id, iy_trim, ...
        x_nam, u_nam, d_nam, {}, ...
        [], [], [], [], ...
        options, ...
        x_min, x_max, u_min, u_max );
    
    if err ~= 0
        fprintf('\n===== jj_trim FAILED =====\n');
        fprintf('ExitCode: %d\n', info.ExitCode);
        error('jj_trim failed');
    end
    
    % ------------------------------------------------------------
    % Store trim results
    % ------------------------------------------------------------
    IC.alpha_trim = x_tr(idx_alpha);
    IC.thtbd_trim = x_tr(idx_theta);
    
    IC.dp_trim = u_tr(1);
    IC.dq_trim = u_tr(2);
    IC.dr_trim = u_tr(3);
    IC.dT_trim = u_tr(4);
    
    % ------------------------------------------------------------
    % Report
    % ------------------------------------------------------------
    if doPrint
        fprintf('\nTrim successful\n');
        fprintf('alpha   : %.3f deg\n', rad2deg(IC.alpha_trim));
        fprintf('Throttle: %.3f\n',     IC.dT_trim);
        fprintf('Elevator: %.3f deg\n\n',   rad2deg(IC.dq_trim));
    end

    % Save trimmed state
    lastCond = current; %#ok<NASGU>
    save(trimFileCond,'lastCond')
    save(trimFileState,'Sim','IC')
end

end
