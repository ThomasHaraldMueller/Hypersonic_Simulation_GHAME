function IC = Reinitialize_Simulation(IC, F)
% Reinitialize_Simulation
%
% Recomputes initial condition structure after trim.
% Uses:
%   IC.alpha_trim   [rad]
%   IC.thtbd_trim   [rad]
%
% Only kinematic and velocity-dependent quantities are updated.
%
% Inputs:
%   IC  : struct containing trim results
%   F   : helper function struct
%
% Output:
%   IC  : updated initial condition struct

%% ------------------------------------------------------------------------
% Update aerodynamic and attitude states from trim
IC.alpha = IC.alpha_trim;
IC.thtbd = IC.thtbd_trim;

% Sideslip assumed zero unless trimmed
IC.beta  = 0;

%% ------------------------------------------------------------------------
% Recompute transformation matrices
IC.TBD = F.mat3tr(IC.psibd, IC.thtbd, IC.phibd);
IC.TBI = IC.TBD * IC.TDI;

%% ------------------------------------------------------------------------
% Recompute velocity components in body coordinates
IC.VBEB = [ ...
    cos(IC.alpha)*cos(IC.beta)*IC.dvbe;
    sin(IC.beta)*IC.dvbe;
    sin(IC.alpha)*cos(IC.beta)*IC.dvbe ];

IC.VBED = IC.TBD' * IC.VBEB;

IC.WEII3 = 7.292115e-5;      % Activate Earth rotation

IC.WEII_skew = [ ...
    0 -IC.WEII3 0;
    IC.WEII3 0 0;
    0 0 0 ];

IC.VBII = IC.TDI' * IC.VBED + IC.WEII_skew * IC.SBII;

IC.dvbi = F.absolute(IC.VBII);

%% ------------------------------------------------------------------------
% Recompute flight-path angles

POLAR = F.pol_from_cart(IC.VBED);
IC.psivd =  POLAR(2);
IC.thtvd =  POLAR(3);
 
%% ------------------------------------------------------------------------
% Reset angular rates to zero for steady trim

IC.WBEB = [0; 0; 0];
IC.WBIB = IC.WBEB + IC.TBI * [0; 0; IC.WEII3];

end
