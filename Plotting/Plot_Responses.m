function Plot_Responses(OUT, Sim, SAVE_EPS, OUTDIR, suffix)

style = PlotStyle();

t = OUT.time;
munc = Sim.mode.muncertainty;

%% ========================================================================
% HEADING + FLIGHT PATH
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

tiledlayout(1,2)

% --- Altitude ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.h, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.h','Color',[0 0 1 0.15])
end

xlabel('Time (s)','Interpreter',style.Interpreter)
ylabel('Altitude (m)','Interpreter',style.Interpreter)
title('Altitude Response','Interpreter',style.Interpreter)

% --- Flight Path ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.flight, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.flight','Color',[0 0 1 0.15])
end

xlabel('Time (s)','Interpreter',style.Interpreter)
ylabel('Flight Path $\gamma$ (deg)','Interpreter',style.Interpreter)
title('Flight Path Response','Interpreter',style.Interpreter)

if SAVE_EPS
    exportgraphics(gcf, ...
        fullfile(OUTDIR, ['Heading_FlightPath' suffix '.pdf']), ...
        'ContentType','vector')
end


%% ========================================================================
% AIRSPEED + THROTTLE
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

tiledlayout(1,2)

% --- Airspeed ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.speed, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.speed','Color',[0 0 1 0.15])
end

xlabel('Time (s)','Interpreter',style.Interpreter)
ylabel('Airspeed $V$ (m/s)','Interpreter',style.Interpreter)
title('Airspeed Response','Interpreter',style.Interpreter)

% --- Throttle ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.throttle, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.throttle','Color',[0 0 1 0.15])
end

xlabel('Time (s)','Interpreter',style.Interpreter)
ylabel('Throttle (-)','Interpreter',style.Interpreter)
title('Throttle Command','Interpreter',style.Interpreter)

if SAVE_EPS
    exportgraphics(gcf, ...
        fullfile(OUTDIR, ['Airspeed_Throttle' suffix '.pdf']), ...
        'ContentType','vector')
end


%% ========================================================================
% ANGLE OF ATTACK + SIDESLIP
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

tiledlayout(1,2)

% --- Angle of Attack ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.alpha, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.alpha','Color',[0 0 1 0.15])
end

xlabel('Time (s)','Interpreter',style.Interpreter)
ylabel('Angle of Attack $\alpha$ (deg)','Interpreter',style.Interpreter)
title('Angle of Attack Response','Interpreter',style.Interpreter)

% --- Sideslip ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.beta, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.beta','Color',[0 0 1 0.15])
end

yline(0,'k--','LineWidth',1)

xlabel('Time (s)','Interpreter',style.Interpreter)
ylabel('Sideslip $\beta$ (deg)','Interpreter',style.Interpreter)
title('Sideslip Response','Interpreter',style.Interpreter)

if SAVE_EPS
    exportgraphics(gcf, ...
        fullfile(OUTDIR, ['AoA_Sideslip_Response' suffix '.pdf']), ...
        'ContentType','vector')
end


%% ========================================================================
% LOAD FACTOR + PITCH RATE
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

tiledlayout(1,2)

% --- Load Factor nz ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.nz, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.nz','Color',[0 0 1 0.15])
end

xlabel('Time (s)','Interpreter',style.Interpreter)
ylabel('Load Factor $n_z$ (-)','Interpreter',style.Interpreter)
title('Load Factor Response','Interpreter',style.Interpreter)


% --- Pitch Rate q ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

% Nominal q (2nd rate)
q_nom = squeeze(OUT.nom.rates(2,:));

plot(t, q_nom, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    q_unc = squeeze(OUT.unc.rates(:,2,:));   % [N x time]
    plot(t, q_unc','Color',[0 0 1 0.15])
end

xlabel('Time (s)','Interpreter',style.Interpreter)
ylabel('Pitch Rate $q$ (deg/s)','Interpreter',style.Interpreter)
title('Pitch Rate Response','Interpreter',style.Interpreter)

if SAVE_EPS
    exportgraphics(gcf, ...
        fullfile(OUTDIR, ['LoadFactor_PitchRate' suffix '.pdf']), ...
        'ContentType','vector')
end

%% ========================================================================
% CONTROL SURFACES
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthCm style.FigureHeightWideCm]);

tiledlayout(3,1)

% --- Elevon Left ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.aileron, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.aileron','Color',[0 0 1 0.15])
end

ylabel('Aileron (deg)','Interpreter',style.Interpreter)

% --- Elevon Right ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.elevator, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.elevator','Color',[0 0 1 0.15])
end

ylabel('Elevator (deg)','Interpreter',style.Interpreter)

% --- Rudder ---
nexttile; hold on; grid on
set(gca,'FontSize',style.FontSize,'FontWeight',style.FontWeight)

plot(t, OUT.nom.rudder, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.rudder','Color',[0 0 1 0.15])
end

xlabel('Time (s)','Interpreter',style.Interpreter)
ylabel('Rudder (deg)','Interpreter',style.Interpreter)

if SAVE_EPS
    exportgraphics(gcf, ...
        fullfile(OUTDIR, ['ControlSurfaces' suffix '.pdf']), ...
        'ContentType','vector')
end

end
