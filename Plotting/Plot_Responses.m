function Plot_Responses(OUT, Sim, SAVE_EPS, OUTDIR, suffix)

style = PlotStyle();
t     = OUT.time;
munc  = Sim.mode.muncertainty;

%% LaTeX defaults (2018b compatible)
set(groot,'defaultTextInterpreter','latex')
set(groot,'defaultAxesTickLabelInterpreter','latex')
set(groot,'defaultLegendInterpreter','latex')

%% ========================================================================
% HEADING + FLIGHT PATH
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

subplot(1,2,1); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.h, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.h','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Altitude (m)')
title('Altitude Response')

subplot(1,2,2); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.flight, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.flight','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Flight Path $\gamma$ (deg)')
title('Flight Path Response')

if SAVE_EPS
    savePDF(gcf, fullfile(OUTDIR,['Heading_FlightPath' suffix]))
end

%% ========================================================================
% AIRSPEED + THROTTLE
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

subplot(1,2,1); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.speed, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.speed','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Airspeed $V$ (m/s)')
title('Airspeed Response')

subplot(1,2,2); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.throttle, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.throttle','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Throttle (-)')
title('Throttle Command')

if SAVE_EPS
    savePDF(gcf, fullfile(OUTDIR,['Airspeed_Throttle' suffix]))
end

%% ========================================================================
% ANGLE OF ATTACK + SIDESLIP
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

subplot(1,2,1); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.alpha, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.alpha','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Angle of Attack $\alpha$ (deg)')
title('Angle of Attack Response')

subplot(1,2,2); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.beta, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.beta','Color',[0.6 0.6 1])
end

plot([t(1) t(end)],[0 0],'k--')

xlabel('Time (s)')
ylabel('Sideslip $\beta$ (deg)')
title('Sideslip Response')

if SAVE_EPS
    savePDF(gcf, fullfile(OUTDIR,['AoA_Sideslip_Response' suffix]))
end

%% ========================================================================
% LOAD FACTOR + PITCH RATE
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

subplot(1,2,1); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.nz, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.nz','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Load Factor $n_z$')
title('Load Factor Response')

subplot(1,2,2); hold on; grid on
set(gca,'FontSize',style.FontSize)

q_nom = squeeze(OUT.nom.rates(2,:));
plot(t, q_nom, 'r', 'LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    q_unc = squeeze(OUT.unc.rates(:,2,:));
    plot(t, q_unc','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Pitch Rate $q$ (deg/s)')
title('Pitch Rate Response')

if SAVE_EPS
    savePDF(gcf, fullfile(OUTDIR,['LoadFactor_PitchRate' suffix]))
end

%% ========================================================================
% CONTROL SURFACES
% ========================================================================

figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthCm style.FigureHeightWideCm]);

subplot(3,1,1); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.aileron, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.aileron','Color',[0.6 0.6 1])
end
ylabel('Aileron (deg)')

subplot(3,1,2); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.elevator, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.elevator','Color',[0.6 0.6 1])
end
ylabel('Elevator (deg)')

subplot(3,1,3); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.rudder, 'r', 'LineWidth',style.LineWidth)
if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.rudder','Color',[0.6 0.6 1])
end
xlabel('Time (s)')
ylabel('Rudder (deg)')

if SAVE_EPS
    savePDF(gcf, fullfile(OUTDIR,['ControlSurfaces' suffix]))
end

end
function savePDF(figHandle, filename)

pos = get(figHandle,'Position');

set(figHandle,'PaperUnits','centimeters')
set(figHandle,'PaperPositionMode','auto')
set(figHandle,'PaperSize',[pos(3) pos(4)])

print(figHandle, filename, '-dpdf')

end
