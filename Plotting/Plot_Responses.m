function Plot_Responses(OUT, Sim, SAVE_EPS, OUTDIR, suffix)

style = PlotStyle();
t     = OUT.time(:);           % ensure column
munc  = Sim.mode.muncertainty;

%% LaTeX defaults (works in older MATLAB)
set(0,'defaultTextInterpreter','latex')
set(0,'defaultAxesTickLabelInterpreter','latex')
set(0,'defaultLegendInterpreter','latex')

%% ========================================================================
% ALTITUDE + FLIGHT PATH
% ========================================================================

fig = figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

subplot(1,2,1); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.h(:),'r','LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.h','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Altitude (m)')
title('Altitude Response')

subplot(1,2,2); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.flight(:),'r','LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.flight','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Flight Path $\gamma$ (deg)')
title('Flight Path Response')

if SAVE_EPS
    savePDF(fig, fullfile(OUTDIR,['Altitude_FlightPath' suffix]))
end

%% ========================================================================
% AIRSPEED + THROTTLE
% ========================================================================

fig = figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

subplot(1,2,1); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.speed(:),'r','LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.speed','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Airspeed $V$ (m/s)')
title('Airspeed Response')

subplot(1,2,2); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.throttle(:),'r','LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.throttle','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Throttle (-)')
title('Throttle Command')

if SAVE_EPS
    savePDF(fig, fullfile(OUTDIR,['Airspeed_Throttle' suffix]))
end

%% ========================================================================
% LOAD FACTOR + PITCH RATE
% ========================================================================

fig = figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthExtraWideCm style.FigureHeightCm]);

subplot(1,2,1); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.nz(:),'r','LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.nz','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Load Factor $n_z$')
title('Load Factor Response')

subplot(1,2,2); hold on; grid on
set(gca,'FontSize',style.FontSize)

q_nom = OUT.nom.rates(2,:)';   % 3xNt → row 2 → column
plot(t, q_nom,'r','LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    q_unc = squeeze(OUT.unc.rates(:,2,:));   % N x Nt
    plot(t, q_unc','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Pitch Rate $q$ (deg/s)')
title('Pitch Rate Response')

if SAVE_EPS
    savePDF(fig, fullfile(OUTDIR,['LoadFactor_PitchRate' suffix]))
end

%% ========================================================================
% CONTROL SURFACES
% ========================================================================

fig = figure('Color',style.Background,...
       'Units','centimeters',...
       'Position',[5 5 style.FigureWidthCm style.FigureHeightWideCm]);

subplot(3,1,1); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.aileron(:),'r','LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.aileron','Color',[0.6 0.6 1])
end
ylabel('Aileron (deg)')

subplot(3,1,2); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.elevator(:),'r','LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.elevator','Color',[0.6 0.6 1])
end
ylabel('Elevator (deg)')

subplot(3,1,3); hold on; grid on
set(gca,'FontSize',style.FontSize)

plot(t, OUT.nom.rudder(:),'r','LineWidth',style.LineWidth)

if munc && ~isempty(OUT.unc)
    plot(t, OUT.unc.rudder','Color',[0.6 0.6 1])
end

xlabel('Time (s)')
ylabel('Rudder (deg)')

if SAVE_EPS
    savePDF(fig, fullfile(OUTDIR,['ControlSurfaces' suffix]))
end

end

function savePDF(figHandle, filename)

set(figHandle,'PaperUnits','centimeters')
set(figHandle,'PaperPositionMode','auto')

print(figHandle, filename, '-dpdf','-bestfit','-painters')

end