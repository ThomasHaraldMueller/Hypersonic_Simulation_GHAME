function style = PlotStyle()
% PlotStyle  Returns a structure of common plotting options
%
% Ensures consistent look across plots and supports
% color- and marker-based encodings.

style = struct();

%==========================================================
% Figure and font appearance
%==========================================================
style.FontSize      = 11;
style.FontWeight    = 'bold';
style.Interpreter   = 'latex';
style.Background    = 'w';
style.LineWidth     = 1;
style.ThinLineWidth     = 0.8;
%==========================================================
% Colormap and transparency
%==========================================================
style.Colormap      = @copper;
style.FaceAlpha     = 0.9;

%==========================================================
% Marker styles 
%==========================================================
style.MarkerList = { ...
    'o','s','^','d','v','>','<','p','h','x','+' };

style.getMarker = @(i) style.MarkerList{ ...
    mod(i-1, numel(style.MarkerList)) + 1 };

%==========================================================
% Marker sizing
%==========================================================
style.MarkerSize      = 8;    % legend markers
style.MarkerSizePlot  = 30;   % scatter SizeData

%==========================================================
% Line styles 
%==========================================================
style.LineStyleList = {'-','--',':','-.','-','--',':','-.','-'};

style.getLineStyle  = @(i) style.LineStyleList{ ...
    mod(i-1, numel(style.LineStyleList)) + 1 };

%==========================================================
% 3D view configuration
%==========================================================
style.ViewAz        = 135;
style.ViewEl        = 30;

%==========================================================
% Legend and colorbar
%==========================================================
style.LegendLoc     = 'northeast';
style.ColorbarLoc   = 'eastoutside';

%==========================================================
% Export settings 
%==========================================================
style.ExportFormats = {'pdf'};
style.ExportDPI     = 1200;     % raster resolution
style.ExportVector  = 'pdf';   % vector format

%==========================================================
% Figure size (physical, for PDF export)
%==========================================================
style.FigureWidthCm  = 10;    % single-column width
style.FigureWidthWideCm  = 18.0;    % wide / double-column style
style.FigureWidthExtraWideCm  = 24.0;    % wide / double-column style
style.FigureHeightWideCm  = 16.0;    % wide / double-column style
style.FigureHeightCm = 9;     % aspect ratio


%==========================================================
% Grayscale sweep utilities
%==========================================================
% Returns an N-by-3 RGB array ranging from light gray to black
% Usage:
%   colors = style.getGraySweep(N);
%   c = colors(i,:);
%
style.GrayLight = 0.7;   % lightest gray (0 = black, 1 = white)
style.GrayDark  = 0.0;   % darkest gray (black)

style.getGraySweep = @(N) ...
    linspace(style.GrayLight, style.GrayDark, N).' * [1 1 1];

end
