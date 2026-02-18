function UNC = Create_Uncertainty(Data, uncertainty_level, seed)
% Create_Uncertainty
%
% Applies uniform multiplicative uncertainty to numeric fields
% in a struct. Allows zero uncertainty.
%
% Inputs:
%   Data               - struct containing nominal coefficients
%   uncertainty_level  - scalar in [0,1]
%                        0     → no uncertainty
%                        0.1   → ±10% uniform uncertainty
%   seed               - random seed for reproducibility
%
% Output:
%   UNC struct:
%       .Nominal       - original coefficients
%       .Perturbed     - uncertain coefficients
%       .Delta         - multiplicative perturbations
%

    if nargin < 2
        uncertainty_level = 0;
    end

    if nargin < 3
        seed = 1;
    end

    if uncertainty_level < 0
        error('Uncertainty level must be non-negative.');
    end

    UNC = struct();
    UNC.Nominal   = Data;
    UNC.Perturbed = Data;
    UNC.Delta     = struct();

    fields = fieldnames(Data);

    % ------------------------------------------------------------
    % Case 1: No uncertainty
    % ------------------------------------------------------------
    if uncertainty_level == 0
        for k = 1:numel(fields)
            field = fields{k};
            UNC.Delta.(field) = zeros(size(Data.(field)));
        end
        return
    end

    % ------------------------------------------------------------
    % Case 2: Apply uniform uncertainty
    % ------------------------------------------------------------
    rng(seed);

    for k = 1:numel(fields)

        field = fields{k};
        value = Data.(field);

        if isnumeric(value)

            delta = (2*uncertainty_level) .* rand(size(value)) ...
                    - uncertainty_level;

            UNC.Delta.(field)     = delta;
            UNC.Perturbed.(field) = value .* (1 + delta);

        else
            UNC.Delta.(field)     = [];
            UNC.Perturbed.(field) = value;
        end
    end
end
