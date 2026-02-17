function POLAR = pol_from_cart(V)
    % Converts a 3D Cartesian vector to polar coordinates
    % Input:
    %   V - 3x1 vector [Vx; Vy; Vz]
    % Output:
    %   POLAR - 3x1 vector [magnitude; azimuth (rad); elevation (rad)]

    % Extract Cartesian components
    v1 = V(1);
    v2 = V(2);
    v3 = V(3);

    % Compute magnitude
    d = sqrt(v1^2 + v2^2 + v3^2);

    % Compute azimuth angle (angle in XY plane from X-axis)
    azimuth = atan2(v2, v1);

    % Compute elevation angle (angle from XY plane)
    denom = sqrt(v1^2 + v2^2);
    if denom > 0
        elevation = atan2(-v3, denom);
    else
        if v3 > 0
            elevation = -pi / 2;
        elseif v3 < 0
            elevation = pi / 2;
        else
            elevation = 0;
        end
    end

    % Construct output POLAR vector
    POLAR = [d; azimuth; elevation];
end