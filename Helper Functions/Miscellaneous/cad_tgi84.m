function TGI = cad_tgi84(lon, lat, alt, GW_CLONG, SMAJOR_AXIS, FLATTENING)

    % Compute celestial longitude of the vehicle
    lon_cel = GW_CLONG + lon;

    % Transformation Matrix of Geodetic to Inertial coordinates (TDI)
    tdi13 = cos(lat);
    tdi33 = -sin(lat);
    tdi22 = cos(lon_cel);
    tdi21 = -sin(lon_cel);
    
    TDI = [ tdi33 * tdi22, -tdi33 * tdi21,  tdi13;
            tdi21,         tdi22,          0;
           -tdi13 * tdi22,  tdi13 * tdi21,  tdi33];

    % Compute Earth's radius to the ellipsoid surface (R0) and normal deflection (dd)
    r0 = SMAJOR_AXIS * (1 - FLATTENING * (1 - cos(2 * lat)) / 2 + ...
        5 * FLATTENING^2 * (1 - cos(4 * lat)) / 16); % Eq 4-21

    dd = FLATTENING * sin(2 * lat) * (1 - FLATTENING / 2 - alt / r0); % Eq 4-15

    % Transformation Matrix of Geographic (Geocentric) wrt Geodetic coordinates (TGD)
    TGD = [cos(dd), 0, -sin(dd);
           0,       1,  0;
           sin(dd), 0,  cos(dd)];

    % Compute Transformation Matrix of Geographic (Geocentric) wrt Inertial coordinates (TGI)
    TGI = TGD * TDI;
end

