function SBII = cad_in_geo84(lon, lat, alt, SMAJOR_AXIS, FLATTENING, GW_CLONG)
    % Step 1: Compute Earth's radius at the given latitude
    r0 = SMAJOR_AXIS * (1 - FLATTENING * (1 - cos(2 * lat)) / 2 + ...
         5 * FLATTENING^2 * (1 - cos(4 * lat)) / 16); % Equation 4-21

    % Step 2: Compute deflection of the normal (dd)
    dd = FLATTENING * sin(2 * lat) * (1 - FLATTENING / 2 - alt / r0); % Equation 4-15

    % Step 3: Compute geodetic displacement vector SBID in ECEF coordinates
    dbi = r0 + alt;
    SBID = [-dbi * sin(dd);
             0;
            -dbi * cos(dd)];
    
    % Step 4: Compute celestial longitude (Earth's rotation is ignored)
    lon_cel = GW_CLONG + lon; % Without time-dependent rotation

    % Step 5: Convert ECEF to ECI
    slat = sin(lat);
    clat = cos(lat);
    slon = sin(lon_cel);
    clon = cos(lon_cel);

    sbid1 = SBID(1);
    sbid2 = SBID(2);
    sbid3 = SBID(3);

    sbii1 = -slat * clon * sbid1 - clat * clon * sbid3;
    sbii2 = -slat * slon * sbid1 - clat * slon * sbid3;
    sbii3 =  clat * sbid1 - slat * sbid3;
    
    % Step 6: Return inertial position vector SBII
    SBII = [sbii1; sbii2; sbii3];
end