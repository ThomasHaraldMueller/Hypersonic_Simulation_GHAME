function TDI = cad_tdi84(lon, lat, GW_CLONG)

    % Compute celestial longitude of the vehicle at simulation 'time'
    lon_cel = GW_CLONG + lon;

    % Compute transformation matrix elements
    tdi13 = cos(lat);
    tdi33 = -sin(lat);
    tdi22 = cos(lon_cel);
    tdi21 = -sin(lon_cel);

    % Construct the transformation matrix
    TDI = [ tdi33 * tdi22, -tdi33 * tdi21, tdi13;
            tdi21,         tdi22,          0;
           -tdi13 * tdi22,  tdi13 * tdi21, tdi33 ];
end

