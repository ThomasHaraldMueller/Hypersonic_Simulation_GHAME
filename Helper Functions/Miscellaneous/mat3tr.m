function TBD = mat3tr(psi, tht, phi)
    % Computes the transformation matrix given yaw (psi), pitch (tht), and roll (phi)

    % Compute sines and cosines of angles
    spsi = sin(psi);
    cpsi = cos(psi);
    stht = sin(tht);
    ctht = cos(tht);
    sphi = sin(phi);
    cphi = cos(phi);

    % Construct the transformation matrix
    TBD = [ cpsi*ctht,  spsi*ctht, -stht;
             cpsi*stht*sphi - spsi*cphi, spsi*stht*sphi + cpsi*cphi, ctht*sphi;
             cpsi*stht*cphi + spsi*sphi, spsi*stht*cphi - cpsi*sphi, ctht*cphi ];
end