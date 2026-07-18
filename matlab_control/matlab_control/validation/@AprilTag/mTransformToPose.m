function X = mTransformToPose(~, T)
% mTransformToPose: Converts a 4x4 homogeneous transformation matrix into a posture vector.
% This internal mathematical method extracts the translation components and 
% calculates Euler angles using the ZYX convention for consistency in 3D space.
%
% --- Example ---
% % Assuming 'T' is a 4x4 homogeneous matrix:
% posture_vector = obj.mTransformToPose(T); 
% % posture_vector will be [x; y; z; roll; pitch; yaw]
% ---------------------

    % ------------------------------------------------------------
    % Translation Extraction
    % ------------------------------------------------------------
    % Extracts the [x; y; z] components from the fourth column 
    t = T(1:3,4);

    % ------------------------------------------------------------
    % Euler Angles Extraction
    % ------------------------------------------------------------
    % rotm2eul returns: [yaw(psi), pitch(theta), roll(phi)] for the 'ZYX' convention.
    % This is consistent with the attitude definitions used in drone modeling.
    eul = rotm2eul(T(1:3,1:3), 'ZYX');

    % ------------------------------------------------------------
    % Reorganization to Project Standard
    % ------------------------------------------------------------
    % The output vector follows the standard posture format: [x; y; z; roll; pitch; yaw].
    X = [ ...
        t(1);        % x      (longitudinal) 
        t(2);        % y      (lateral) 
        t(3);        % z      (vertical) 
        eul(3);      % roll   (phi) 
        eul(2);      % pitch  (theta) 
        eul(1)       % yaw    (psi) 
    ];
end