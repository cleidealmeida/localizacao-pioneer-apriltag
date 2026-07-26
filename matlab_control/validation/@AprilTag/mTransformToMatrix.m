function T = mTransformToMatrix(~, X)
% mTransformToMatrix: Converts a posture vector into a 4x4 homogeneous transformation matrix.
% This internal mathematical method constructs a rotation matrix using the ZYX
% Euler convention and integrates the [x; y; z] translation components.
%
% --- Example ---
% % Assuming 'X' is a posture vector [x; y; z; roll; pitch; yaw]:
% T_matrix = obj.mTransformToMatrix(X);
% % T_matrix will be the 4x4 homogeneous transformation matrix.
% ---------------------

% ------------------------------------------------------------
% Position Extraction
% ------------------------------------------------------------
% Extracts the Cartesian coordinates from the posture vector 
x = X(1);
y = X(2);
z = X(3);

% ------------------------------------------------------------
% Euler Angles Extraction (Radians/Degrees based on project setup)
% ------------------------------------------------------------
% Posture standard: phi (roll), theta (pitch), psi (yaw) 
phi   = X(4);   % roll  (X-axis)
theta = X(5);   % pitch (Y-axis)
psi   = X(6);   % yaw   (Z-axis)

% ------------------------------------------------------------
% Rotation Matrix Construction
% ------------------------------------------------------------
% eul2rotm expects angles in the order: [yaw, pitch, roll] for 'ZYX' convention.
% This convention is standard for the rigid body dynamics of UAVs.
R = eul2rotm([psi theta phi], 'ZYX');

% ------------------------------------------------------------
% Homogeneous Transformation Assembly
% ------------------------------------------------------------
% Combines the 3x3 rotation matrix and the 3x1 translation vector 
T = [ R, [x; y; z]; ...
    0  0  0  1   ];
end