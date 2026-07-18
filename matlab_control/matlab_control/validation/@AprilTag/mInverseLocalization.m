function mInverseLocalization(tag, Xtag_w)
% mInverseLocalization: Calculates the CAMERA's pose in the world frame.
% This method uses the known fixed position of a Tag in the world (Xtag_w) 
% and the relative detection (Xt2c) to estimate where the robot/camera is 
% located in the global coordinate system.
%
% --- Example ---
% % Define the tag's known position on a wall (x=2m, y=0.5m, z=1.2m)
% TagInWorld = [2.0; 0.5; 1.2; 0; 0; 0]; 
% % Calculate the robot's current pose based on this tag
% RobotPose = tag.mInverseLocalization(TagInWorld);
% ---------------------
       
    % If no tag is detected, the calculation cannot be performed
    if ~tag.pFlag.Connected, return; end
    
    % Mathematical transformations using homogeneous matrices
    % T_wt: Transformation from Tag to World
    T_wt = tag.rTransformaX(Xtag_w);      

    % T_ct: Transformation from Tag to Camera (as measured by the sensor)
    % Note: pPos.Xt2c represents the relative posture [x;y;z;roll;pitch;yaw]
    T_ct = tag.rTransformaX(tag.pPos.Xt2c);  
    
    % Core Math: T_wc (Camera to World) = T_wt * inv(T_ct)
    % In MATLAB, the slash operator (/) can be used for matrix inversion/multiplication
    T_wc = T_wt / T_ct; 
    
    % Extract the camera's posture vector from the homogeneous matrix
    X_cam = tag.rTransformaT(T_wc);
    
    % Displaying the estimated robot position for debugging
    disp(['Robot (Camera) estimated pose: ', mat2str(X_cam, 2)]);
end