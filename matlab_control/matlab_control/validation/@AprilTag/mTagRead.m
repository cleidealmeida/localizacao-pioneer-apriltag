function mTagRead(tag)
% mTagRead: Detects the AprilTag in the image and updates its relative posture.
% This method captures a frame from the associated camera, identifies the
% tag based on its ID, and calculates the pose vector [x; y; z; roll; pitch; yaw].
%
% --- Mini Example ---
% % Assuming 'obj' is an AprilTag instance:
% obj.mTagRead();
% if obj.pDetected
%     relative_pos = obj.pPos.Xt2c(1:3); % Gets [x; y; z]
% end
% ---------------------

% 1. Initial Validations (Hardware and Calibration check)
if isempty(tag.pCam)
    error('No camera associated. Use rConnectCamera first.');
end

if isempty(tag.pCamParams)
    error('Calibration not loaded. Use mLoadCalibration first.');
end

% 2. Image Acquisition and Tag Detection
try
    % snapshot captures the frame; readAprilTag processes it
    [ids, ~, tagPose] = readAprilTag(tag.pCam.snapshot, tag.pPar.family, ...
        tag.pCamParams.Intrinsics, tag.pPar.size);
catch
    tag.pFlag.Detected = false; % Detection failed if snapshot or processing errors
    % return;
end

% 3. Data Processing
if isempty(tag.pPar.id)

    idx = 1:length(ids);

else

    idx = find(ismember(ids, tag.pPar.id));

end

tag.pDetections.ids  = ids(idx);
tag.pDetections.pose = tagPose(idx);

if isempty(idx)

    tag.pFlag.Detected = false;

    tag.pPos.Xt2c = zeros(6,1);

    return;

end

%% SALVA DETECCOES

tag.pDetections.ids  = ids(idx);
tag.pDetections.pose = tagPose(idx);

    tag.pFlag.Detected = true;

    % Extrai a pose da tag específica encontrada
    foundPose = tagPose(idx(1));

    % --- MATLAB Version Compatibility (rigidTform3d vs old formats) ---
    if isprop(foundPose, 'A')
        H = foundPose.A;      % MATLAB R2022b+
    elseif isprop(foundPose, 'T')
        H = foundPose.T';     % Older versions (Post-multiply transpose)
    else
        H = foundPose;        % Fallback for direct matrix output
    end

    tag.pPar.H = H; % Stores the 4x4 Homogeneous Matrix

    % 4. Posture Vector Extraction (Translation and Rotation)
    % Position [x; y; z] from the homogeneous transformation
    pos = H(1:3,4);

    % Rotation (Matrix to Euler ZYX)
    R = H(1:3,1:3);
    eulRad = rotm2eul(R, 'ZYX');  % Returns [Yaw(Z), Pitch(Y), Roll(X)]
    % eulDeg = eulRad * (180/pi);   % Convert to degrees for posture monitoring

    % Organize into standard posture vector: [x; y; z; roll; pitch; yaw]
    % Note: eulDeg(3) is Roll(phi), eulDeg(2) is Pitch(theta), eulDeg(1) is Yaw(psi)
    tag.pPos.Xt2c = [ ...
        pos(1);      % x
        pos(2);      % y
        pos(3);      % z
        eulRad(3);   % roll  (phi)
        eulRad(2);   % pitch (theta)
        eulRad(1)    % yaw   (psi)
        ];

end
