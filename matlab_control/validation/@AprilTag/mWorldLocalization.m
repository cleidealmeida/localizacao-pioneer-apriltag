function mWorldLocalization(tag, X_c2w)
% mWorldLocalization: Detects the tag and calculates its global posture in the world frame.
% This method captures an image, identifies the target AprilTag, and computes
% its position and orientation relative to the world coordinate system (Xt2w)
% by combining the camera's world pose (X_c2w) with the tag's relative detection.
%
% --- Mini Example ---
% % Assuming 'obj' is an AprilTag instance and 'CamPose' is a 4x4 matrix:
% obj.mWorldLocalization(CamPose);
% if obj.pDetected
%     world_pos = obj.pPos.Xt2w; % Gets [x; y; z; roll; pitch; yaw] in world
% end
% ---------------------

% 1. Initial Validations (Check hardware association and calibration)
if isempty(tag.pCam)
    error('No camera associated. Use rConnectCamera first.');
end

if isempty(tag.pCamParams)
    error('Calibration not loaded. Use rLoadCalibration first.');
end

% 2. Image Acquisition and Tag Detection
try
    [ids, ~, tagPose] = readAprilTag(tag.pCam.snapshot, tag.pPar.family, ...
        tag.pCamParams.Intrinsics, tag.pPar.size);
catch
    tag.pFlag.Connected = false; % If snapshot or processing fails
    return;
end

% 3. Data Processing
if ~isempty(ids)
    % Search for the desired ID index (defined in tag.pPar.id)
    idx = find(ids == tag.pPar.id, 1);

    if isempty(idx)
        % Tag detected but does not match the target ID
        disp(['Tag ID ' num2str(tag.pPar.id) ' not found (other IDs were detected).']);

        tag.pFlag.Connected = false;
        tag.pPos.Xt2c = zeros(6,1);
        return;
    end

    tag.pFlag.Connected = true;

    % Extract the specific tag pose
    foundPose = tagPose(idx);

    % --- MATLAB Version Compatibility (rigidTform3d vs old formats) ---
    if isprop(foundPose, 'A')
        H = foundPose.A;      % MATLAB R2022b+
    elseif isprop(foundPose, 'T')
        H = foundPose.T';     % Older versions (Post-multiply transpose)
    else
        H = foundPose;        % Fallback
    end

    tag.pPar.H = H;           % Store relative homogeneous matrix
    T_t2c = H;                % Transformation: Tag to Camera

    % 4. Global Transformation Logic
    % Transformation: Tag to World (T_t2w = T_c2w * T_t2c)
    T_t2w = X_c2w * T_t2c;

    tag.pPos.HT2W = T_t2w; % Plot(3)'s Matrix

    % Extract global Translation [x; y; z]
    pos_w = T_t2w(1:3,4);

    % Extract global Rotation (Matrix to Euler ZYX)
    R_w = T_t2w(1:3,1:3);
    eulRad = rotm2eul(R_w, 'ZYX'); % Returns [Yaw(Z), Pitch(Y), Roll(X)]
    eulDeg_m = eulRad * (180/pi);  % Convert to degrees

    % Organize into the global posture vector: [x; y; z; roll; pitch; yaw]
    % Note: eulDeg_w(3) is Roll(phi), eulDeg_w(2) is Pitch(theta), eulDeg_w(1) is Yaw(psi)
    tag.pPos.Xt2w = [ ...
        pos_w(1);      % x
        pos_w(2);      % y
        pos_w(3);      % z
        eulDeg_m(3);   % roll  (phi)
        eulDeg_m(2);   % pitch (theta)
        eulDeg_m(1)    % yaw   (psi)
        ];
else
    tag.pFlag.Connected = false;
    %tag.pPos.X = zeros(6,1);
    disp('No AprilTag detected.');
end
end

