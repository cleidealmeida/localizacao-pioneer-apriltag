function rCameraConnect(tag, camera)
% rConnectCamera: Associates a physical or virtual camera object with the AprilTag instance.
% This method establishes the link between the tag processing logic and the image sensor.
%
% Usage:
%   tag.rConnectCamera(cam)
%
% 'camera' can be:
%   - A standard MATLAB webcam object (cam = webcam(x)).
%   - A specialized Bebop or AuRoRA camera object (e.g., B.rCamera).
%
% --- Exemplo ---
% % To associate a standard laptop webcam:
%   cam = webcam(1);
%   tag.rCameraConnect(cam);
% % Now the system is ready for calibration and detection.
% ---------------------
%
% Note: After associating the camera, calibration must be loaded separately
% using mLoadCalibration.


% ------------------------------------------------------------
% Input Validation
% ------------------------------------------------------------
if nargin < 2 || isempty(camera)
    error(['No camera provided.', newline, ...
        'Initialize the camera before associating it with the tag.', newline, ...
        'Examples:', newline, ...
        '  B = Bebop();', newline, ...
        '  tag = AprilTag()', newline, ...
        '  tag.rConnectCamera(B.rCamera);', newline, ...
        'or', newline, ...
        '  cam = webcam(1);', newline, ...
        '  tag = AprilTag();', newline, ...
        '  tag.rConnectCamera(cam);']);
end

% Direct association of the camera hardware/object
tag.pCam = camera;

% Informative Status Messages
disp('Camera successfully associated.');

% Check if calibration parameters are already bundled with the camera object
if isprop(camera, 'cameraParams') || isfield(camera, 'cameraParams')
    disp('Calibration parameters already found within the camera object.');
else
    disp('Warning: Calibration parameters not yet loaded.');
    disp('Use the cameraCalibrator app and then call mLoadCalibration.');
end
end
