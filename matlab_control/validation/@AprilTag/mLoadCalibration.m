function mLoadCalibration(tag, file)
% mLoadCalibration: Loads the camera calibration parameters from a .mat file.
%
% The .mat file MUST contain the variable "cameraParams", typically generated 
% using the MATLAB Camera Calibrator app.
%
% Recommended Workflow:
%   1) Run in the Command Window: >> cameraCalibrator
%   2) Calibrate your camera using a checkerboard pattern.
%   3) In the app: Export -> Export Camera Parameters.
%   4) Save to your workspace: >> save('cameraParams.mat', 'cameraParams').
%   5) Use this method: >> tag.mLoadCalibration('cameraParams.mat').
%
% --- Example ---
% % To load calibration parameters manually:
% Tag = AprilTag(1);
% Tag.mLoadCalibration('CameraParams.mat');
% % The intrinsics will now be used by mTagRead for pose estimation.
% ---------------------

% Check if a file was provided; if not, open a selection dialog
if nargin < 2
    [filename, pathname] = uigetfile('*.mat', 'Select a Calibration File');
        if isequal(filename, 0)
            disp('User selected Cancel. Calibration not loaded.');
            return;
        end
        file = fullfile(pathname, filename);
else

    % Verify if the specified file exists on the disk
    if ~isfile(file)
        error(['Calibration file not found: ', file, newline, ...
            'Ensure you saved the parameters correctly using: save(''cameraParams.mat'')']);
    end
end

% Load the .mat file specifically looking for the 'cameraParams' variable
%load(file, 'cameraParams');
load(file, 'cameraParams_Bebop');
tag.pCamParams = cameraParams_Bebop;

disp('Camera calibration successfully loaded into tag.pCamParams');
end