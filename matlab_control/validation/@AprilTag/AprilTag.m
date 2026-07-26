classdef AprilTag < handle
    %% ===================== Classe AprilTag =====================
    % Structure compatible with AuRoRA Platform
    % Authors: Cleide and Maria
    % Note: If cameraParameters are not provided as input, a uigetfile dialog will open.

    %% --------------------- PROPRIEDADES ------------------------
    properties
        pCam        % Camera (webcam object)
        pCamParams  % Camera Parameters (cameraParameters object)
        pID         % Desired Tag ID

        pFlag       % Flags: Detection status (true/false)
        pCAD        % Structure to store CAD model, handle to plot objects, and initial vertices

        pPar        % Structure: Tag parameters (family, size, ID, H)
        pPos        % Structure: Xt2c (Relative pose: tag to camera), Xt2w (World pose: tag to world)
        pDetections
    end

    %% ----------------------- MÉTODOS ---------------------------
    methods
        % Constructor
        function tag = AprilTag(ID)
            if nargin < 1
                ID = 0;
            end
            tag.pID = ID;
            
            % Initialize with default values
            tag.iParameters(); 
            tag.iFlags();
        end

        % Initialization Parameters
        iParameters(tag, varargin);

        % Initialization of the Flags
        iFlags(tag);

        % Camera Association used in tag.pCam
        rCameraConnect(tag, camera)

        % Release Camera
        rCameraDisconnect(tag)

        % Read Camera Calibration
        mLoadCalibration(tag, file)

        % Read AprilTag
        mTagRead(tag)

        % Plot tag axes
        mTagPlot(tag, mode, I, Xcam_w)

        % Draw Axes on Tag
        I_out = mDrawAxes(tag, I)

        % Inverse Localization to find Camera
        X_cam = mInverseLocalization(tag, Xtag_w)

        % Locate in World
        mWorldLocalization(tag, Xcam_w)


        %% Plot
        mCADload(tag, modelName)
        mCADmake(tag, mode)
        mTagPlot2(tag, mode, I, Xcam_w)
    end

    methods (Access = private)
        X = mTransformToPose(obj, X) % Converte matriz homogênea em vetor de pose
        T = mTransformToMatrix(obj, T) % Converte vetor de pose em matriz homogênea
    end

end

