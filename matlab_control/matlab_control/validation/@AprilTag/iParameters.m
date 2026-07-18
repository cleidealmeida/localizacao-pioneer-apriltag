function iParameters(tag, varargin) % (tag, family, size, id)
% iParameters: Internal initialization of AprilTag parameters.
% This method defines the default structural and pose properties.

% Exemple of how to change the defaut values
% - Change the family 
%   tag = AprilTag();
%   tag.iParameters('family', 'tag25h9');
%
% - Change the size
%   tag = AprilTag();
%   tag.iParameters('size', 0.25);
%
% - Change the id
%   tag = AprilTag();
%   tag.iParameters('id', 10);
%
% - Change all
%   tag = AprilTag();
%   tag.iParameters('family', 'tag25h9', 'size', 0.25, 'id', 10);

% --- Default Values ---
    tag.pPar.family = 'tag36h11'; % Tag family (e.g., tag36h11, tag25h9)
    tag.pPar.size   = 0.105;     % Physical size of the tag in meters
    tag.pPar.id     = [];         % Default desired ID

    tag.pDetections = struct();

    tag.pDetections.id  = [];
    tag.pDetections.pose = [];

    tag.pPar.H      = eye(4);    % Homogeneous transformation matrix (Identity)
    
    % Posture Parameters (Pose vectors: [x; y; z; roll; pitch; yaw])
    tag.pPos.Xt2c = zeros(6,1);  % Relative pose: Tag with respect to the Camera
    tag.pPos.Xt2w = zeros(6,1);  % Global pose: Tag with respect to the World


   % --- User-defined Parameters (Name-Value Pairs) ---
    if mod(length(varargin),2) ~= 0
        error('Parameters must be provided in name-value pairs.');
    end

    % Parsing inputs
    for k = 1:2:length(varargin)

        name  = lower(varargin{k});
        value = varargin{k+1};

        switch name
            case 'family'
                if ~isempty(value)
                    tag.pPar.family = value;
                end

            case 'size'
                if ~isempty(value)
                    tag.pPar.size = value;
                end

            case 'id'
                if ~isempty(value)
                    tag.pPar.id = value;
                end

            otherwise
                error('Unknown parameter: %s', varargin{k});
        end
    end
end
