function mTagPlot(tag, mode, I, Xcam_w)
% mTagPlot2: Unified graphical method for AprilTag visualization.
% This function manages the logic of where the tag should be drawn based on
% the last detected position.
%
% mode: 1 - Overlay on Image (2D Projection)
%       2 - 3D CAD relative to Camera frame
%       3 - 3D CAD relative to World frame
%
% Following the AuRoRA platform pattern for high-level UAV modeling.

% --- PART 1: Initialization (Equivalent to flagCreated check in Bebop) ---
% Only runs if CAD objects haven't been created yet for 3D modes
if mode > 1 && tag.pFlag.CADCreated == 0
    fprintf('Chamando mCADmake com mode: %d\n', mode);
    tag.mCADmake(mode);
end

% --- PART 2: Strategy selection and Transformation Matrix (H) logic ---
% We define which transformation matrix will move the CAD vertices
switch mode
    case 1
        % Strategy 1: Project 3D axes on snapshot I
        if nargin < 3 || isempty(I)
            error('Mode 1 requires an image input (I).');
        end
        tag.mDrawAxes(I);
        return; % Mode 1 exits here as it doesn't use the 3D handles

    case 2
        % Strategy 2: 3D CAD relative to Camera
        % Uses the last relative detection matrix (Tag -> Camera)
        H = tag.pPar.H;

    case 3
        % Strategy 3: 3D CAD relative to World
        if nargin < 4, error('Camera world pose matrix (Xcam_w) required for mode 3.'); end

        % Update global posture Xt2w using the high-level localization method
        tag.mWorldLocalization(Xcam_w);

        % Convert world posture vector [x;y;z;phi;theta;psi] back to matrix H
        H = Xcam_w*tag.pPos.HT2W;

end

% --- PART 4: Vertex Transformation Logic (Unified Update) ---
% Apply homogeneous transformation to the body vertices: V_new = H * [V_original; 1]
% This follows the rigid body dynamics and kinematics standard.
V_orig = tag.pCAD.obj{1}.v;
V_homog = [V_orig; ones(1, size(V_orig, 2))];
V_transformed = H * V_homog;


% Update the Patch handle property (Nx3 format) for real-time tracking
set(tag.pCAD.i3D{1}, 'Vertices', V_transformed(1:3,:)');

% Update Axis Lines position (X=Red, Y=Green, Z=Blue)
% Extract origin from translation and transform predefined endpoints
origin = H(1:3, 4);
V_axes = H * tag.pCAD.axes0;

for i = 1:3
    set(tag.pCAD.hLines{i}, ...
        'XData', [origin(1) V_axes(1, i+1)], ...
        'YData', [origin(2) V_axes(2, i+1)], ...
        'ZData', [origin(3) V_axes(3, i+1)]);
end

drawnow limitrate;
end