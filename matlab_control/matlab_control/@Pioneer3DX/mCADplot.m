function mCADplot(obj)
% Plot Pioneer 3D CAD model on its current position
% drone.pPos.Xc = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T

if nargin < 2
    obj.pCAD.flagLines = 0;
end


if obj.pCAD.flagCreated == 0
    mCADmake(obj)
    mCADplot(obj)
    
else
    % Update robot pose
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(obj.pPos.Xc(4)) -sin(obj.pPos.Xc(4)); 0 sin(obj.pPos.Xc(4)) cos(obj.pPos.Xc(4))];
    RotY = [cos(obj.pPos.Xc(5)) 0 sin(obj.pPos.Xc(5)); 0 1 0; -sin(obj.pPos.Xc(5)) 0 cos(obj.pPos.Xc(5))];
    RotZ = [cos(obj.pPos.Xc(6)) -sin(obj.pPos.Xc(6)) 0; sin(obj.pPos.Xc(6)) cos(obj.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot obj.pPos.Xc(1:3); 0 0 0 1];
    
    vertices = H*[obj.pCAD.obj{1}.v; ones(1,size(obj.pCAD.obj{1}.v,2))];
    obj.pCAD.i3D{1}.Vertices = vertices(1:3,:)';

    % obj.pCAD.ObjImag(4) = plot3([obj.pPos.Xc(1) obj.pPos.X(1)],[obj.pPos.Xc(2) obj.pPos.X(2)],[0.1 0.1],'-*k');
end

end

% =========================================================================
function mCADmake(obj)

obj.pCAD.i3D{1} = patch('Vertices',obj.pCAD.obj{1}.v','Faces',obj.pCAD.obj{1}.f3');

for ii = 1:length(obj.pCAD.obj{1}.umat3)
    mtlnum = obj.pCAD.obj{1}.umat3(ii);
    for jj=1:length(obj.pCAD.mtl{1})
        if strcmp(obj.pCAD.mtl{1}(jj).name,obj.pCAD.obj{1}.usemtl(mtlnum-1))
            break;
        end
    end
    fvcd3(ii,:) = obj.pCAD.mtl{1}(jj).Kd';
    %fvcd(ii,:) = rand(1,3);
end

obj.pCAD.i3D{1}.FaceVertexCData = fvcd3;
obj.pCAD.i3D{1}.FaceColor = 'flat';
obj.pCAD.i3D{1}.EdgeColor = 'none';
obj.pCAD.i3D{1}.FaceAlpha = 1;
obj.pCAD.i3D{1}.Visible = 'on';
% light;

obj.pCAD.flagCreated = 1;

% % Arrumando a orientação inicial
% 
%     Th = [0 0 -pi/2];
%     RotXini = [1 0 0; 0 cos(Th(1)) -sin(Th(1)); 0 sin(Th(1)) cos(Th(1))];
%     RotYini = [cos(Th(2)) 0 sin(Th(2)); 0 1 0; -sin(Th(2)) 0 cos(Th(2))];
%     RotZini = [cos(-Th(3)) sin(-Th(3)) 0; -sin(-Th(3)) cos(-Th(3)) 0; 0 0 1];
% 
%     Rot_ini = RotZini*RotYini*RotXini;
%     % Hd   = [Rot_elo0 obj.pPos.Xc(10:12,1); 0 0 0 1]; % Matriz Homegênea de deslocamento da peça
%     Hd   = [Rot_ini [0; +0.01; 0.19-0.015]; 0 0 0 1]; % Matriz Homegênea de deslocamento da peça
% 
%     Vert_ini = Hd*[obj.pCAD.obj.v; ones(1,length(obj.pCAD.obj.v))];
%     obj.pCAD.obj.v = Vert_ini(1:3,:);
    
drawnow
end