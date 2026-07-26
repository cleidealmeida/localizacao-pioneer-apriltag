function mCADcolor(p3dx,color)
% Modify drone color

if nargin > 1
    
    %-- Pioneer3DXAssembly(old):
%     p3dx.pCAD.mtl{1}(2).Kd = color';  % 2  -> Top plat
%                                       % 9  -> Suporte rodinha traseira
%                                       % 10 -> rodinha traseira
                                   
    %-- Pioneer3DXAssembly_maisLeve:
    p3dx.pCAD.mtl{1}(2).Kd = color';  % 2,3  -> Top plat
    p3dx.pCAD.mtl{1}(3).Kd = color';  % 9    -> Suporte rodinha traseira
                                      % 10   -> rodinha traseira
                                   
end

for ii = 1:length(p3dx.pCAD.obj{1}.umat3)
    mtlnum = p3dx.pCAD.obj{1}.umat3(ii);
    for jj=1:length(p3dx.pCAD.mtl{1})
        if strcmp(p3dx.pCAD.mtl{1}(jj).name,p3dx.pCAD.obj{1}.usemtl(mtlnum-1))
            break;
        end
    end
    fvcd3(ii,:) = p3dx.pCAD.mtl{1}(jj).Kd';
end

p3dx.pCAD.i3D{1}.FaceVertexCData  = fvcd3;
end