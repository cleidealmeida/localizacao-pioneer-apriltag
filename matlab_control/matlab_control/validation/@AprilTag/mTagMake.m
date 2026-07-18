function mTagMake(tag)

s = tag.pPar.size/2;

%% Vértices do quadrado no frame da TAG
tag.pCAD.square0 = [ ...
    -s  s  s -s;
    -s -s  s  s;
     0  0  0  0];

tag.pCAD.square = patch( ...
    'Vertices', tag.pCAD.square0', ...
    'Faces', [1 2 3 4], ...
    'FaceColor','yellow', ...
    'FaceAlpha',0.3, ...
    'EdgeColor','k');

%% Eixos
axis_len = s;

tag.pCAD.axes0{1} = [axis_len 0 0]'; % X
tag.pCAD.axes0{2} = [0 axis_len 0]'; % Y
tag.pCAD.axes0{3} = [0 0 axis_len]'; % Z

tag.pCAD.axes{1} = plot3([0 axis_len],[0 0],[0 0],'r','LineWidth',2);
tag.pCAD.axes{2} = plot3([0 0],[0 axis_len],[0 0],'g','LineWidth',2);
tag.pCAD.axes{3} = plot3([0 0],[0 0],[0 axis_len],'b','LineWidth',2);

tag.pCAD.flagCreated = 1;

axis equal
grid on
hold on

end
