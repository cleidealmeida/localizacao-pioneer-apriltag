% function mCADmake(tag, mode)
% 
% if tag.pFlag.CADCreated == 1
%     return;
% end
% 
% hold on;
% 
% % tamanho físico da tag
% s = tag.pPar.size/2;
% 
% % plano centrado na origem
% X = [-s s; -s s];
% Y = [-s -s; s s];
% Z = [0 0; 0 0];
% 
% img = imread(fullfile('Tag_CADdata','april_tag_bundle_0-5.png'));
% 
% tag.pCAD.i3D{1} = surface( ...
%     'XData', X, ...
%     'YData', Y, ...
%     'ZData', Z, ...
%     'CData', img, ...
%     'FaceColor','texturemap', ...
%     'EdgeColor','none');
% 
% tag.pFlag.CADCreated = 1;
% 
% tag.pCAD.hLines{1} = plot3([tag.pPos.Xt2c(1) tag.pPar.size], [0 0], [0 0], 'r', 'LineWidth', 2); % X
% tag.pCAD.hLines{2} = plot3([0 0], [tag.pPos.Xt2c(2) tag.pPar.size], [0 0], 'g', 'LineWidth', 2); % Y
% tag.pCAD.hLines{3} = plot3([0 0], [0 0], [tag.pPos.Xt2c(3) tag.pPar.size], 'b', 'LineWidth', 2); % Z
% 
% drawnow limitrate;
% end

function mCADmake(tag, mode)

if tag.pFlag.CADCreated == 1
    return;
end

hold on;

v = tag.pCAD.obj{1}.v';
f = tag.pCAD.obj{1}.f3';

% cria o cubo na origem
tag.pCAD.i3D{1} = patch( ...
    'Vertices', v, ...
    'Faces', f, ...
    'FaceColor', [0.8 0.8 0.8], ...
    'EdgeColor', 'black', ...
    'Visible','on');

tag.pFlag.CADCreated = 1;

% ==========================================================
% Criar linhas dos eixos coordenados
% ==========================================================
origin = [0 0 0];

tag.pCAD.hLines{1} = plot3([origin(1) tag.pPar.size], [0 0], [0 0], 'r', 'LineWidth', 2); % X
tag.pCAD.hLines{2} = plot3([0 0], [origin(2) tag.pPar.size], [0 0], 'g', 'LineWidth', 2); % Y
tag.pCAD.hLines{3} = plot3([0 0], [0 0], [origin(3) tag.pPar.size], 'b', 'LineWidth', 2); % Z

drawnow limitrate;
end
