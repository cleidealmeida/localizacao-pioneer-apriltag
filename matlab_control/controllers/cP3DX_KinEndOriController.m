function p3dx = cP3DX_KinEndOriController(p3dx,pgains,umax)

% Parâmetros de controle
if nargin < 2
    umax = 0.55; % m/s
    wmax = 0.44; % rad/s
end

% Control gains
if nargin < 2
    k_alpha = 1.0e-0; %0.44;
    k_theta = 1.0e-3;
    pgains = [k_alpha k_theta];
end

alpha_min = 1*pi/180; % rad
% alpha_min = 0.01;

% -----------------------------------------------------
% Computing the pose error
p3dx.pPos.Xtil = p3dx.pPos.Xd - p3dx.pPos.X;
        
% Modelagem cinemática em coordenada polares:
p3dx.pPos.rho = norm(p3dx.pPos.Xtil(1:2)); 
p3dx.pPos.THETA = atan2(p3dx.pPos.Xtil(2),p3dx.pPos.Xtil(1));

if abs(p3dx.pPos.THETA) > pi % Theta_til
    if p3dx.pPos.THETA > 0
        p3dx.pPos.THETA = -2*pi + p3dx.pPos.THETA;
    else 
        p3dx.pPos.THETA  = 2*pi + p3dx.pPos.THETA;
    end
end

p3dx.pPos.thetaTil = p3dx.pPos.thetad - p3dx.pPos.THETA;

if abs(p3dx.pPos.thetaTil) > pi % Theta_til
    if p3dx.pPos.thetaTil > 0
        p3dx.pPos.thetaTil = -2*pi + p3dx.pPos.thetaTil;
    else 
        p3dx.pPos.thetaTil  = 2*pi + p3dx.pPos.thetaTil;
    end
end


p3dx.pPos.alpha = p3dx.pPos.THETA - p3dx.pPos.X(6);

if abs(p3dx.pPos.alpha) > pi
    if p3dx.pPos.alpha > 0
        p3dx.pPos.alpha = -2*pi + p3dx.pPos.alpha;
    else 
        p3dx.pPos.alpha  = 2*pi + p3dx.pPos.alpha;
    end
end

% Control:
if p3dx.pPos.rho > 0.0005 % Limite para erro de posição!
    % Arquivo Controladores_Navegacao.PDF
    % Eq (2.30) -> u
    p3dx.pSC.Ud(1) = umax*tanh(p3dx.pPos.rho)*cos(p3dx.pPos.alpha);

    % Eq (2.33) -> w
    if abs(p3dx.pPos.alpha) >= alpha_min
        p3dx.pSC.Ud(2) = k_alpha*p3dx.pPos.alpha + k_theta*(p3dx.pPos.thetaTil^2)/p3dx.pPos.alpha + p3dx.pSC.Ud(1)*(sin(p3dx.pPos.alpha)/p3dx.pPos.rho) - p3dx.pSC.Ud(1)*(p3dx.pPos.thetaTil/p3dx.pPos.rho)*(sin(p3dx.pPos.alpha)/p3dx.pPos.alpha);
    end

else
    p3dx.pSC.Ud(1:2) = 0;
end


% Saturation of the control signal, based on the P3DX robot's datasheet
% if abs(p3dx.pSC.Ur(1)) > 0.75
%     p3dx.pSC.Ur(1) = sign(p3dx.pSC.Ur(1))*0.75;
% end
% if abs(p3dx.pSC.Ur(2)) > 1
%     p3dx.pSC.Ur(2) = sign(p3dx.pSC.Ur(2))*1;
% end
end