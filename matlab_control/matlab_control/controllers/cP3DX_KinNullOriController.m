function p3dx = cP3DX_KinNullOriController(p3dx,pgains,umax)

% Parâmetros de controle
if nargin < 2
    umax = 0.35; % m/s
    wmax = 0.44; % rad/s
end

% Control gains
if nargin < 2
    k_alpha = 1.2;
    k_theta = 1.5e-5;
    pgains = [k_alpha k_theta];
end

alpha_min = 10*pi/180; % rad
% alpha_min = 0.01;

% -----------------------------------------------------
% Computing the pose error
p3dx.pPos.Xtil = p3dx.pPos.Xd - p3dx.pPos.X;
        
% Modelagem cinemática em coordenada polares:
p3dx.pPos.rho = norm(p3dx.pPos.Xtil(1:2)); 
p3dx.pPos.theta = atan2(p3dx.pPos.Xtil(2),p3dx.pPos.Xtil(1));

if abs(p3dx.pPos.theta) > pi % Theta_til
    if p3dx.pPos.theta > 0
        p3dx.pPos.theta = -2*pi + p3dx.pPos.theta;
    else 
        p3dx.pPos.theta  = 2*pi + p3dx.pPos.theta;
    end
end

p3dx.pPos.alpha = p3dx.pPos.theta - p3dx.pPos.X(6);

if abs(p3dx.pPos.alpha) > pi
    if p3dx.pPos.alpha > 0
        p3dx.pPos.alpha = -2*pi + p3dx.pPos.alpha;
    else 
        p3dx.pPos.alpha  = 2*pi + p3dx.pPos.alpha;
    end
end

% Control:
if p3dx.pPos.rho > 0.0003 % Limite para erro de posição!
    % Arquivo Controladores_Navegacao.PDF
    % Eq (2.16) -> u
    p3dx.pSC.Ud(1) = umax*tanh(p3dx.pPos.rho)*cos(p3dx.pPos.alpha);

    % Eq (2.22) -> w
    if abs(p3dx.pPos.alpha) >= alpha_min
        p3dx.pSC.Ud(2) = pgains(1)*tanh(p3dx.pPos.alpha) + pgains(2)*p3dx.pPos.theta*(tanh(p3dx.pPos.theta)/p3dx.pPos.alpha) + umax*(tanh(p3dx.pPos.rho)/p3dx.pPos.rho)*cos(p3dx.pPos.alpha)*sin(p3dx.pPos.alpha) + p3dx.pPos.theta*umax*(tanh(p3dx.pPos.rho)/p3dx.pPos.rho)*(sin(p3dx.pPos.alpha)/sin(p3dx.pPos.alpha))*cos(p3dx.pPos.alpha);
    else
        p3dx.pSC.Ud(2) = pgains(1)*alpha_min + pgains(2)*p3dx.pPos.theta*(tanh(p3dx.pPos.theta)/alpha_min) + umax*p3dx.pPos.theta*(tanh(p3dx.pPos.rho)/p3dx.pPos.rho);
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