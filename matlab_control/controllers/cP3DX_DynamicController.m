function p3dx = cP3DX_DynamicController(p3dx,cgains)
% This function calculates the control signal or pioneer 3dx considering
% its dynamic variables.
%
% cgains is a 8 elements vector input with kinematic and dynamic gains:
% the first four are the kinematic gains Kcin1 and Kcin2
% the last four are the dynamic gains Kdin1 and Kdin2

% Case do not have input gains
if nargin < 2
    %     disp('Gains not given. Using standard ones.');
    cgains = [0.35 0.35 0.8 0.8 0.75 0.75 0.12 0.035];
end

% PARAMETERS ##########################################################
% Kinematic controller gains
Kcin1 = diag(cgains(1:2));    % maximum value gain
Kcin2 = diag(cgains(3:4));    % saturation gain
% Dynamic controller gain

Kdin1 = diag(cgains(5:6));    % maximum value gain
Kdin2 = diag(cgains(7:8));    % saturation gain

% Pioneer3DX initial parameters
theta = p3dx.pPar.theta; 
% theta = [0.5338; 0.2168; -0.0134; 0.9560; -0.0843; 1.0590];
% theta = [0.2604; 0.2509; -0.0005; 0.9965; 0.0026; 1.0768];

% Dynamic matrices
H = [theta(1) 0; 0 theta(2)];
C = [theta(4) -theta(3)*p3dx.pSC.U(2); theta(5)*p3dx.pSC.U(2) theta(6)];

% KINEMATIC CONTROL ##################################################

% Desired velocity - kinematic controller
p3dx.pPos.Xtil = p3dx.pPos.Xd - p3dx.pPos.X;

pPar.a = 0.1;

% Kinematic Model
K = [ cos(p3dx.pPos.X(6)), -pPar.a*sin(p3dx.pPos.X(6)); ...
      sin(p3dx.pPos.X(6)), +pPar.a*cos(p3dx.pPos.X(6))];

% p3dx.pSC.Uda = p3dx.pSC.Ud;
% Kinematic controller
if p3dx.pSC.Kinematics_control ==0
    p3dx.pSC.Ur = K\(p3dx.pPos.Xd(7:8) + Kcin1*tanh(Kcin2*p3dx.pPos.Xtil(1:2)));
else
    p3dx.pSC.Ur = p3dx.pPos.Xr(7:8);
end


% Signal control saturation [Based on Pioneer 3DX datasheet]
if abs(p3dx.pSC.Ur(1)) > 1
    p3dx.pSC.Ur(1) = sign(p3dx.pSC.Ur(1))*1;
end
if abs(p3dx.pSC.Ur(2)) > 1
    p3dx.pSC.Ur(2) = sign(p3dx.pSC.Ur(2))*1;
end

% DYNAMIC COMPENSATION ##############################################
% p3dx.pSC.dUd = (p3dx.pSC.U - p3dx.pSC.Ua)/0.1;  % acceleration
p3dx.pSC.dUd = (p3dx.pSC.U - p3dx.pSC.Ua)/p3dx.pPar.Ts;  % acceleration VALENTIM

% nu = Kdin1*tanh(Kdin2*(robo.pSC.Ud-robo.pSC.U));
nu = 0.1*tanh(0.1*p3dx.pSC.dUd) + Kdin1*tanh(Kdin2*(p3dx.pSC.Ur(1:2)-p3dx.pSC.U));

% Reference control signal
p3dx.pSC.Ud = H*nu + C*p3dx.pSC.Ur(1:2);

% Signal control saturation [Based on Pioneer 3DX datasheet]
if abs(p3dx.pSC.Ud(1)) > 1
    p3dx.pSC.Ud(1) = sign(p3dx.pSC.Ud(1))*1;
end
if abs(p3dx.pSC.Ud(2)) > 1
    p3dx.pSC.Ud(2) = sign(p3dx.pSC.Ud(2))*1;
end

% Save last velocity
p3dx.pSC.Ua = p3dx.pSC.U(1:2);

end

