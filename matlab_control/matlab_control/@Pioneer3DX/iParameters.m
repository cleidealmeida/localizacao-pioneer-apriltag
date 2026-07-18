function iParameters(p3dx)

p3dx.pPar.Model = 'P3DX'; % robot model

% Sample time
p3dx.pPar.Ts = 0.1; % For numerical integration
p3dx.pPar.ti = tic; % Flag time

% Dynamic Model Parameters 
p3dx.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration

% [kg] 
p3dx.pPar.m = 0.429; %0.442;  

% [m and rad] 
p3dx.pPar.a = 0.15;  % point of control
p3dx.pPar.alpha = 0; % angle of control

% [Identified Parameters]
% Reference: 
% Martins, F. N., & Brandão, A. S. (2018). 
% Motion Control and Velocity-Based Dynamic Compensation for Mobile Robots. 
% In Applications of Mobile Robots. IntechOpen.
% DOI: http://dx.doi.org/10.5772/intechopen.79397
p3dx.pPar.theta = [0.5338; 0.2168; -0.0134; 0.9560; -0.0843; 1.0590];