function p3dx = cP3DX_KinematicController(p3dx,pgains)

    % Control gains
    % pgains = [1.5 1 1.5 1]; % Ganhos Daniel

    if nargin < 2

    pgains = [0.35 0.35 0.4 0.4];

    end 

    Kp1 = diag([pgains(1), pgains(2)]);
    Kp2 = diag([pgains(3), pgains(4)]);

    pPar.a = 0.1;

    K = [ cos(p3dx.pPos.X(6)), -pPar.a*sin(p3dx.pPos.X(6)); ...
          sin(p3dx.pPos.X(6)), +pPar.a*cos(p3dx.pPos.X(6))];

    p3dx.pPos.Xtil = p3dx.pPos.Xd - p3dx.pPos.X;

    p3dx.pSC.Ur = K\(p3dx.pPos.Xd(7:8) + Kp1*tanh(Kp2*p3dx.pPos.Xtil(1:2)));

    % Saturação do sinal de controle, baseado na folha de dados do Pioneer 3DX
    if abs(p3dx.pSC.Ur(1)) > 0.75
        p3dx.pSC.Ur(1) = sign(p3dx.pSC.Ur(1))*0.75;
    end
    if abs(p3dx.pSC.Ur(2)) > 1
        p3dx.pSC.Ur(2) = sign(p3dx.pSC.Ur(2))*1;
    end

    p3dx.pSC.Ud = p3dx.pSC.Ur;

end