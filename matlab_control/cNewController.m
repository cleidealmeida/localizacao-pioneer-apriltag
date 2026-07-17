function p3dx = cNewController(p3dx)
% Controlador cinemático de posicionamento com saturação tanh.
% Mesma lei de controle dos experimentos anteriores (comparabilidade
% preservada); mudanças apenas de segurança/higiene:
%   - assinatura limpa (o argumento K antigo era sobrescrito internamente);
%   - saturação final em Ud nos limites físicos do Pioneer 3-DX. Com os
%     ganhos atuais ela nunca atua (tanh já limita); protege apenas contra
%     comandos absurdos se os ganhos forem alterados no futuro.

    % Constantes do controlador
    Ka = 0.5*5;
    Kb = Ka;
    K1 = 1.06/5;
    K2 = K1;
    K  = [K1 0;
          0  K2];

    % Erro de posição
    p3dx.pPos.Xtil = p3dx.pPos.Xd - p3dx.pPos.X;

    % Função de controle saturada
    f = [tanh(Ka * p3dx.pPos.Xtil(1));
         tanh(Kb * p3dx.pPos.Xtil(2))];

    % Cinemática inversa no ponto de controle (offset a)
    A = [cos(p3dx.pPos.X(6)) -p3dx.pPar.a * sin(p3dx.pPos.X(6));
         sin(p3dx.pPos.X(6))  p3dx.pPar.a * cos(p3dx.pPos.X(6))];

    % Sinal de controle
    p3dx.pSC.Ud = A \ (p3dx.pPos.Xd([7, 8]) + K * f);

    % Saturação de segurança (limites físicos do Pioneer 3-DX)
    U_MAX = 0.75;   % [m/s]
    W_MAX = 1.75;   % [rad/s] (~100 graus/s)
    p3dx.pSC.Ud(1) = max(min(p3dx.pSC.Ud(1), U_MAX), -U_MAX);
    p3dx.pSC.Ud(2) = max(min(p3dx.pSC.Ud(2), W_MAX), -W_MAX);
end
