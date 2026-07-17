function P = iProcessarEntrada(P, n)
% Mapeia o gesto reconhecido (A-E) para o alvo de posicionamento na arena.
%
% Limpezas em relação à versão anterior:
%   - REMOVIDO o P.rGetSensorData() de cada case: a leitura de sensores é
%     responsabilidade do loop principal (main_controller). No modo ARIA,
%     a chamada aqui sobrescrevia P.pPos.X com odometria crua e podia
%     anular a pose fundida do Kalman dependendo da ordem do loop.
%   - Removidos os cases duplicados comentados (código morto).
%
% Alvos na arena (referencial do mundo / tag de referência):
%   A ( 1.15,  1.15) | B ( 0.90, -0.45) | C (-2.10, -1.15)
%   D (-1.60,  1.22) | E ( 0.00,  0.00) | outro -> origem

    switch n
        case 'A'
            P.pPos.Xd(1) = 1.15;
            P.pPos.Xd(2) = 1.15;
            P.pPos.Xd(6) = 0;

        case 'B'
            P.pPos.Xd(1) = 0.9;
            P.pPos.Xd(2) = -0.45;
            P.pPos.Xd(6) = 0;

        case 'C'
            P.pPos.Xd(1) = -2.10;
            P.pPos.Xd(2) = -1.15;
            P.pPos.Xd(6) = 0;

        case 'D'
            P.pPos.Xd(1) = -1.6;
            P.pPos.Xd(2) = 1.22;
            P.pPos.Xd(6) = 0;

        case 'E'
            P.pPos.Xd(1) = 0;
            P.pPos.Xd(2) = 0;
            P.pPos.Xd(6) = 0;

        otherwise
            disp('Gesto desconhecido: retornando o Pioneer à posição inicial.');
            P.pPos.Xd(1) = 0;
            P.pPos.Xd(2) = 0;
            P.pPos.Xd(6) = 0;
    end
end
