%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   CONTROLADOR DO PIONEER 3-DX — VERSÃO INTEGRADA À LOCALIZAÇÃO EXTERNA
%
%   Arquitetura (2 máquinas na mesma rede):
%     Ubuntu 22.04 / ROS 2 Humble  ->  publica /robot_pose e /Gesture
%     Windows / MATLAB (este script) -> assina os dois e publica /cmd_vel
%
%   Mudanças em relação à versão anterior:
%     - REMOVIDA toda a pilha de visão do MATLAB (webcam, readAprilTag,
%       w_H_c, r_H_t, cameraParams). A localização agora vem pronta do
%       pose_estimator (Python) pelo tópico /robot_pose.
%     - CORRIGIDO o bug do subscriber de gestos: a versão antiga definia
%       gestureCallback mas NUNCA o registrava no ros2subscriber — no modo
%       ROS a fila de gestos jamais era alimentada. Agora o callback é
%       passado explicitamente na criação do subscriber.
%     - Removido o profile FastDDS de WSL2 (duas máquinas físicas em LAN
%       usam a descoberta multicast padrão; ver network/SETUP.md).
%     - Chave de experimento: use_fused_pose seleciona entre odometria pura
%       (cenário 1) e pose fundida do Kalman (cenário 2), mantendo os
%       gráficos e o CSV de métricas para o artigo.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Limpeza e Configuração de Ambiente
clearvars; close all; clc;
global gesture_queue;
gesture_queue = {};

FolderCurrent = which(mfilename);
FolderKey = '\IC 2024';
FolderRootId = strfind(FolderCurrent, FolderKey);
if ~isempty(FolderRootId)
    FolderRoot = FolderCurrent(1:FolderRootId(end)+numel(FolderKey)-1);
    addpath(genpath(FolderRoot))
end

%% --- CHAVES SELETORAS DE EXPERIMENTO ---
use_fused_pose  = true;   % true  = cenário 2: controla com /robot_pose (Kalman)
                          % false = cenário 1: controla só com odometria /pose
use_ros_gestures = true;  % false = usa a lista fixa abaixo p/ testes sem operador
test_gesture_list = {"A", "C", "E", "B", "D"};

%% Seção 1: Nó e Comunicação ROS 2
rosshutdown;
setenv('ROS_DOMAIN_ID', '0');   % mesmo valor da máquina Ubuntu
% Sem FASTRTPS_DEFAULT_PROFILES_FILE: em duas máquinas físicas na mesma
% sub-rede a descoberta multicast padrão do FastDDS é suficiente. Se a
% descoberta falhar, siga o diagnóstico do network/SETUP.md (firewall do
% Windows e multicast do roteador) antes de recorrer a profiles.

disp('Criando nó ROS 2...');
node = ros2node('/robot_controller');
disp('Nó criado.');

% --- SUBSCRIBER DE GESTOS (com callback DE FATO registrado) ---
qos_gesture = struct('Reliability', 'reliable', 'Durability', 'volatile');
sub_gesture = ros2subscriber(node, '/Gesture', 'std_msgs/String', ...
    @gestureCallback, qos_gesture); %#ok<NASGU>
disp('Subscriber /Gesture criado (callback registrado).');

% --- SUBSCRIBER DA POSE FUNDIDA (Kalman, vinda do pose_estimator) ---
qos_pose = struct('History','keeplast', 'Depth',10, 'Reliability','besteffort');
sub_robot_pose = ros2subscriber(node, '/robot_pose', ...
    'geometry_msgs/PoseWithCovarianceStamped', qos_pose);
disp('Subscriber /robot_pose criado.');

% --- SUBSCRIBER DA ODOMETRIA BRUTA (RosAria) ---
sub_odom = ros2subscriber(node, '/pose', 'nav_msgs/Odometry', qos_pose);
disp('Subscriber /pose (odometria) criado.');

% --- PUBLISHER DE COMANDO ---
pub_cmd_vel = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
cmdVelMsg = ros2message(pub_cmd_vel);
disp('Publisher /cmd_vel criado.');

% --- LOG DE RESULTADOS (CSV p/ o artigo) ---
log_filename = 'resultados_experimentos.csv';
if ~exist(log_filename, 'file')
    try
        fid = fopen(log_filename, 'w');
        fprintf(fid, 'Timestamp,Cenario,RMSE_Odom_vs_Fusao_m,ErroRastreamento_medio_m\n');
        fclose(fid);
    catch e
        disp(['AVISO: log CSV indisponível: ' e.message]);
    end
end

%% Seção 2: Robô
P = Pioneer3DX();
P.rSetPose([0 0 0 0]);

%% Seção 3: Loop Principal de Controle
disp('Aguardando primeira mensagem de /robot_pose (Kalman inicializa na 1ª detecção)...');
if use_fused_pose
    % espera ativa curta: o pose_estimator só publica após a 1ª detecção
    t_wait = tic;
    while isempty(sub_robot_pose.LatestMessage) && toc(t_wait) < 15
        pause(0.2);
    end
    if isempty(sub_robot_pose.LatestMessage)
        warning(['Sem /robot_pose após 15 s. Verifique: nós Python no ar? ' ...
                 'Tag do robô visível? Rede OK (network/SETUP.md)? ' ...
                 'Seguindo em modo APENAS ODOMETRIA.']);
        use_fused_pose = false;
    end
end

disp('Iniciando loop de controle...');
tmax = 400;
t  = tic;
tp = tic;
target_reached_threshold = 0.03;   % [m]

if ~use_ros_gestures
    disp('MODO DE TESTE: lista fixa de gestos, ignorando /Gesture.');
    gesture_queue = test_gesture_list;
end

% --- Logging ---
Nmax = round(tmax / 0.1) + 100;
tempo_log        = zeros(1, Nmax);
odometria_log    = zeros(3, Nmax);
fusao_log        = nan(3, Nmax);
controle_log     = zeros(3, Nmax);
alvo_log         = nan(2, Nmax);
timing_ciclo_log = zeros(1, Nmax);
idx_log = 1;

while toc(t) < tmax
    if toc(tp) > 0.1
        tp = tic;
        t_ciclo = tic;

        % --- 1. Leitura da odometria (sempre, p/ comparação) ---
        odomMsg = sub_odom.LatestMessage;
        if isempty(odomMsg)
            disp('Aviso: sem odometria. Pulando ciclo.');
            pause(0.1);
            continue;
        end
        po = odomMsg.pose.pose;
        quat_o = [po.orientation.w, po.orientation.x, ...
                  po.orientation.y, po.orientation.z];
        eul_o = quat2eul(quat_o);            % [yaw pitch roll]
        pose_odom = [po.position.x; po.position.y; eul_o(1)];

        % --- 2. Pose de controle: fundida (/robot_pose) ou odometria ---
        pose_controle = pose_odom;
        fusedMsg = sub_robot_pose.LatestMessage;
        if ~isempty(fusedMsg)
            pf = fusedMsg.pose.pose;
            quat_f = [pf.orientation.w, pf.orientation.x, ...
                      pf.orientation.y, pf.orientation.z];
            eul_f = quat2eul(quat_f);
            pose_fusao = [pf.position.x; pf.position.y; eul_f(1)];
            fusao_log(:, idx_log) = pose_fusao;
            if use_fused_pose
                pose_controle = pose_fusao;
            end
        end

        % --- 3. Lógica de controle com fila de gestos ---
        if ~isempty(gesture_queue)
            current_gesture = gesture_queue{1};
            P = iProcessarEntrada(P, current_gesture);
            target = P.pPos.Xd(1:2);

            if norm(pose_controle(1:2) - target) < target_reached_threshold
                disp(['Alvo do gesto "' char(current_gesture) '" alcançado!']);
                gesture_queue(1) = [];
                if isempty(gesture_queue)
                    disp('Fila vazia. Robô vai parar.');
                    P.pSC.Ud = [0; 0];
                else
                    disp(['Próximo alvo: gesto "' char(gesture_queue{1}) '".']);
                end
            end
        else
            P.pSC.Ud = [0; 0];
        end

        P.pPos.X(1) = pose_controle(1);
        P.pPos.X(2) = pose_controle(2);
        P.pPos.X(6) = pose_controle(3);

        if ~isempty(gesture_queue)
            P = cNewController(P);
        end

        % --- 4. Atuação ---
        cmdVelMsg.linear.x  = P.pSC.Ud(1);
        cmdVelMsg.angular.z = P.pSC.Ud(2);
        send(pub_cmd_vel, cmdVelMsg);

        % --- 5. Logging ---
        if idx_log <= Nmax
            tempo_log(idx_log)       = toc(t);
            odometria_log(:, idx_log)= pose_odom;
            controle_log(:, idx_log) = pose_controle;
            if ~isempty(gesture_queue)
                alvo_log(:, idx_log) = P.pPos.Xd(1:2);
            end
            timing_ciclo_log(idx_log) = toc(t_ciclo);
            idx_log = idx_log + 1;
        end
    end
end

%% Seção 4: Encerramento e Gráficos
disp('Finalizando: parando o robô...');
cmdVelMsg.linear.x = 0; cmdVelMsg.angular.z = 0;
send(pub_cmd_vel, cmdVelMsg);
rosshutdown;
clear node sub_gesture sub_robot_pose sub_odom pub_cmd_vel;

n = idx_log - 1;
tempo_log        = tempo_log(1:n);
odometria_log    = odometria_log(:, 1:n);
fusao_log        = fusao_log(:, 1:n);
controle_log     = controle_log(:, 1:n);
alvo_log         = alvo_log(:, 1:n);
timing_ciclo_log = timing_ciclo_log(1:n);

if use_fused_pose
    scenario_suffix = '_Com Fusao Kalman';
else
    scenario_suffix = '_Apenas Odometria';
end
scen = strrep(scenario_suffix, '_', ' ');

% --- Posição X/Y ---
figure(1);
subplot(2,1,1);
plot(tempo_log, odometria_log(1,:), 'b-', 'LineWidth', 1); hold on;
plot(tempo_log, fusao_log(1,:), 'm-', 'LineWidth', 1);
plot(tempo_log, controle_log(1,:), 'r-', 'LineWidth', 1.5);
plot(tempo_log, alvo_log(1,:), 'g--', 'LineWidth', 1.5);
title(['Posição X' scen]); xlabel('Tempo [s]'); ylabel('X [m]'); grid on;
legend('Odometria', 'Fusão (/robot\_pose)', 'Pose de Controle', 'Alvo');
subplot(2,1,2);
plot(tempo_log, odometria_log(2,:), 'b-', 'LineWidth', 1); hold on;
plot(tempo_log, fusao_log(2,:), 'm-', 'LineWidth', 1);
plot(tempo_log, controle_log(2,:), 'r-', 'LineWidth', 1.5);
plot(tempo_log, alvo_log(2,:), 'g--', 'LineWidth', 1.5);
title(['Posição Y' scen]); xlabel('Tempo [s]'); ylabel('Y [m]'); grid on;
legend('Odometria', 'Fusão (/robot\_pose)', 'Pose de Controle', 'Alvo');
savefig(gcf, ['Resultados_Posicao' scenario_suffix '.fig']);

% --- Erro de rastreamento ---
erro_rastr = sqrt((alvo_log(1,:) - controle_log(1,:)).^2 + ...
                  (alvo_log(2,:) - controle_log(2,:)).^2);
figure(2);
plot(tempo_log, erro_rastr, 'k-', 'LineWidth', 1.5);
title(['Erro de Rastreamento' scen]);
xlabel('Tempo [s]'); ylabel('Distância ao Alvo [m]'); grid on;
savefig(gcf, ['Resultados_ErroRastreamento' scenario_suffix '.fig']);

% --- Tempo de ciclo ---
figure(3);
plot(tempo_log, timing_ciclo_log * 1000, 'c-', 'LineWidth', 1);
title(['Tempo de Ciclo do Controlador' scen]);
xlabel('Tempo [s]'); ylabel('Duração [ms]'); grid on;
savefig(gcf, ['Resultados_TempoCiclo' scenario_suffix '.fig']);

% --- Métricas ---
disp(' ');
disp(['--- MÉTRICAS (' scen ') ---']);
idx_v = ~isnan(fusao_log(1,:));
if any(idx_v)
    dif = odometria_log(1:2, idx_v) - fusao_log(1:2, idx_v);
    rmse_odom_fusao = sqrt(mean(sum(dif.^2, 1)));
    disp(['Divergência odometria vs fusão (RMSE): ' ...
          num2str(rmse_odom_fusao) ' m  (mede a DERIVA da odometria)']);
else
    rmse_odom_fusao = NaN;
    disp('Sem dados de /robot_pose registrados neste experimento.');
end
erro_rastr_medio = mean(erro_rastr, 'omitnan');
disp(['Erro de rastreamento médio: ' num2str(erro_rastr_medio) ' m']);

try
    fid = fopen(log_filename, 'a');
    fprintf(fid, '%s,%s,%.4f,%.4f\n', ...
        datestr(now, 'yyyy-mm-dd HH:MM:SS'), strtrim(scen), ...
        rmse_odom_fusao, erro_rastr_medio);
    fclose(fid);
    disp(['Métricas anexadas a ' log_filename]);
catch e
    disp(['ERRO ao salvar CSV: ' e.message]);
end

%% Callback de Gestos (registrado no ros2subscriber da Seção 1)
function gestureCallback(msg)
    global gesture_queue;
    new_gesture = msg.data;
    if isempty(gesture_queue) || ~strcmp(gesture_queue{end}, new_gesture)
        gesture_queue{end+1} = new_gesture;
        disp(['Gesto "' char(new_gesture) '" recebido. Fila: ' ...
              num2str(numel(gesture_queue))]);
    else
        disp(['Gesto "' char(new_gesture) '" duplicado em sequência. Ignorado.']);
    end
end
