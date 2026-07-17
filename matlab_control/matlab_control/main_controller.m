%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   CONTROLADOR DO PIONEER 3-DX — v3
%
%   Novidades da v3:
%     - DUPLA VIA DE CONEXÃO com o robô, selecionável por chave:
%         'rosaria' : tópicos ROS /pose e /cmd_vel via Jetson (como na v2)
%         'aria'    : conexão serial direta pelos mex do ARIA (sem ROS no
%                     robô). Neste modo a odometria lida do ARIA é
%                     REPUBLICADA em /pose para que o pose_estimator
%                     continue fazendo a predição do Kalman.
%     - GROUND TRUTH OPTITRACK (opcional): pose do rigid body logada em
%       paralelo; RMSE da fusão e da odometria calculados contra o
%       OptiTrack no fim (métrica central do artigo).
%     - Mantém da v2: watchdog de /robot_pose, e-stop por joystick (botão B),
%       correção do callback de /Gesture, cenários use_fused_pose.
%
%   Dependências no path: @Pioneer3DX (versão com rConnect/rGetSensorData —
%   a do pacote "Pioneer 3DX"), @JoyControl (opcional), toolbox OptiTrack
%   (opcional), iProcessarEntrada.m, cNewController.m, e os mex do ARIA no
%   path se robot_connection = 'aria'.
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

%% --- CHAVES SELETORAS ---
robot_connection = 'rosaria'; % 'rosaria' (Jetson/ROS) | 'aria' (serial direto)

use_fused_pose  = true;   % true = cenário 2 (Kalman); false = cenário 1 (odometria)
use_ros_gestures = true;  % false = lista fixa abaixo (teste sem operador)
test_gesture_list = {"A", "C", "E", "B", "D"};

use_optitrack = true;     % ground truth (arena NERO); false fora da arena
opt_rb_index  = 1;        % índice do rigid body do Pioneer no Motive

use_joystick_estop = false; % botão B para o robô na hora
pose_timeout_s = 1.0;       % watchdog de /robot_pose

%% Seção 1: Nó e Comunicação ROS 2
rosshutdown;
setenv('ROS_DOMAIN_ID', '0');

disp('Criando nó ROS 2...');
node = ros2node('/robot_controller');

% --- Gestos (callback registrado — correção da v2) ---
qos_gesture = struct('Reliability','reliable','Durability','volatile');
sub_gesture = ros2subscriber(node, '/Gesture', 'std_msgs/String', ...
    @gestureCallback, qos_gesture); %#ok<NASGU>
disp('Subscriber /Gesture criado.');

% --- Pose fundida (Kalman do pose_estimator) ---
qos_pose = struct('History','keeplast','Depth',10,'Reliability','besteffort');
sub_robot_pose = ros2subscriber(node, '/robot_pose', ...
    'geometry_msgs/PoseWithCovarianceStamped', qos_pose);
disp('Subscriber /robot_pose criado.');

% --- Via de conexão com o robô ---
switch robot_connection
    case 'rosaria'
        sub_odom = ros2subscriber(node, '/pose', 'nav_msgs/Odometry', qos_pose);
        pub_cmd_vel = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
        cmdVelMsg = ros2message(pub_cmd_vel);
        pub_odom = []; odomMsgOut = []; %#ok<NASGU>
        disp('Modo ROSARIA: /pose (Jetson) + /cmd_vel.');
    case 'aria'
        sub_odom = [];
        pub_cmd_vel = []; cmdVelMsg = [];
        % Republica a odometria do ARIA p/ o pose_estimator (predição do KF)
        pub_odom = ros2publisher(node, '/pose', 'nav_msgs/Odometry');
        odomMsgOut = ros2message(pub_odom);
        disp('Modo ARIA: serial direto; odometria republicada em /pose.');
        disp('ATENÇÃO: garanta que o RosAria da Jetson NÃO está rodando');
        disp('(dois publicadores de /pose e duas atuações = conflito).');
    otherwise
        error('robot_connection deve ser ''rosaria'' ou ''aria''.');
end

% --- CSV de métricas ---
log_filename = 'resultados_experimentos.csv';
if ~exist(log_filename, 'file')
    try
        fid = fopen(log_filename, 'w');
        fprintf(fid, ['Timestamp,Cenario,Conexao,RMSE_Odom_vs_Fusao_m,' ...
            'ErroRastr_medio_m,RMSE_Fusao_vs_GT_m,RMSE_Odom_vs_GT_m\n']);
        fclose(fid);
    catch e
        disp(['AVISO: log CSV indisponível: ' e.message]);
    end
end

%% Seção 2: Robô, Joystick e OptiTrack
P = Pioneer3DX();
P.rSetPose([0 0 0 0]);

if strcmp(robot_connection, 'aria')
    disp('Conectando ao Pioneer via ARIA...');
    P.rConnect();
    try arrobot_enable_motors; catch, end
    if ~P.pFlag.Connected
        error('Falha na conexão ARIA. Confira cabo serial/USB e drivers.');
    end
    disp('ARIA conectado, motores habilitados.');
end

% --- Joystick de emergência (opcional) ---
J = [];
if use_joystick_estop
    try
        J = JoyControl(1);
        if ~J.pIsConnected, J = []; end
    catch
        J = [];
    end
    if isempty(J)
        warning('Joystick indisponível — SEM parada de emergência manual.');
    else
        disp('E-stop armado: botão B para o robô.');
    end
end

% --- OptiTrack (ground truth) ---
OPT = [];
if use_optitrack
    try
        OPT = OptiTrack;
        OPT.Initialize;
        disp('OptiTrack inicializado (ground truth ativo).');
    catch e
        OPT = [];
        warning(['OptiTrack indisponível (' e.message '). Seguindo sem GT.']);
    end
end

%% Seção 3: Loop Principal
disp('Aguardando primeira /robot_pose (Kalman inicializa na 1ª detecção)...');
if use_fused_pose
    t_wait = tic;
    while isempty(sub_robot_pose.LatestMessage) && toc(t_wait) < 15
        pause(0.2);
    end
    if isempty(sub_robot_pose.LatestMessage)
        warning(['Sem /robot_pose após 15 s. Nós Python no ar? Tag visível? ' ...
                 'Rede OK? Seguindo em modo APENAS ODOMETRIA.']);
        use_fused_pose = false;
    end
end

disp('Iniciando loop de controle...');
tmax = 400;
t  = tic;
tp = tic;
target_reached_threshold = 0.03;

if ~use_ros_gestures
    disp('MODO DE TESTE: lista fixa de gestos.');
    gesture_queue = test_gesture_list;
end

Nmax = round(tmax / 0.1) + 100;
tempo_log     = zeros(1, Nmax);
odometria_log = zeros(3, Nmax);
fusao_log     = nan(3, Nmax);
controle_log  = zeros(3, Nmax);
alvo_log      = nan(2, Nmax);
gt_log        = nan(3, Nmax);      % ground truth OptiTrack [x;y;yaw]
timing_log    = zeros(1, Nmax);
idx_log = 1;

while toc(t) < tmax
    if toc(tp) > 0.1
        tp = tic;
        t_ciclo = tic;

        % --- 1. Odometria (conforme a via de conexão) ---
        switch robot_connection
            case 'rosaria'
                odomMsg = sub_odom.LatestMessage;
                if isempty(odomMsg)
                    disp('Aviso: sem odometria (/pose). Pulando ciclo.');
                    pause(0.1);
                    continue;
                end
                po = odomMsg.pose.pose;
                quat_o = [po.orientation.w, po.orientation.x, ...
                          po.orientation.y, po.orientation.z];
                eul_o = quat2eul(quat_o);
                pose_odom = [po.position.x; po.position.y; eul_o(1)];
            case 'aria'
                P.rGetSensorData();           % escreve em P.pPos.X
                pose_odom = [P.pPos.X(1); P.pPos.X(2); P.pPos.X(6)];
                % Republica p/ o pose_estimator (predição do Kalman)
                odomMsgOut.pose.pose.position.x = pose_odom(1);
                odomMsgOut.pose.pose.position.y = pose_odom(2);
                q = eul2quat([pose_odom(3) 0 0]);   % [w x y z]
                odomMsgOut.pose.pose.orientation.w = q(1);
                odomMsgOut.pose.pose.orientation.x = q(2);
                odomMsgOut.pose.pose.orientation.y = q(3);
                odomMsgOut.pose.pose.orientation.z = q(4);
                send(pub_odom, odomMsgOut);
        end

        % --- 1.2 Ground truth OptiTrack ---
        if ~isempty(OPT)
            rb = OPT.RigidBody;
            if numel(rb) >= opt_rb_index && rb(opt_rb_index).isTracked
                rbi = rb(opt_rb_index);
                eul_gt = quat2eul(rbi.Quaternion);
                % mesma convenção do getOptData do AuRoRA (mm->m; yaw = -eul(1))
                gt_log(:, idx_log) = [rbi.Position(1)/1000; ...
                                      rbi.Position(2)/1000; -eul_gt(1)];
            end
        end

        % --- 1.5 E-stop joystick ---
        if ~isempty(J)
            J.mRead();
            if J.pDigital(2) == 1
                disp('*** PARADA DE EMERGÊNCIA (botão B) ***');
                stopRobot(robot_connection, P, pub_cmd_vel, cmdVelMsg);
                break;
            end
        end

        % --- 2. Pose de controle (fundida com watchdog, ou odometria) ---
        pose_controle = pose_odom;
        fusedMsg = sub_robot_pose.LatestMessage;
        if ~isempty(fusedMsg)
            st = fusedMsg.header.stamp;
            msg_time = double(st.sec) + double(st.nanosec)*1e-9;
            now_ros = ros2time(node, 'now');
            now_time = double(now_ros.sec) + double(now_ros.nanosec)*1e-9;
            pose_age = now_time - msg_time;

            pf = fusedMsg.pose.pose;
            quat_f = [pf.orientation.w, pf.orientation.x, ...
                      pf.orientation.y, pf.orientation.z];
            eul_f = quat2eul(quat_f);
            pose_fusao = [pf.position.x; pf.position.y; eul_f(1)];

            if pose_age <= pose_timeout_s
                fusao_log(:, idx_log) = pose_fusao;
                if use_fused_pose
                    pose_controle = pose_fusao;
                end
            elseif use_fused_pose
                warning('WATCHDOG: /robot_pose obsoleta há %.1f s. Robô parado; degradando p/ odometria.', pose_age);
                stopRobot(robot_connection, P, pub_cmd_vel, cmdVelMsg);
                use_fused_pose = false; %#ok<NASGU>
            end
        end

        % --- 3. Fila de gestos ---
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

        % --- 4. Atuação (conforme a via) ---
        switch robot_connection
            case 'rosaria'
                cmdVelMsg.linear.x  = P.pSC.Ud(1);
                cmdVelMsg.angular.z = P.pSC.Ud(2);
                send(pub_cmd_vel, cmdVelMsg);
            case 'aria'
                P.rSendControlSignals();   % arrobot_setvel (zera Ud após enviar)
        end

        % --- 5. Logging ---
        if idx_log <= Nmax
            tempo_log(idx_log)        = toc(t);
            odometria_log(:, idx_log) = pose_odom;
            controle_log(:, idx_log)  = pose_controle;
            if ~isempty(gesture_queue)
                alvo_log(:, idx_log) = P.pPos.Xd(1:2);
            end
            timing_log(idx_log) = toc(t_ciclo);
            idx_log = idx_log + 1;
        end
    end
end

%% Seção 4: Encerramento, Gráficos e Métricas
disp('Finalizando: parando o robô...');
stopRobot(robot_connection, P, pub_cmd_vel, cmdVelMsg);
if strcmp(robot_connection, 'aria')
    try arrobot_disconnect; catch, end
    try aria_shutdown; catch, end
end
rosshutdown;
clear node sub_gesture sub_robot_pose sub_odom pub_cmd_vel pub_odom;

n = idx_log - 1;
tempo_log     = tempo_log(1:n);
odometria_log = odometria_log(:, 1:n);
fusao_log     = fusao_log(:, 1:n);
controle_log  = controle_log(:, 1:n);
alvo_log      = alvo_log(:, 1:n);
gt_log        = gt_log(:, 1:n);
timing_log    = timing_log(1:n);

if use_fused_pose
    scenario_suffix = '_Com Fusao Kalman';
else
    scenario_suffix = '_Apenas Odometria';
end
scen = strrep(scenario_suffix, '_', ' ');

% --- Posição X/Y (com ground truth) ---
figure(1);
subplot(2,1,1);
plot(tempo_log, odometria_log(1,:), 'b-', 'LineWidth', 1); hold on;
plot(tempo_log, fusao_log(1,:), 'm-', 'LineWidth', 1);
plot(tempo_log, gt_log(1,:), 'k-', 'LineWidth', 1.5);
plot(tempo_log, alvo_log(1,:), 'g--', 'LineWidth', 1.5);
title(['Posição X' scen]); xlabel('Tempo [s]'); ylabel('X [m]'); grid on;
legend('Odometria', 'Fusão (/robot\_pose)', 'OptiTrack (GT)', 'Alvo');
subplot(2,1,2);
plot(tempo_log, odometria_log(2,:), 'b-', 'LineWidth', 1); hold on;
plot(tempo_log, fusao_log(2,:), 'm-', 'LineWidth', 1);
plot(tempo_log, gt_log(2,:), 'k-', 'LineWidth', 1.5);
plot(tempo_log, alvo_log(2,:), 'g--', 'LineWidth', 1.5);
title(['Posição Y' scen]); xlabel('Tempo [s]'); ylabel('Y [m]'); grid on;
legend('Odometria', 'Fusão (/robot\_pose)', 'OptiTrack (GT)', 'Alvo');
savefig(gcf, ['Resultados_Posicao' scenario_suffix '.fig']);

% --- Trajetória no plano (visão do artigo) ---
figure(2);
plot(odometria_log(1,:), odometria_log(2,:), 'b-'); hold on;
plot(fusao_log(1,:), fusao_log(2,:), 'm-', 'LineWidth', 1.2);
plot(gt_log(1,:), gt_log(2,:), 'k-', 'LineWidth', 1.5);
axis equal; grid on;
title(['Trajetória no Plano' scen]); xlabel('X [m]'); ylabel('Y [m]');
legend('Odometria', 'Fusão', 'OptiTrack (GT)');
savefig(gcf, ['Resultados_Trajetoria' scenario_suffix '.fig']);

% --- Erro de rastreamento ---
erro_rastr = sqrt((alvo_log(1,:) - controle_log(1,:)).^2 + ...
                  (alvo_log(2,:) - controle_log(2,:)).^2);
figure(3);
plot(tempo_log, erro_rastr, 'k-', 'LineWidth', 1.5);
title(['Erro de Rastreamento' scen]);
xlabel('Tempo [s]'); ylabel('Distância ao Alvo [m]'); grid on;
savefig(gcf, ['Resultados_ErroRastreamento' scenario_suffix '.fig']);

% --- Métricas ---
disp(' ');
disp(['--- MÉTRICAS (' scen ' | conexão: ' robot_connection ') ---']);

idx_f = ~isnan(fusao_log(1,:));
if any(idx_f)
    d = odometria_log(1:2, idx_f) - fusao_log(1:2, idx_f);
    rmse_odom_fusao = sqrt(mean(sum(d.^2, 1)));
    disp(['Deriva (odometria vs fusão, RMSE): ' num2str(rmse_odom_fusao) ' m']);
else
    rmse_odom_fusao = NaN;
end

idx_g = ~isnan(gt_log(1,:));
rmse_fusao_gt = NaN; rmse_odom_gt = NaN;
if any(idx_g)
    ig = idx_g & idx_f;
    if any(ig)
        d = fusao_log(1:2, ig) - gt_log(1:2, ig);
        rmse_fusao_gt = sqrt(mean(sum(d.^2, 1)));
    end
    d = odometria_log(1:2, idx_g) - gt_log(1:2, idx_g);
    rmse_odom_gt = sqrt(mean(sum(d.^2, 1)));
    disp(['RMSE Fusão vs OptiTrack:     ' num2str(rmse_fusao_gt) ' m  <- resultado central']);
    disp(['RMSE Odometria vs OptiTrack: ' num2str(rmse_odom_gt) ' m']);
else
    disp('Sem dados de OptiTrack neste experimento.');
end
erro_rastr_medio = mean(erro_rastr, 'omitnan');
disp(['Erro de rastreamento médio: ' num2str(erro_rastr_medio) ' m']);

try
    fid = fopen(log_filename, 'a');
    fprintf(fid, '%s,%s,%s,%.4f,%.4f,%.4f,%.4f\n', ...
        datestr(now, 'yyyy-mm-dd HH:MM:SS'), strtrim(scen), robot_connection, ...
        rmse_odom_fusao, erro_rastr_medio, rmse_fusao_gt, rmse_odom_gt);
    fclose(fid);
    disp(['Métricas anexadas a ' log_filename]);
catch e
    disp(['ERRO ao salvar CSV: ' e.message]);
end

%% Funções locais
function stopRobot(mode, P, pub_cmd_vel, cmdVelMsg)
    % Para o robô pela via ativa (ROS ou ARIA).
    switch mode
        case 'rosaria'
            if ~isempty(pub_cmd_vel)
                cmdVelMsg.linear.x = 0; cmdVelMsg.angular.z = 0;
                send(pub_cmd_vel, cmdVelMsg);
            end
        case 'aria'
            P.pSC.Ud = [0; 0];
            P.rSendControlSignals();
    end
end

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
