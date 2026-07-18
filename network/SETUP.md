# Configuração de rede — Ubuntu (percepção) ↔ Windows (controle)

Duas máquinas físicas na mesma rede local, comunicando via ROS 2 (DDS).

## Checklist antes de rodar qualquer nó do robô

- [ ] Rede **cabeada** nas duas máquinas (multicast DDS sofre em Wi-Fi).
- [ ] Multicast liberado no roteador (desligar "AP isolation" se existir).
- [ ] IP fixo ou reserva DHCP nas duas máquinas.
- [ ] Mesmo `ROS_DOMAIN_ID` nas duas pontas.
- [ ] `ROS_LOCALHOST_ONLY=0` explícito no Ubuntu.
- [ ] Regra de entrada UDP liberada no Firewall do Windows Defender.
- [ ] Relógios sincronizados (seção *chrony* abaixo).

## Variáveis de ambiente (Ubuntu)

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

## MATLAB (Windows)

```matlab
setenv('ROS_DOMAIN_ID', '0');
```
Em duas máquinas físicas na mesma sub-rede, a descoberta multicast padrão do
FastDDS é suficiente — profiles só se o teste abaixo falhar.

## Teste de sanidade (ANTES de qualquer nó do robô)

1. `ping <ip-da-outra-máquina>` nos dois sentidos.
2. Ubuntu: `ros2 topic pub /teste std_msgs/String "data: 'ola'" --rate 1`
3. MATLAB: `ros2("topic","list")` e assinar `/teste`.
Se a mensagem chegar, a rede está OK — qualquer problema depois é lógica.

## Sincronização de relógio (chrony)

Timestamps coerentes entre as máquinas evitam falsos "pose obsoleta" e
bagunça nas análises de latência do rosbag.

```bash
# Ubuntu
sudo apt install chrony
chronyc tracking     # conferir offset (deve ficar em ms)
```
O Windows sincroniza por padrão (Configurações > Hora). Confira se os dois
apontam para servidores NTP e se o offset entre eles é < 100 ms.

## rosbridge (visualizador web — modos Ao vivo e Robô virtual)

```bash
# Ubuntu (uma vez)
sudo apt install ros-humble-rosbridge-suite
# a cada sessão
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# liberar a porta p/ outros dispositivos da rede
sudo ufw allow 9090
```
No site (`simulador_arena_nero_v4.html`): `ws://IP-DO-UBUNTU:9090` → Conectar.

## IPs (preencher)

| Máquina  | IP        | Observações |
|----------|-----------|-------------|
| Ubuntu (percepção) | `_____` | ROS 2 Humble + rosbridge |
| Windows (controle) | `_____` | MATLAB |
