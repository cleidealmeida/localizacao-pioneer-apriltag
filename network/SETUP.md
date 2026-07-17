# Configuração de rede — Ubuntu (percepção) ↔ Windows (controle)

Duas máquinas físicas na mesma rede local, comunicando via ROS 2 (DDS).

## Checklist antes de rodar qualquer nó do robô

- [ ] Rede **cabeada** nas duas máquinas (Wi-Fi é o principal suspeito de perda de
      pacote em multicast DDS; controle em malha fechada não perdoa latência).
- [ ] Multicast liberado no roteador (desligar "AP isolation" se existir).
- [ ] IP fixo ou reserva DHCP nas duas máquinas.
- [ ] Mesmo `ROS_DOMAIN_ID` nas duas pontas.
- [ ] `ROS_LOCALHOST_ONLY=0` explícito no Ubuntu.
- [ ] Regra de entrada UDP liberada no Firewall do Windows Defender.

## Variáveis de ambiente (Ubuntu)

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

## Variáveis de ambiente / profile (Windows, dentro do MATLAB)

```matlab
setenv('ROS_DOMAIN_ID', '0');
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', 'C:\caminho\para\fastdds_profile.xml');
```

> Ajustar o profile FastDDS para a rede física real (não mais o cenário WSL2/NAT
> virtual) — revisar `initialPeersList` do profile para apontar para o IP da
> máquina Ubuntu na rede local.

## Teste de sanidade (fazer isso ANTES de qualquer nó do robô)

1. `ping <ip-da-outra-máquina>` nos dois sentidos.
2. No Ubuntu:
   ```bash
   ros2 topic pub /teste std_msgs/String "data: 'ola'" --rate 1
   ```
3. No Windows/MATLAB:
   ```matlab
   ros2("topic","list")
   sub = ros2subscriber(node, "/teste", "std_msgs/String");
   ```
   Se `/teste` aparecer e a mensagem chegar, a rede está OK — qualquer problema
   depois disso é lógica de aplicação, não DDS.

## IPs (preencher)

| Máquina  | IP        | Observações |
|----------|-----------|-------------|
| Ubuntu (percepção) | `_____` | ROS 2 Humble |
| Windows (controle) | `_____` | MATLAB |
