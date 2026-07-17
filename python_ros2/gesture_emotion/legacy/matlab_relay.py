import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

# Configurações
MATLAB_IP = '127.0.0.1'  # IP do Host Windows (visto pelo WSL)
MATLAB_PORT = 9999       # Porta para enviar dados para o MATLAB

class GestureRelay(Node):
    def __init__(self):
        super().__init__('gesture_relay_node')
        self.subscription = self.create_subscription(
            String,
            '/Gesture',
            self.listener_callback,
            10)
        # Cria um socket UDP para falar com o MATLAB
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info('Nó de relay iniciado. Aguardando gestos no tópico /Gesture...')
        self.get_logger().info(f'Enviando para o MATLAB em {MATLAB_IP}:{MATLAB_PORT}')

    def listener_callback(self, msg):
        gesture = msg.data
        self.get_logger().info(f'Recebido gesto: "{gesture}". Reenviando para o MATLAB...')
        # Envia a mensagem recebida para o MATLAB via UDP
        self.sock.sendto(gesture.encode('utf-8'), (MATLAB_IP, MATLAB_PORT))

def main(args=None):
    rclpy.init(args=args)
    gesture_relay = GestureRelay()
    rclpy.spin(gesture_relay)
    gesture_relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
