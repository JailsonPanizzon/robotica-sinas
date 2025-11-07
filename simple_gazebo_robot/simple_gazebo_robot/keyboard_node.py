import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Velocidades fixas
        self.linear_speed = 0.5
        self.angular_speed = 1.0

        # Configurar terminal
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info("Teleop iniciado — use as setas para mover, espaço para parar, Q para sair")

        self.current_cmd = Twist()
        self.timer = self.create_timer(0.1, self.update)

    def get_key(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def update(self):
        key = self.get_key()
        cmd = Twist()

        if key is None:
            self.pub.publish(self.current_cmd)
            return

        # Detectar setas (sequência ESC [ A/B/C/D)
        if key == '\x1b':
            seq = sys.stdin.read(2)
            if seq == '[A':  # ↑ frente
                cmd.linear.x = self.linear_speed
            elif seq == '[B':  # ↓ trás
                cmd.linear.x = -self.linear_speed
            elif seq == '[C':  # → direita
                cmd.angular.z = -self.angular_speed
            elif seq == '[D':  # ← esquerda
                cmd.angular.z = self.angular_speed
        elif key == ' ':  # parar
            cmd = Twist()
        elif key.lower() == 'q':  # sair
            self.get_logger().info("Encerrando teleop...")
            self.cleanup()
            rclpy.shutdown()
            return

        self.current_cmd = cmd
        self.pub.publish(self.current_cmd)

    def cleanup(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        stop = Twist()
        self.pub.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
