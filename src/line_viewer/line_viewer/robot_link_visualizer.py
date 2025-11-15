#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry  # <--- IMPORTANTE: Importa a Odometria
from functools import partial      # <--- Útil para os callbacks

class GazeboLinePublisher(Node):
    def __init__(self):
        super().__init__('gazebo_line_publisher')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.robots_list = ["robot1", "robot2", "robot3", "robot4", "robot5"]
        
        # Dicionário para armazenar a pose mais recente de cada robô
        self.robot_poses = {} 

        # --- Cria um subscriber para cada robô ---
        for robot in self.robots_list:
            # O nome do tópico que definimos na ponte
            topic_name = f'/{robot}/odom' 
            
            # Cria o subscriber, usando 'partial' para passar o nome do robô
            # para a função de callback
            self.create_subscription(
                Odometry,
                topic_name,
                partial(self.odom_callback, robot_name=robot),
                10  # QoS profile
            )
            self.get_logger().info(f'Subscrito ao tópico {topic_name}')

        # Mantém o timer para publicar os marcadores em uma frequência razoável
        self.timer = self.create_timer(1.0, self.publish_lines)
        self.get_logger().info('Gazebo Line Publisher Iniciado')

    def odom_callback(self, msg, robot_name):
        """
        Callback chamado sempre que uma nova odometria é recebida.
        Armazena a posição do robô.
        """
        # Armazena o objeto 'Point' (que contém x, y, z)
        self.robot_poses[robot_name] = msg.pose.pose.position
        
        # self.get_logger().info(f'Pose recebida para {robot_name}') # Descomente para debug

    def create_marker(self, point1, point2, marker_id):
        """
        Cria um marcador LINE_LIST entre dois objetos Point.
        """
        marker = Marker()
        marker.header.frame_id = "world" # Frame de referência global
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_lines"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.05  # Espessura da linha
        marker.color.a = 1.0  # Opacidade
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Adiciona os objetos Point diretamente
        marker.points.append(point1)
        marker.points.append(point2)

        return marker

    def publish_lines(self):
        """
        Publica as linhas entre todos os robôs para os quais temos dados de pose.
        """
        for i, robot1 in enumerate(self.robots_list):
            # Pega a pose do dicionário
            pos1 = self.robot_poses.get(robot1) 
            
            if pos1 is None:
                # Loga um aviso se ainda não tivermos recebido a pose
                self.get_logger().warn(f'Sem dados de pose para {robot1}, pulando.', throttle_duration_sec=5)
                continue

            for j, robot2 in enumerate(self.robots_list):
                if i >= j: # Evita linhas duplicadas (robot1->robot2 vs robot2->robot1)
                    continue

                # Pega a pose do segundo robô
                pos2 = self.robot_poses.get(robot2)
                
                if pos2 is None:
                    self.get_logger().warn(f'Sem dados de pose para {robot2}, pulando.', throttle_duration_sec=5)
                    continue

                # Cria um ID único e consistente para o par de robôs
                marker_id = hash(frozenset([robot1, robot2])) % 1000
                
                # Cria o marcador passando os objetos Point
                marker = self.create_marker(pos1, pos2, marker_id)
                self.marker_pub.publish(marker)
                
                # self.get_logger().info(f'Publicada linha: {robot1} -> {robot2}') # Descomente para debug

    # A função read_pose(self, robot) pode ser DELETADA

def main(args=None):
    rclpy.init(args=args)
    node = GazeboLinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()