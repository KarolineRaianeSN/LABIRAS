#!/usr/bin/env python3
import math
import rclpy
from functools import partial
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_closest_turtle_first", True)

        # Parâmetro para determinar se deve capturar a tartaruga mais próxima primeiro
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value
        self.turtle_to_catch_ = None
        self.pose_ = None
        # Publicador para enviar comandos de velocidade para a tartaruga
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        # Assinatura para receber a posição da tartaruga
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        # Assinatura para receber a lista de tartarugas vivas
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        # Timer para controlar o loop de movimento
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    # Cálculo da distância euclidiana entre a tartaruga controlada e a tartaruga alvo
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
                    # Verifica se a tartaruga atual é a mais próxima
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
            else:
                # Se não for para capturar a tartaruga mais próxima, captura a primeira da lista
                self.turtle_to_catch_ = msg.turtles[0]

    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return

        # Cálculo da distância euclidiana entre a tartaruga controlada e a tartaruga alvo
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        # Fórmula da distância euclidiana: distance = sqrt((dist_x)^2 + (dist_y)^2)
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        # Inicializa a mensagem Twist para enviar comandos de velocidade
        msg = Twist()

        if distance > 0.5:
            # Se a distância for maior que 0.5, move-se em direção à tartaruga alvo

            # Define a velocidade linear proporcional à distância
            # Aqui estamos utilizando uma constante de 2 para multiplicar a distância
            msg.linear.x = 2 * distance

            # Calcula o ângulo de direção usando a função atan2
            # atan2 retorna o ângulo em radianos entre o eixo x e a linha do ponto (x, y)
            goal_theta = math.atan2(dist_y, dist_x)
            # Diferença entre o ângulo desejado e a orientação atual
            diff = goal_theta - self.pose_.theta
            # Ajusta o ângulo para estar no intervalo [-π, π]
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            # Define a velocidade angular proporcional à diferença de ângulo
            # Aqui estamos utilizando uma constante de 6 para multiplicar a diferença angular
            msg.angular.z = 6 * diff
        else:
            # Se a tartaruga estiver a uma distância menor que 0.5, considera que o alvo foi alcançado
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # Chama o serviço para capturar a tartaruga
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        # Publica a mensagem de velocidade
        self.cmd_vel_publisher_.publish(msg)

    def call_catch_turtle_server(self, turtle_name):
        # Cria um cliente para chamar o serviço de captura de tartarugas
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        # Cria e preenche o pedido do serviço
        request = CatchTurtle.Request()
        request.name = turtle_name

        # Chama o serviço de forma assíncrona
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle, turtle_name=turtle_name))

    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " could not be caught")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
