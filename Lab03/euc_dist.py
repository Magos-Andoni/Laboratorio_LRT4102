#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from math import atan2, sqrt, pi

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller')

        # Frecuencia de publicación (10 Hz)
        self.rate = rospy.Rate(10)

        # Variables para almacenar la posición actual de la tortuga
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Servicio de spawn y kill (espera a que estén disponibles)
        rospy.wait_for_service('spawn')
        self.spawn_service = rospy.ServiceProxy('spawn', Spawn)
        rospy.wait_for_service('kill')
        self.kill_service = rospy.ServiceProxy('kill', Kill)

        # Inicialmente no hay tortuga activa (nueva)
        self.active_turtle = None

        # Inicialmente no se ha definido publisher ni subscriber
        self.velocity_publisher = None
        self.pose_subscriber = None

    def kill_turtle(self, turtle_name):
        try:
            self.kill_service(turtle_name)
            rospy.loginfo("Tortuga '%s' eliminada.", turtle_name)
        except rospy.ServiceException as e:
            rospy.logerr("Error al matar la tortuga '%s': %s", turtle_name, e)

    def spawn_turtle(self, x, y, theta, name):
        try:
            self.spawn_service(x, y, theta, name)
            rospy.loginfo("Spawn de la tortuga '%s' en: x=%f, y=%f, theta=%f", name, x, y, theta)
        except rospy.ServiceException as e:
            rospy.logerr("Error al hacer spawn de la tortuga '%s': %s", name, e)

    def get_desired_pose_from_user(self):
        print("Ingrese la posición deseada en el eje x:")
        x = float(input("Coordenada x: "))
        print("Ingrese la posición deseada en el eje y:")
        y = float(input("Coordenada y: "))
        print("Ingrese la orientación final (theta en radianes):")
        theta = float(input("Theta: "))
        return x, y, theta

    def pose_callback(self, pose):
        # Actualiza la posición actual de la tortuga activa
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
        # Ganancias del controlador proporcional
        Kp_distance = 0.5   # Para la velocidad lineal
        Kp_angle = 1.0      # Para la velocidad angular
        tolerance = 0.01    # Umbral para considerar que se alcanzó la meta

        while not rospy.is_shutdown():
            # Calcular DTG (distancia a la meta)
            dtg = sqrt((desired_x - self.current_x)**2 + (desired_y - self.current_y)**2)
            # Calcular ATG (ángulo hacia la meta)
            atg = atan2(desired_y - self.current_y, desired_x - self.current_x)
            # Error angular
            error_theta = atg - self.current_theta

            # Normalización del error angular al rango [-pi, pi]
            while error_theta > pi:
                error_theta -= 2 * pi
            while error_theta < -pi:
                error_theta += 2 * pi

            rospy.loginfo("DTG: %f, Error angular: %f", dtg, error_theta)

            # Si la tortuga está en la meta, detenemos el controlador
            if dtg < tolerance:
                rospy.loginfo("Meta alcanzada.")
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.velocity_publisher.publish(twist_msg)
                break

            # Mapeo de velocidades (control proporcional)
            linear_velocity = Kp_distance * dtg
            angular_velocity = Kp_angle * error_theta

            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity

            self.velocity_publisher.publish(twist_msg)
            self.rate.sleep()

    def run(self):
        # Al iniciar, se mata la tortuga original (turtle1) si existe
        self.kill_turtle("turtle1")

        while not rospy.is_shutdown():
            # Solicitar nuevas coordenadas al usuario
            desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()

            # Si ya hay una tortuga activa (spawn anterior), se mata antes de crear la nueva
            if self.active_turtle:
                self.kill_turtle(self.active_turtle)

            # Usaremos siempre el mismo nombre para la nueva tortuga
            self.active_turtle = "turtle_active"
            self.spawn_turtle(desired_x, desired_y, desired_theta, self.active_turtle)

            # Configurar publisher y subscriber para la tortuga activa
            self.velocity_publisher = rospy.Publisher('/' + self.active_turtle + '/cmd_vel', Twist, queue_size=10)
            self.pose_subscriber = rospy.Subscriber('/' + self.active_turtle + '/pose', Pose, self.pose_callback)

            # Espera breve para asegurarse de recibir el primer mensaje de pose
            rospy.sleep(1)

            # Ejecutar el controlador posicional (aunque la tortuga ya se spawnea en la meta,
            # este controlador verifica y publica comandos en caso de pequeñas variaciones)
            self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)

            # Al finalizar la iteración se repite el proceso para ingresar nuevas coordenadas

if __name__ == '__main__':
    try:
        controller = TurtleController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
