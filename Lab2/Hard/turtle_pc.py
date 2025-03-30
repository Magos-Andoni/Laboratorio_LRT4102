#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, pi

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_xy_theta')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def pose_callback(self, pose):
        # Actualiza la posición y orientación actual de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
        # Constantes de proporcionalidad del controlador (ajustables)
        Kp_x = 1
        Kp_y = 1
        Kp_theta = 1
        threshold = 0.1  # Tolerancia para considerar que se ha alcanzado la posición u orientación

        # Paso 1: Moverse en el eje X
        rospy.loginfo("Moviendo en X...")
        while not rospy.is_shutdown():
            error_x = desired_x - self.current_x
            if abs(error_x) < threshold:
                rospy.loginfo("Posición en X alcanzada: %f", self.current_x)
                break
            twist_msg = Twist()
            twist_msg.linear.x = Kp_x * error_x
            twist_msg.linear.y = 0
            twist_msg.angular.z = 0
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Movimiento en X: x actual: %f, error: %f", self.current_x, error_x)
            self.rate.sleep()

        # Detenerse antes de continuar
        self.velocity_publisher.publish(Twist())
        rospy.sleep(0.5)

        # Paso 2: Moverse en el eje Y
        rospy.loginfo("Moviendo en Y...")
        while not rospy.is_shutdown():
            error_y = desired_y - self.current_y
            if abs(error_y) < threshold:
                rospy.loginfo("Posición en Y alcanzada: %f", self.current_y)
                break
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.linear.y = Kp_y * error_y
            twist_msg.angular.z = 0
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Movimiento en Y: y actual: %f, error: %f", self.current_y, error_y)
            self.rate.sleep()

        # Detenerse antes de continuar
        self.velocity_publisher.publish(Twist())
        rospy.sleep(0.5)

        # Paso 3: Ajustar la orientación (theta)
        rospy.loginfo("Ajustando orientación...")
        while not rospy.is_shutdown():
            # Calcular el error y normalizarlo al rango [-pi, pi]
            error_theta = desired_theta - self.current_theta
            error_theta = (error_theta + pi) % (2 * pi) - pi

            if abs(error_theta) < threshold:
                rospy.loginfo("Orientación alcanzada: %f", self.current_theta)
                break
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.linear.y = 0
            twist_msg.angular.z = Kp_theta * error_theta
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Rotación: theta actual: %f, error: %f", self.current_theta, error_theta)
            self.rate.sleep()

        # Detener cualquier movimiento final
        self.velocity_publisher.publish(Twist())
        rospy.loginfo("Movimiento completado.")

    def get_desired_pose_from_user(self):
        print("Ingrese la posición deseada en el eje x:")
        desired_x = float(input("Coordenada x: "))
        print("Ingrese la posición deseada en el eje y:")
        desired_y = float(input("Coordenada y: "))
        print("Ingrese la orientación final (theta en radianes):")
        desired_theta = float(input("Theta: "))
        return desired_x, desired_y, desired_theta

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()
            # Mover la tortuga a la posición y orientación deseada en tres pasos
            self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
