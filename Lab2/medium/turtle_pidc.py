#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, pi

class MoveTurtlePIDControl:
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
        
        # Variables para la parte derivativa e integral del controlador PID
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_theta = 0
        
        self.error_accumulation_x = 0
        self.error_accumulation_y = 0
        self.error_accumulation_theta = 0

    def pose_callback(self, pose):
        # Actualiza la posición y orientación actual de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def normalize_angle(self, angle):
        # Normaliza el ángulo al rango [-pi, pi]
        return (angle + pi) % (2 * pi) - pi

    def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
        # Constantes PID (ajustables)
        Kp_x = 1.0
        Ki_x = 0.01
        Kd_x = 0.1
        
        Kp_y = 1.0
        Ki_y = 0.01
        Kd_y = 0.1
        
        Kp_theta = 1.0
        Ki_theta = 0.01
        Kd_theta = 0.1

        threshold = 0.1  # Tolerancia para alcanzar la meta
        
        # --------------------
        # PASO 1: Movimiento en el eje X
        rospy.loginfo("Moviendo en X...")
        self.last_error_x = desired_x - self.current_x
        self.error_accumulation_x = 0.0
        while not rospy.is_shutdown():
            error_x = desired_x - self.current_x
            self.error_accumulation_x += error_x
            vel_x = Kp_x * error_x + Ki_x * self.error_accumulation_x + Kd_x * (error_x - self.last_error_x)
            self.last_error_x = error_x
            
            if abs(error_x) < threshold:
                rospy.loginfo("Posición en X alcanzada: %f", self.current_x)
                break
            
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = 0
            twist_msg.angular.z = 0
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Movimiento en X: x actual: %f, error: %f, velocidad: %f", self.current_x, error_x, vel_x)
            self.rate.sleep()
        
        # Detener la tortuga y pausar brevemente
        self.velocity_publisher.publish(Twist())
        rospy.sleep(0.5)
        
        # --------------------
        # PASO 2: Movimiento en el eje Y
        rospy.loginfo("Moviendo en Y...")
        self.last_error_y = desired_y - self.current_y
        self.error_accumulation_y = 0.0
        while not rospy.is_shutdown():
            error_y = desired_y - self.current_y
            self.error_accumulation_y += error_y
            vel_y = Kp_y * error_y + Ki_y * self.error_accumulation_y + Kd_y * (error_y - self.last_error_y)
            self.last_error_y = error_y
            
            if abs(error_y) < threshold:
                rospy.loginfo("Posición en Y alcanzada: %f", self.current_y)
                break
            
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.linear.y = vel_y
            twist_msg.angular.z = 0
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Movimiento en Y: y actual: %f, error: %f, velocidad: %f", self.current_y, error_y, vel_y)
            self.rate.sleep()
        
        self.velocity_publisher.publish(Twist())
        rospy.sleep(0.5)
        
        # --------------------
        # PASO 3: Ajuste de la orientación (theta)
        rospy.loginfo("Ajustando orientación...")
        error_theta = self.normalize_angle(desired_theta - self.current_theta)
        self.last_error_theta = error_theta
        self.error_accumulation_theta = 0.0
        while not rospy.is_shutdown():
            error_theta = self.normalize_angle(desired_theta - self.current_theta)
            self.error_accumulation_theta += error_theta
            vel_theta = Kp_theta * error_theta + Ki_theta * self.error_accumulation_theta + Kd_theta * (error_theta - self.last_error_theta)
            self.last_error_theta = error_theta
            
            if abs(error_theta) < threshold:
                rospy.loginfo("Orientación alcanzada: %f", self.current_theta)
                break
            
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.linear.y = 0
            twist_msg.angular.z = vel_theta
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Rotación: theta actual: %f, error: %f, velocidad angular: %f", self.current_theta, error_theta, vel_theta)
            self.rate.sleep()
        
        self.velocity_publisher.publish(Twist())
        rospy.loginfo("Movimiento completado.")

    def get_desired_pose_from_user(self):
        print("Ingrese la posición deseada en el eje x:")
        desired_x = float(input("Coordenada x: "))
        print("Ingrese la posición deseada en el eje y:")
        desired_y = float(input("Coordenada y: "))
        print("Ingrese la orientación deseada (theta en radianes):")
        desired_theta = float(input("Theta: "))
        return desired_x, desired_y, desired_theta

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()
            self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
