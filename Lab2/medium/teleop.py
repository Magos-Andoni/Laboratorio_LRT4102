#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

class TurtleControl:
    def __init__(self):
        rospy.init_node('turtle_keyboard_control', anonymous=True)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.current_pose = None
        self.rate = rospy.Rate(10)
        self.turtle_size = 0.5  # Asumiendo un tamaño de la tortuga para cálculos de colisión

    def update_pose(self, data):
        self.current_pose = data

    def move_to_point(self, target_x, target_y, kp_linear=1.0, kp_angular=2.0):
        while not rospy.is_shutdown():
            if self.current_pose is None:
                continue

            # Calculate angle to target and distance
            angle_to_target = math.atan2(target_y - self.current_pose.y, target_x - self.current_pose.x)
            distance = math.sqrt((target_x - self.current_pose.x) ** 2 + (target_y - self.current_pose.y) ** 2)

            # Adjust orientation to face the target
            angle_error = angle_to_target - self.current_pose.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if abs(angle_error) > 0.01:
                twist = Twist()
                twist.angular.z = kp_angular * angle_error
                self.pub.publish(twist)
            elif distance > 0.05:
                twist = Twist()
                twist.linear.x = kp_linear * distance
                twist.angular.z = kp_angular * angle_error
                self.pub.publish(twist)
            else:
                print(f"Reached corner at ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})")
                break

            self.rate.sleep()

        # Stop the turtle after reaching the point
        self.pub.publish(Twist())
        rospy.sleep(0.5)

    def valid_position(self, x, y):
        # Verifica que la posición está dentro del rango permitido (por ejemplo, la ventana de turtlesim)
        if x < self.turtle_size or x > 11 - self.turtle_size or \
           y < self.turtle_size or y > 11 - self.turtle_size:
            return False
        return True

    def request_position(self):
        while True:
            try:
                x, y = map(float, input("Ingrese las coordenadas de inicio para dibujar la figura (x, y): ").split())
                if self.valid_position(x, y):
                    return x, y
                else:
                    print("La posición es inválida, la figura estaría fuera del área de dibujo. Intente nuevamente.")
            except ValueError:
                print("Entrada inválida. Por favor, ingrese coordenadas numéricas.")

    def draw_figure(self, figure_type):
        start_x, start_y = self.request_position()
        self.move_to_point(start_x, start_y)  # Mover a la posición inicial

        if figure_type == 'rhombus':
            side_length = 2.0
            angles = [math.radians(120), math.radians(60), math.radians(120), math.radians(60)]
            current_angle = self.current_pose.theta

            for angle in angles:
                next_x = start_x + side_length * math.cos(current_angle)
                next_y = start_y + side_length * math.sin(current_angle)
                self.move_to_point(next_x, next_y)
                print(f"Esquina en ({next_x:.2f}, {next_y:.2f})")
                current_angle += angle
                start_x, start_y = next_x, next_y  # Update start position to new corner

        elif figure_type == 'pentagon':
            side_length = 2.0
            angle = math.radians(72)
            current_angle = self.current_pose.theta

            for _ in range(5):
                next_x = start_x + side_length * math.cos(current_angle)
                next_y = start_y + side_length * math.sin(current_angle)
                self.move_to_point(next_x, next_y)
                print(f"Esquina en ({next_x:.2f}, {next_y:.2f})")
                current_angle += angle
                start_x, start_y = next_x, next_y  # Update start position to new corner

    def run(self):
        while not rospy.is_shutdown():
            print("Press 'r' to draw rhombus, 'p' to draw pentagon, 'x' to exit")
            command = input().strip().lower()
            if command == 'x':
                break
            elif command == 'r':
                self.draw_figure('rhombus')
            elif command == 'p':
                self.draw_figure('pentagon')

if __name__ == '__main__':
    try:
        turtle_control = TurtleControl()
        turtle_control.run()
    except rospy.ROSInterruptException:
        pass
