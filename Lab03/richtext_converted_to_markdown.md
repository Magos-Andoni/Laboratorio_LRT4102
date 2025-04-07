Import Statements:
------------------
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from math import atan2, sqrt, pi
```
*   **rospy**: The ROS Python client library for creating nodes.
    
*   **Twist**: Used to send velocity commands (linear and angular).
    
*   **Pose**: Used to receive position data of the turtle.
    
*   **Spawn and Kill**: Services for creating and removing turtles from the simulation.
    
*   **atan2, sqrt, pi**: Math utilities for calculating distance and angle.
    

Class Definition (TurtleController):
------------------------------------

This is the main class encapsulating the entire functionality.

### Initialization (\_\_init\_\_):
```python
def __init__(self):
    rospy.init_node('turtle_controller')

    self.rate = rospy.Rate(10)  # 10 Hz loop rate

    # Initialize turtle position
    self.current_x = 0.0
    self.current_y = 0.0
    self.current_theta = 0.0

    # Wait until services are available
    rospy.wait_for_service('spawn')
    self.spawn_service = rospy.ServiceProxy('spawn', Spawn)
    rospy.wait_for_service('kill')
    self.kill_service = rospy.ServiceProxy('kill', Kill)

    self.active_turtle = None
    self.velocity_publisher = None
    self.pose_subscriber = None
```

*   Initializes ROS node turtle\_controller.
    
*   Sets loop frequency to 10 Hz.
    
*   Sets initial values for turtle pose.
    
*   Waits for spawn and kill ROS services to be available.
    
*   Prepares variables to manage active turtle states.
    

### Killing Turtles (kill\_turtle):
```python
def kill_turtle(self, turtle_name):
    try:
        self.kill_service(turtle_name)
        rospy.loginfo("Tortuga '%s' eliminada.", turtle_name)
    except rospy.ServiceException as e:
        rospy.logerr("Error al matar la tortuga '%s': %s", turtle_name, e)

```
*   Attempts to remove a turtle by name.
    
*   Logs success or error message.
    

### Spawning Turtles (spawn\_turtle):
```python
def spawn_turtle(self, x, y, theta, name):
    try:
        self.spawn_service(x, y, theta, name)
        rospy.loginfo("Spawn de la tortuga '%s' en: x=%f, y=%f, theta=%f", name, x, y, theta)
    except rospy.ServiceException as e:
        rospy.logerr("Error al hacer spawn de la tortuga '%s': %s", name, e)

```
*   Creates a new turtle at the specified (x, y, theta) coordinates.
    
*   Logs the spawn action or errors.
    

### Getting User Input (get\_desired\_pose\_from\_user):
```python
def get_desired_pose_from_user(self):
    print("Ingrese la posición deseada en el eje x:")
    x = float(input("Coordenada x: "))
    print("Ingrese la posición deseada en el eje y:")
    y = float(input("Coordenada y: "))
    print("Ingrese la orientación final (theta en radianes):")
    theta = float(input("Theta: "))
    return x, y, theta
```
*   Prompts the user to enter desired coordinates and orientation.
    
*   Returns the tuple (x, y, theta).
    

### Updating Current Pose (pose\_callback):
```python
def pose_callback(self, pose):
    self.current_x = pose.x
    self.current_y = pose.y
    self.current_theta = pose.theta
```
*   Callback function triggered every time the turtle publishes its current position.
    
*   Updates internal variables tracking turtle's state.
    

### Controlling Turtle Movement (move\_turtle\_to\_desired\_pose):

```python
def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
    Kp_distance = 0.5  # Proportional constant for linear speed
    Kp_angle = 1.0     # Proportional constant for angular speed
    tolerance = 0.01   # Goal threshold

    while not rospy.is_shutdown():
        dtg = sqrt((desired_x - self.current_x)**2 + (desired_y - self.current_y)**2)
        atg = atan2(desired_y - self.current_y, desired_x - self.current_x)
        error_theta = atg - self.current_theta

        # Normalize angle error between [-pi, pi]
        while error_theta > pi:
            error_theta -= 2 * pi
        while error_theta < -pi:
            error_theta += 2 * pi

        rospy.loginfo("DTG: %f, Error angular: %f", dtg, error_theta)

        if dtg < tolerance:
            rospy.loginfo("Meta alcanzada.")
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.velocity_publisher.publish(twist_msg)
            break

        linear_velocity = Kp_distance * dtg
        angular_velocity = Kp_angle * error_theta

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity

        self.velocity_publisher.publish(twist_msg)
        self.rate.sleep()

```
*   Moves turtle toward a target point using **proportional control** (P controller).
    
*   Calculates:
    
    *   Distance to goal (**DTG**)
        
    *   Angle to goal (**ATG**)
        
    *   Angular error
        
*   Applies proportional gains to set linear and angular velocities.
    
*   Stops when reaching within a tolerance threshold.
    

### Main Execution Loop (run):
```python
def run(self):
    self.kill_turtle("turtle1")  # Removes default turtle at start.

    while not rospy.is_shutdown():
        desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()

        if self.active_turtle:
            self.kill_turtle(self.active_turtle)

        self.active_turtle = "turtle_active"
        self.spawn_turtle(desired_x, desired_y, desired_theta, self.active_turtle)

        self.velocity_publisher = rospy.Publisher('/' + self.active_turtle + '/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/' + self.active_turtle + '/pose', Pose, self.pose_callback)

        rospy.sleep(1)  # Brief pause for subscriber to get initial position.

        self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)
```
*   Repeatedly:
    
    *   Prompts user input.
        
    *   Removes old turtle, spawns new one at user-specified position.
        
    *   Initializes ROS publisher/subscriber for new turtle.
        
    *   Uses proportional control to ensure the turtle precisely reaches the goal.
        

