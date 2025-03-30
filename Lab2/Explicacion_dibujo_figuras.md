# Explicacion Detallada de como use atan y RMS para definir los movimientos del dibujo de figuras

**a) Método __init__**
```python
def __init__(self):
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
    self.current_pose = None
    self.rate = rospy.Rate(10)
    self.turtle_size = 0.5  # Asumiendo un tamaño de la tortuga para cálculos de colisión
```
Inicialización del nodo:
`rospy.init_node('turtle_keyboard_control', anonymous=True)` inicia un nodo en ROS con el nombre turtle_keyboard_control. El parámetro `anonymous=True` asegura que, si se lanzan múltiples instancias, cada una tenga un nombre único.

Publicador de velocidad:
`self.pub` se utiliza para publicar mensajes en el tópico `/turtle1/cmd_vel`. Los mensajes del tipo Twist contienen la velocidad lineal y angular para mover la tortuga.

Subscriptor a la posición:
`self.pose_sub` se suscribe al tópico `/turtle1/pose` para recibir actualizaciones de la posición y orientación de la tortuga. Cada mensaje recibido ejecuta el método update_pose.

Variables adicionales:

  - `self.current_pose`: Almacena la última posición conocida de la tortuga.
  
  - `self.rate`: Establece una tasa de 10 Hz para los bucles de actualización.
  
  - `self.turtle_size`: Define un tamaño “ficticio” para la tortuga, evitando que se salga del área de dibujo.

**b) Método update_pose**
```python
def update_pose(self, data):
    self.current_pose = data
```

Callback del suscriptor:
Cada vez que se recibe un mensaje de tipo Pose en `/turtle1/pose`, este método actualiza la variable `self.current_pose` con la nueva posición y orientación de la tortuga.

**c) Método move_to_point**
```python
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
```
Propósito:
Mover la tortuga a una posición objetivo `(target_x, target_y)` utilizando control proporcional.

*Pasos principales:*

Comprobación inicial:
Se verifica que `self.current_pose` no sea None para asegurar que se ha recibido una posición válida.

Cálculo del ángulo y distancia:

 - `angle_to_target:` Calcula el ángulo para llegar al punto objetivo utilizando atan2.
  
 - `distance:` Calcula la distancia Euclidiana entre la posición actual y el objetivo.

Cálculo del error angular:
Se obtiene la diferencia entre el ángulo deseado y la orientación actual, normalizándola entre -π y π.

Control del movimiento:

Ajuste de orientación:
Si el error angular es mayor a 0.01 radianes, se envía un mensaje Twist con velocidad angular proporcional.
Avance hacia el objetivo:
Si el error angular es pequeño pero la distancia es mayor a 0.05, se envía un mensaje Twist con velocidad lineal y un ajuste angular.
Llegada al objetivo:
Cuando ambos, el error angular y la distancia, son mínimos, se imprime un mensaje indicando la posición alcanzada y se rompe el bucle.
Detención final:
Al salir del bucle, se publica un mensaje vacío Twist() para detener la tortuga y se espera 0.5 segundos por si aun no ha terminado algo.

**d) Método valid_position**
```python
def valid_position(self, x, y):
    if x < self.turtle_size or x > 11 - self.turtle_size or \
       y < self.turtle_size or y > 11 - self.turtle_size:
        return False
    return True
```
Propósito:
Verificar que las coordenadas (x, y) estén dentro del área de dibujo permitida (generalmente una ventana de 11x11 unidades en Turtlesim).

Lógica:
La posición es válida si x e y se encuentran entre self.turtle_size y 11 - self.turtle_size, evitando que la tortuga se dibuje demasiado cerca del borde.

**e) Método request_position**
```python

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
```

Función interactiva:
Solicita al usuario que ingrese dos números separados por espacio, representando las coordenadas iniciales para dibujar la figura.

*Proceso:*
Convierte la entrada a números flotantes.
Valida las coordenadas utilizando valid_position.
En caso de entrada incorrecta o coordenadas fuera del rango permitido, se informa al usuario y se repite la solicitud.

**f) Método draw_figure**
```python
Copy
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
            start_x, start_y = next_x, next_y  # Actualiza la posición de inicio para la siguiente esquina
```

Propósito:
Dibujar una figura geométrica (rombo o pentágono) moviendo la tortuga a través de sus vértices.

*Pasos generales:*

Obtener posición inicial:
Se llama a request_position para que el usuario ingrese el punto de partida y se mueve la tortuga a esa posición usando move_to_point.

Dibujo del Rombo (rhombus):

 - Se define side_length = 2.0.
 - Se utiliza una lista de ángulos ([120°, 60°, 120°, 60°] convertidos a radianes) que determina la geometría del rombo.
 - Para cada ángulo, se calcula la siguiente coordenada y se mueve la tortuga.

**g) Método run**
```python
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
```

Función principal del programa:
Ejecuta un bucle que se mantiene activo mientras el nodo ROS esté en funcionamiento.

Interacción con el usuario:
Muestra un menú en la terminal que solicita:

 - Presionar 'r' para dibujar un rombo.
 - Presionar 'p' para dibujar un pentágono.
 - Presionar 'x' para salir.

Según el comando ingresado, se llama a draw_figure con el tipo de figura correspondiente o se finaliza la ejecución.

## 3. Bloque Principal del Script
```python
if __name__ == '__main__':
    try:
        turtle_control = TurtleControl()
        turtle_control.run()
    except rospy.ROSInterruptException:
        pass
```

Punto de entrada del script:
 - Se crea una instancia de la clase TurtleControl.
 - Se llama al método run para iniciar la interacción con el usuario.
 - Se utiliza un bloque try/except para capturar cualquier excepción de interrupción de ROS (por ejemplo, si se cierra el nodo o se presiona Ctrl+C).

# Conclusión
En resumen, este código implementa un nodo ROS que:
Se inicializa y configura para comunicarse con el simulador Turtlesim.
Publica comandos de velocidad a través del tópico /turtle1/cmd_vel para controlar el movimiento de la tortuga.
Se suscribe a la posición actual de la tortuga mediante /turtle1/pose y actualiza la variable current_pose.
Utiliza control proporcional para mover la tortuga hacia un punto objetivo, primero corrigiendo la orientación y luego avanzando en línea recta.
Permite la interacción con el usuario para definir una posición inicial y seleccionar el tipo de figura a dibujar (rombo o pentágono).
Calcula dinámicamente las posiciones de los vértices de la figura utilizando funciones trigonométricas y mueve la tortuga a cada uno de ellos.
