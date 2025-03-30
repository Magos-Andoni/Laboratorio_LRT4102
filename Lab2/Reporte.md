# Reporte de Laboratorio 02 ANDONI DIAZ

## Introducción
Este repositorio contiene los codigod de laboratorio realizadas en ROSPY. Se han implementado diversos ejercicios que van desde la creación de un paquete básico con comunicación entre nodos hasta el control de posición en turtlesim utilizando algoritmos de control (P, PI y PID).

## Requisitos
- **ROS** (Robot Operating System)
- Paquetes y dependencias: `rospy`, `roscpp`, `std_msgs`
- **turtlesim**
- Herramienta de graficación (por ejemplo, [Plot Juggler](https://github.com/facontidavide/PlotJuggler) u otra alternativa)

## Basic
1. **Creación del paquete**: Se creó un paquete llamado `Practicas_lab` con las dependencias `rospy`, `roscpp` y `std_msgs`.
2. **Archivos de nodos**: Se colocaron los archivos `listener.py` y `talker.py` en el directorio `scripts`.
3. **Compilación**: Se compiló el paquete utilizando `catkin_make`.
4. **Ejecución**:
   - Ejecutar el **talker**: `rosrun Practicas_lab talker.py`
   - Ejecutar el **listener**: `rosrun Practicas_lab listener.py`

- ### Listener

Este script implementa un nodo de ROS que se encarga de escuchar mensajes en el tópico "chatter". A continuación, se describe brevemente el funcionamiento del código:

1. **Importaciones**:  
   - `rospy`: Permite interactuar con el sistema de ROS en Python.  
   - `String` de `std_msgs.msg`: Define el tipo de mensaje que se recibirá.

2. **Función `callback(data)`**:  
   Esta función se ejecuta cada vez que se recibe un mensaje en el tópico "chatter". Utiliza `rospy.loginfo` para imprimir en consola el identificador del nodo y el contenido del mensaje recibido.

3. **Función `listener()`**:  
   - Se inicializa el nodo con el nombre "listener". El parámetro `anonymous=True` garantiza que cada instancia del nodo tenga un nombre único para evitar conflictos.  
   - Se suscribe al tópico "chatter", asociando la función `callback` para procesar los mensajes entrantes.  
   - `rospy.spin()` mantiene el nodo en ejecución, permitiendo que siga escuchando mensajes hasta que se detenga manualmente.

4. **Ejecución del script**:  
   La condición `if __name__ == '__main__':` asegura que la función `listener()` se ejecute cuando el script se invoque directamente, iniciando el proceso de escucha.

Esta estructura permite que el nodo "listener" funcione de forma continua, recibiendo y mostrando en consola los mensajes publicados en el tópico "chatter".
- ### Talker

Este script implementa un nodo de ROS encargado de publicar mensajes en el tópico "chatter". A continuación se detalla el funcionamiento del código:

1. **Importaciones**:
   - `rospy`: Permite interactuar con ROS desde Python.
   - `String` de `std_msgs.msg`: Define el tipo de mensaje utilizado para enviar cadenas de texto.

2. **Función `talker()`**:
   - Se crea un publicador (`pub`) que envía mensajes de tipo `String` al tópico "chatter" con un tamaño de cola de 10 mensajes.
   - Se inicializa el nodo con el nombre "talker" utilizando `rospy.init_node`, y el parámetro `anonymous=True` asegura que cada instancia tenga un nombre único.
   - Se establece una tasa de publicación de 10 Hz mediante `rospy.Rate(10)`.
   - Dentro de un bucle `while not rospy.is_shutdown()`, se genera un mensaje que concatena "hello world" con el tiempo actual obtenido de `rospy.get_time()`. Este mensaje se registra en la consola usando `rospy.loginfo` y se publica en el tópico "chatter". El método `rate.sleep()` se utiliza para mantener la frecuencia establecida.

3. **Manejo de Excepciones**:
   - La estructura `try/except` captura `rospy.ROSInterruptException` para manejar de forma limpia la interrupción del nodo, evitando errores abruptos durante la ejecución.

Este nodo, en conjunto con el nodo `listener`, permite verificar la correcta comunicación en ROS mediante el envío y recepción de mensajes en el tópico "chatter".


5. **Conclusión**: Se evaluó el funcionamiento de la comunicación entre nodos, comprobando el envío y recepción de mensajes.

## Medium
1. **Control por teclado para turtlesim**: Se implementó un nodo que permite controlar la tortuga de turtlesim utilizando el teclado.
2. **Dibujos geométricos**: Se desarrollo un scripts para dibujar un rombo y un pentagono en turtlesim utilizando un controlador PC (originalmente no usaba el controlador pero dado que ese fue el challenge del examen decidi cambiarlo porque esta mas padre).

   ### Control por Teclado para Turtlesim: Dibujo de Figuras

Este script implementa un control interactivo de la tortuga en turtlesim, permitiendo al usuario dibujar figuras (rhombus o pentágono) mediante el teclado. A continuación se detalla el funcionamiento del código:

1. **Inicialización y Configuración**:
   - Se inicializa el nodo con `rospy.init_node('turtle_keyboard_control', anonymous=True)`.
   - Se establece un publicador en el tópico `/turtle1/cmd_vel` para enviar comandos de movimiento.
   - Se suscribe al tópico `/turtle1/pose` para obtener la posición y orientación actual de la tortuga, actualizando la variable `current_pose`.
   - Se define una tasa de publicación de 10 Hz y se asume un tamaño de la tortuga (`turtle_size = 0.5`) para cálculos de colisión y validación de posiciones.

2. **Actualización de la Posición**:
   - La función `update_pose` actualiza continuamente `current_pose` con los datos del mensaje `Pose`, permitiendo el cálculo dinámico de los movimientos necesarios.

3. **Movimiento hacia un Punto**:
   - La función `move_to_point` calcula el ángulo hacia el objetivo y la distancia utilizando funciones trigonométricas (`math.atan2` y `math.sqrt`).
   - Se ajusta la orientación de la tortuga para enfrentar el objetivo, corrigiendo el error angular y publicando comandos de giro.
   - Una vez orientada correctamente, si la distancia es mayor a un umbral, se envía un comando de velocidad lineal proporcional a la distancia.
   - El proceso se repite hasta que la tortuga alcanza el punto objetivo, tras lo cual se detiene brevemente.

4. **Validación e Interacción con el Usuario**:
   - La función `valid_position` comprueba que las coordenadas ingresadas se encuentren dentro del área válida de la ventana de turtlesim, evitando que la figura se dibuje fuera del espacio permitido.
   - `request_position` solicita al usuario las coordenadas iniciales para dibujar la figura y verifica su validez antes de proceder.

5. **Dibujo de Figuras**:
   - La función `draw_figure` permite seleccionar entre dos figuras:
     - **Rhombus**: Se define una longitud de lado fija y se utilizan ángulos alternos (120° y 60°) para calcular las posiciones de cada esquina.
     - **Pentagon**: Se utiliza un ángulo constante de 72° para calcular las 5 esquinas del pentágono.
   - En ambos casos, la tortuga se desplaza de una esquina a la siguiente utilizando `move_to_point`, imprimiendo las coordenadas alcanzadas en cada vértice.

6. **Ejecución Interactiva**:
   - El método `run` implementa un bucle que espera comandos del usuario:
     - Presionar `'r'` dibuja un rhombus.
     - Presionar `'p'` dibuja un pentagon.
     - Presionar `'x'` finaliza el programa.
   - Dependiendo del comando ingresado, se llama a la función correspondiente para iniciar el dibujo de la figura seleccionada.

En resumen, este script combina la navegación de la tortuga hacia puntos específicos con una interfaz de usuario interactiva, permitiendo el dibujo de figuras geométricas en el entorno de turtlesim.


## Advanced
1. **Control de posición para turtlesim**:
   - **Controlador P**: Se implementó un controlador proporcional para mover la tortuga a posiciones deseadas.
   - **Controlador PI**: Se agregó un término integral al controlador para mejorar la precisión.
   - **Controlador PID**: Se añadió un término derivativo para optimizar aún más el desempeño.
    ### Control Posicional (Movimiento Eje a Eje)

Este script implementa un nodo ROS en Python3 que utiliza un controlador proporcional para mover la tortuga en turtlesim a una posición y orientación deseadas, de forma secuencial a lo largo de cada eje (X, Y) y luego ajustando la orientación (theta). A continuación se detalla su funcionamiento:

1. **Inicialización y Configuración**:
   - Se inicializa el nodo con `rospy.init_node('control_tortuga_xy_theta')`.
   - Se crea una suscripción al tópico `/turtle1/pose` para recibir la posición y orientación actuales de la tortuga. La función `pose_callback` actualiza las variables `current_x`, `current_y` y `current_theta`.
   - Se configura un publicador en el tópico `/turtle1/cmd_vel` para enviar comandos de movimiento a la tortuga.
   - Se establece una tasa de publicación de 10 Hz para asegurar actualizaciones regulares.

2. **Control Proporcional**:
   - Se definen constantes de proporcionalidad (`Kp_x`, `Kp_y`, `Kp_theta`) que determinan la respuesta del controlador en cada eje y para la orientación.
   - Se utiliza un umbral (`threshold`) para decidir cuándo la tortuga ha alcanzado la posición u orientación deseada, evitando oscilaciones.

3. **Movimiento en Secuencia**:
   - **Movimiento en el eje X**:  
     El código calcula el error entre la posición deseada y la actual en el eje X, y publica un comando de velocidad lineal proporcional a ese error. Una vez que el error es menor al umbral, se detiene el movimiento en X.
   - **Movimiento en el eje Y**:  
     Tras alcanzar la posición en X, se repite el proceso para el eje Y, ajustando la velocidad lineal en función del error en la coordenada Y.
   - **Ajuste de Orientación (Theta)**:  
     Finalmente, se ajusta la orientación de la tortuga. Se calcula el error angular entre la orientación deseada y la actual, normalizándolo al rango [-π, π]. Se publica un comando angular proporcional a este error hasta que la orientación esté dentro del umbral permitido.

4. **Interacción con el Usuario**:
   - La función `get_desired_pose_from_user()` solicita al usuario que ingrese las coordenadas deseadas (x, y) y el ángulo (theta) en radianes.
   - Con `move_turtle_interactively()`, se permite al usuario ingresar múltiples objetivos de manera interactiva, moviendo la tortuga a cada posición y orientación solicitada.

5. **Manejo de Excepciones**:
   - El bloque `try/except` en el bloque principal captura `rospy.ROSInterruptException`, lo que permite finalizar el nodo de forma controlada en caso de interrupciones.

En resumen, este script permite un control posicional interactivo para turtlesim, moviendo la tortuga primero en el eje X, luego en el eje Y y, finalmente, ajustando su orientación, utilizando un simple controlador proporcional para cada etapa.

### Control Posicional (Movimiento Eje a Eje con Controlador PD)

Este script implementa un nodo ROS en Python3 que utiliza un controlador PD (Proporcional-Derivativo) para mover la tortuga en turtlesim hacia una posición y orientación deseadas de forma secuencial en cada eje (X, Y) y posteriormente ajustando la orientación (theta). A continuación se describe el funcionamiento del código:

1. **Inicialización y Configuración**:
   - Se inicializa el nodo con `rospy.init_node('control_tortuga_xy_theta')`.
   - Se suscribe al tópico `/turtle1/pose` para recibir la posición y orientación actuales de la tortuga. La función `pose_callback` actualiza las variables `current_x`, `current_y` y `current_theta`.
   - Se crea un publicador en el tópico `/turtle1/cmd_vel` para enviar comandos de movimiento.
   - Se establece una tasa de publicación de 10 Hz con `rospy.Rate(10)`.

2. **Variables para Control PD**:
   - Se inicializan variables `last_error_x`, `last_error_y` y `last_error_theta` para almacenar el error previo en cada eje. Estas se utilizan para calcular la parte derivativa del controlador.
   - La función `normalize_angle` se encarga de normalizar el error angular al rango [-π, π].

3. **Control PD para el Movimiento**:
   - **Eje X**:
     - Se calcula el error en X como la diferencia entre la posición deseada y la actual.
     - La velocidad en X se calcula como:  
       `vel_x = Kp_x * error_x + Kd_x * (error_x - last_error_x)`
     - Se actualiza `last_error_x` y se publica el comando de velocidad. El proceso se repite hasta que el error en X sea menor a un umbral definido.
   - **Eje Y**:
     - Se procede de forma similar al eje X, calculando el error en Y y la velocidad correspondiente mediante:  
       `vel_y = Kp_y * error_y + Kd_y * (error_y - last_error_y)`
     - Se actualiza `last_error_y` y se publica el comando hasta alcanzar la posición deseada en Y.
   - **Orientación (Theta)**:
     - Se calcula el error angular, normalizado al rango [-π, π].
     - La velocidad angular se determina con:  
       `vel_theta = Kp_theta * error_theta + Kd_theta * (error_theta - last_error_theta)`
     - Se actualiza `last_error_theta` y se envía el comando angular hasta que la diferencia angular sea menor que el umbral.

4. **Interacción con el Usuario**:
   - La función `get_desired_pose_from_user()` solicita al usuario ingresar la posición deseada en X, Y y la orientación (theta en radianes).
   - La función `move_turtle_interactively()` permite al usuario proporcionar múltiples objetivos, ejecutando el movimiento en tres fases (X, Y y theta) para cada entrada.

5. **Manejo de Pausas y Finalización**:
   - Entre cada fase del movimiento (X, Y y theta), se publica un mensaje vacío (`Twist()`) y se realiza una breve pausa (`rospy.sleep(0.5)`) para asegurar que la tortuga se detenga antes de iniciar el siguiente movimiento.
   - Se utiliza un bloque `try/except` para capturar la excepción `rospy.ROSInterruptException` y finalizar el nodo de forma controlada.

En resumen, este script permite un control posicional interactivo para turtlesim utilizando un controlador PD, donde la acción derivativa ayuda a suavizar la respuesta del sistema, reduciendo la oscilación en cada fase del movimiento.

### Control Posicional (Movimiento Eje a Eje con Controlador PID)

Este script implementa un nodo ROS en Python3 que utiliza un controlador PID (Proporcional-Integral-Derivativo) para mover la tortuga en turtlesim hacia una posición y orientación deseadas. El movimiento se realiza en tres fases secuenciales: eje X, eje Y y ajuste de la orientación (theta). A continuación se detalla su funcionamiento:

1. **Inicialización y Configuración**:
   - Se inicializa el nodo con `rospy.init_node('control_tortuga_xy_theta')`.
   - Se suscribe al tópico `/turtle1/pose` para recibir actualizaciones sobre la posición y orientación de la tortuga, utilizando la función `pose_callback` que actualiza las variables `current_x`, `current_y` y `current_theta`.
   - Se crea un publicador en el tópico `/turtle1/cmd_vel` para enviar comandos de movimiento a la tortuga.
   - Se establece una tasa de publicación de 10 Hz mediante `rospy.Rate(10)`.

2. **Variables para el Control PID**:
   - Se definen variables para almacenar el error previo en cada eje (`last_error_x`, `last_error_y`, `last_error_theta`) y para acumular el error (término integral: `error_accumulation_x`, `error_accumulation_y`, `error_accumulation_theta`).
   - La función `normalize_angle` normaliza un ángulo al rango [-π, π], lo que es esencial para manejar correctamente el error angular.

3. **Control PID en Tres Fases**:
   - **Fase 1: Movimiento en el Eje X**:
     - Se calcula el error en X como la diferencia entre la posición deseada y la actual.
     - Se acumula el error para el término integral.
     - Se calcula la velocidad en X con la fórmula:  
       `vel_x = Kp_x * error_x + Ki_x * error_accumulation_x + Kd_x * (error_x - last_error_x)`
     - Se actualiza el error previo y se publica el comando de velocidad en X hasta que el error sea menor que un umbral definido.
   - **Fase 2: Movimiento en el Eje Y**:
     - Se procede de forma similar al eje X, calculando y acumulando el error en Y, y aplicando la fórmula PID:  
       `vel_y = Kp_y * error_y + Ki_y * error_accumulation_y + Kd_y * (error_y - last_error_y)`
     - Se actualiza el error previo y se publica el comando de velocidad en Y hasta alcanzar la posición deseada.
   - **Fase 3: Ajuste de la Orientación (Theta)**:
     - Se calcula el error angular entre la orientación deseada y la actual, normalizándolo con la función `normalize_angle`.
     - Se acumula el error angular y se utiliza la fórmula PID para determinar la velocidad angular:  
       `vel_theta = Kp_theta * error_theta + Ki_theta * error_accumulation_theta + Kd_theta * (error_theta - last_error_theta)`
     - Se actualiza el error previo y se publica el comando angular hasta que el error angular esté por debajo del umbral definido.

4. **Interacción y Ejecución**:
   - La función `get_desired_pose_from_user()` solicita al usuario ingresar las coordenadas deseadas en los ejes X y Y, además de la orientación (theta en radianes).
   - La función `move_turtle_interactively()` permite ejecutar el proceso de movimiento de forma interactiva para múltiples objetivos, ejecutando en cada iteración las tres fases de movimiento.
   - Se incluyen pausas (`rospy.sleep(0.5)`) entre las fases para asegurar que la tortuga se detenga antes de iniciar el siguiente movimiento.

5. **Manejo de Excepciones**:
   - Se utiliza un bloque `try/except` en el bloque principal para capturar `rospy.ROSInterruptException`, lo que permite finalizar el nodo de manera controlada en caso de interrupciones.

En resumen, este script utiliza un controlador PID para ajustar de forma precisa tanto la posición como la orientación de la tortuga en turtlesim, combinando los términos proporcional, integral y derivativo para minimizar el error y mejorar la estabilidad durante el movimiento.

### Conclusiones y Pensamientos Finales

- **Fallas Lógicas en los Controladores:**  
  Se observó que los tres códigos implementados para el control de la tortuga (utilizando controladores posicionales, posicionales diferenciales y posicionales integrales diferenciales) presentan una falla lógica importante. La primera vez que se mueve la tortuga, no se generan errores (siempre y cuando se seleccionen puntos dentro del mundo permitido). Sin embargo, una vez que se asigna una orientación (theta) diferente de cero y la tortuga se posiciona y orienta correctamente, al ingresar una nueva coordenada el movimiento no se cumple adecuadamente. Esto ocurre porque las nuevas coordenadas se interpretan en función del eje de la tortuga (su orientación) y no del eje del mundo. Por ejemplo, si la tortuga se desplaza a (0, 0) con una orientación de 45°, las siguientes coordenadas tendrán un desfase de 45°.

- **Modificaciones en la Sección de Dificultad Media:**  
  El código modificado para la parte intermedia difiere del esperado en el laboratorio, incorporando tres características importantes:
  - **Detección de colisiones:** Se valida que la posición de inicio para dibujar la figura no provoque que la tortuga choque contra los límites del entorno.
  - **Selección de posición inicial:** El usuario puede escoger dónde comenzar a dibujar la figura.
  - **Control posicional para el movimiento:** Aunque se utiliza un controlador posicional (que depende únicamente de la distancia), se evidencia que la velocidad de la tortuga es mayor cuando está lejos del objetivo y disminuye a medida que se acerca a la meta, lo cual es apreciable en la ejecución de los movimientos.

- **Aspectos Técnicos y Metodológicos:**  
  En el control posicional se observó que la velocidad de la tortuga es proporcional a la distancia al objetivo. Esto implica que la tortuga arranca a una alta velocidad y, al acercarse a la posición deseada, reduce su velocidad significativamente hasta detenerse. Esta característica fue útil para el dibujo de figuras, aunque no es perfecta, ya que la precisión depende del controlador posicional.

### Archivos Launch y Participación de ChatGPT en los Códigos

- **Archivos Launch:**  
  Se realizaron varios archivos de tipo launch para poder inicializar de una manera más dinámica y cómoda cada uno de los códigos. Estos archivos permiten lanzar rápidamente el teleoperador, el dibujo de figuras y los tres tipos de códigos que utilizan controladores. En total, se crearon cuatro archivos launch.

- **Sugerencias de Estructura del Controlador Posicional:**  
  La participación de ChatGPT en los códigos se limitó a sugerir la estructura del controlador posicional. Se recomendó utilizar un bucle `while` por cada parámetro (x, y y theta) para publicar la velocidad correspondiente, manteniendo en cero los otros valores y utilizando pausas (`sleep`) para enviar un mensaje vacío de `Twist` que detenga la tortuga antes de iniciar el siguiente bucle.

- **Modelo Cinemático y Dibujo de Figuras:**  
  ChatGPT también aportó ideas para estructurar el movimiento de la tortuga tomando en cuenta el modelo cinemático de Turtlesim. A diferencia de los códigos de posición, donde el movimiento se realiza eje a eje, en el caso del dibujo de figuras se calcula un ángulo entre la posición actual de la tortuga y la posición deseada. Este enfoque se basa en la tangente de las coordenadas a las que se desea llegar, lo que simplifica el código.  
  - El usuario solo ingresa las coordenadas iniciales de la figura, y todas las demás coordenadas (targets) se calculan automáticamente utilizando las dimensiones preestablecidas de la figura.

- **Mejoras en Control y Robustez del Código:**  
  Se prevé que en futuros reportes se abordará el cálculo dinámico del ángulo (teta) basado en las coordenadas objetivo, así como métodos para controlar la velocidad mediante control proporcional. Esto permitirá mejorar la calidad, robustez y dinamismo general del código, evitando un proceso tan manual en el dibujo de figuras.  
  - Por ejemplo, para dibujar un rombo se definen cuatro esquinas con dos ángulos internos de 120° y dos de 60°. Utilizando funciones trigonométricas, se calculan los componentes en X (con el coseno) y en Y (con el seno) para cada vértice.  
  - Esta metodología simplifica el proceso ya que, al utilizar una sola longitud de lado (ya que todos son iguales), se actualiza de forma automática la posición actual a medida que se alcanzan los vértices.

En resumen, la estructura sugerida por ChatGPT facilitó la implementación del controlador posicional y el modelo cinemático en los códigos, haciendo que el dibujo de las figuras se realice de forma más dinámica y automatizada, y permitiendo concentrarse en otros aspectos importantes del laboratorio.

- **Herramientas y Asistencia en la Elaboración del Reporte:**  
  - La redacción y estructura de este reporte se realizaron con la asistencia de ChatGPT, lo cual fue de gran ayuda dado que aún no estoy muy familiarizado con Markdown.
  - Para convertir mi explicación verbal en texto y trabajar de manera más rápida, utilicé la herramienta SpeechTexter.

En resumen, a pesar de la falla lógica identificada en el manejo de las coordenadas tras un primer movimiento exitoso, este laboratorio permitió aprender sobre las diferencias y limitaciones de los controladores implementados, y se realizaron mejoras significativas en el código intermedio para lograr un control más robusto en la ejecución de las figuras.
