# Explanation of Python Codes
Done by ANDONI DIAZ CASTELLANOS 169292

### DISCLAIMER

The most recent code related to navigation and this file's formatting (excluding the content) was developed with assistance from ChatGPT.

I encountered numerous challenges while trying to refactor the logic to utilize objects, as the code was originally written using functions only. To meet my supervisor's requirements for this "project," I sought help from ChatGPT. They suggested organizing the functions with shared objectives into classes and using instance variables like self.n.

Additionally, ChatGPT helped rephrase my notes, transforming them from fragmented thoughts into more coherent text.

### **Introduction to Python**

Python is a high-level, interpreted programming language known for its simplicity and readability. It is widely used in various fields, including web development, data science, artificial intelligence, and automation. Python supports multiple programming paradigms, including procedural, object-oriented, and functional programming.

### **Variable Types in Python**
Python provides several built-in data types for storing different kinds of values. Some of the most commonly used types include:
- **int**: Represents integer values 
- **float**: Represents floating-point numbers 
- **str**: Represents text or string values 
- **bool**: Represents boolean values
- **list**: A collection of ordered, mutable items 
- **tuple**: A collection of ordered, immutable items 
- **dict**: A collection of key-value pairs 
- **set**: A collection of unordered, unique elements

### **Control Structures: for and while Loops**
Loops in Python allow for repetitive execution of code blocks.

### **Object-Oriented Programming (OOP) in Python**

## What is OOP?
Object-Oriented Programming (OOP) is a programming paradigm that organizes code into objects that contain both data and behavior. Python supports OOP through classes and objects.

## Key Concepts of OOP

### 1. Classes and Objects
- **Class**: A blueprint for creating objects.
- **Object**: An instance of a class.

### 2. Encapsulation
Encapsulation restricts direct access to object data and only allows modification through methods.

### 3. Inheritance
Inheritance allows one class to inherit attributes and methods from another class.

### 4. Polymorphism
Polymorphism allows different classes to be used interchangeably if they share the same method names.

### 5. Abstraction
Abstraction hides complex implementation details and exposes only the necessary parts.


# Problems:

## 1. Sum of the first `n` natural numbers
Escribir un programa que lea un entero positivo “n” introducido por el usuario y después muestre
en pantalla la suma de todos los enteros desde 1 hasta n . La suma de los primeros enteros

```python
n= int (input("What's the value of n?")) #asks for the limit n
print (n) #prints n
suma = sum(range(1, n + 1)) #sums n values from the range of 1 to n+1
print (suma)
```

### Explanation:
- The user is prompted to enter an integer `n`.
- `n` is printed.
- The sum of all natural numbers from `1` to `n` is calculated using the `sum()` function and the `range()` function.
- Finally, the sum result is printed.

---

## 2. Salary calculation based on worked hours
Escribir un programa que pregunte al usuario por el número de horas trabajadas y el costo por hora.
Después debe mostrar por pantalla la paga que le corresponde.

```python
# Get number of hours worked from user input
horas = int(input("Aight man, tell me how many hours did you punch in today? "))

# Get hourly wage from user input (in dollars)
money = int(input("Bet man, now tell me how much gold ya making out per hour... "))

# Calculate total earnings (hours multiplied by hourly rate)
salario = horas * money

# Display the result showing total earnings
print("Those suits owe ya ", salario, "dollars man")
```

### Explanation:
- The user is asked for two integer values: `horas` (worked hours) and `money` (hourly wage).
- The salary is calculated by multiplying `horas` by `money`.
- The salary is printed with a message.

---

## 3. Salary calculation for employees using a dictionary
Crea una lista de nombre + sueldo por hora + horas trabajadas de al menos seis operadores.
Imprime el nombre y el sueldo a pagar de cada operador.

```python
# Dictionary storing employee data with nested dictionaries for hourly wage and hours
employees = {
    "Sara": {
        "hourly wage": 27,  # Sara's hourly rate in dollars
        "hours": 4          # Number of hours Sara worked
    }
}

# Function to calculate and display salaries for all employees
def salary():
    # Iterate through each employee in the dictionary
    for name, data in employees.items():
        # Calculate salary: hourly wage × hours worked
        sal = data["hourly wage"] * data["hours"]
        # Display formatted result with employee name and calculated salary
        print(f"The salary of {name} is: $ {sal} ")

# Execute the salary calculation function
salary()
```

### Explanation:
- A dictionary `employees` is defined, where each key is an employee's name and its value is another dictionary containing the `hourly wage` and `hours` worked.
- A function `salary()` iterates through the `employees` dictionary, calculating each employee's salary by multiplying `hourly wage` by `hours`.
- The salary of each employee is printed with a message.
- Finally, the function `salary()` is called to execute the calculation.

---

## 4. Calculating the average of even numbers and the product of multiples of 3
• Crea una lista llamada numeros que contenga al menos 10 números.
• Calcula el promedio de los números pares y el producto de los números impares.
• Imprime los resultados
```python
import statistics
import math

numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

# Get even numbers 
even_nums = [num for num in numbers if num % 2 == 0] 
avg = statistics.mean(even_nums)
print(f"Even numbers average: {avg}")  

# Get ODD numbers (not divisible by 2)
odd_nums = [num for num in numbers if num % 2 != 0]  
prod = math.prod(odd_nums)
print(f"Product of odd numbers: {prod}")  
```

### Explanation:
- The `statistics` and `math` modules are imported to perform statistical and mathematical calculations.
- A list `numbers` is defined with values from 1 to 10.
- Even numbers are extracted as another list into the `even_nums` list using list comprehension (`if num % 2 == 0`).
- The average of even numbers is calculated using `statistics.mean()`.
- Numbers that are multiples of 3 are extracted into `odd_nums` (`if num % 3 == 0`).
- The product of these numbers is calculated using `math.prod()`.
- Both results are printed.

---
## 5. Guessing game with a random number
 Crea un programa que solicite al usuario adivinar un número secreto. El programa debe generar
un número aleatorio entre 1 y 10, y el usuario debe intentar adivinarlo. El programa debe
proporcionar pistas si el número ingresado por el usuario es demasiado alto o bajo. El bucle while
debe continuar hasta que el usuario adivine correctamente. Al final se debe imprimir en cuantos
intentos el usuario logró adivinar el número
```python
# Import random module for generating the secret number
import random

def guess_game():
    # Generate secret number between 1-10 and initialize game variables
    secret_number = random.randint(1, 10)  # The holy grail number
    attempts = 0       # Track how many tries the player needs
    got_it = False     # Flag to check if they've guessed right

    # Drop some streetwise instructions
    print("Ight man this one ain't easy....")
    print("Guess the number I'm thinking of, it's between 1 and 10")

    # Main game loop - keeps running until they guess right
    while not got_it:
        try:
            # Get player's guess and convert to integer
            guess = int(input("Say a number, lil' man..."))
            attempts += 1  # Increase try counter

            # Check if guess needs to go up or down
            if guess < secret_number:
                print("Oof! Too low, lil' gangster. Try again!")
            elif guess > secret_number:
                print("Too high, brother! Go lower.")
            else:
                # Correct guess - end the game
                got_it = True
                print(f"Bull's eye, brother! It only took you {attempts} tries... dayum.")
                
        except ValueError:
            # Handle non-number inputs like a streetwise bouncer
            print("No no, don't try to hustle me. That ain't an integer...")

# Start the game by calling the function
guess_game()
```

### Explanation:
- The `random` module is imported to generate a random number.
- The function `guess_game()` generates a secret number between 1 and 10.
- The user is prompted to guess the number, and each guess is counted.
- If the guess is too low or too high, the user gets a hint.
- When the correct number is guessed, the game ends and the number of attempts is displayed.
- The function ensures only integers are accepted as input.
- The function uses a try since its more than likely to fail, an except in needed in case the user uses something different from a number
- The conditions are managed with a simple if, elif and else
- we use got_it as a flag with boolean values to detect when the user gets it right
  
---

## 6. Navigation using DFS methos and POO programming 
## Key Features
- **Map Generation**: Creates an `n x n` grid with ~25% obstacles (`X`).
- **Pathfinding**: Uses DFS to find a valid path (marked with arrows: `→`, `↓`, `←`, `↑`).
- **Visualization**: Prints the initial and solved maps.

## Classes Overview

### 1. `Mapa` Class
Handles map creation and printing.

#### Attributes:
- `n`: Grid size (`n x n`).
- `matriz`: 2D list of lists representing the grid.

#### Methods:
```python
def __init__(self, n):
    # Constructor: Initializes grid and generates obstacles

def generar_mapa(self):
    # Generates the grid with random obstacles ('X')

def imprimir_matriz(self):
    # Prints the grid in two formats (list and joined strings)
```

### 2. `BuscadorCamino` Class
Handles pathfinding logic.

#### Attributes:
- `mapa`: Reference to the `Mapa` object.
- `movimientos`: Possible movements (right, down, left, up).
- `visited`: Tracks visited cells.
- `camino`: Stores the path sequence.

#### Methods:
```python
def __init__(self, mapa):
    # Constructor: Initializes pathfinding components

def dfs(self, x, y):
    # Depth-First Search recursive implementation
    
def encontrar_camino(self):
    # Initiates DFS and marks the path on the grid
```

## Code Flow

### Step 1: Map Creation
```python
mapa = Mapa(10)  # Creates a 10x10 grid
```
- Generates a grid with `o` (free cells) and `X` (obstacles).
- Ensures start `(0,0)` and end `(n-1,n-1)` are always free.

### Step 2: Pathfinding
```python
buscador = BuscadorCamino(mapa)
buscador.encontrar_camino()
```
1. **DFS Algorithm**:
   - Starts at `(0,0)`.
   - Explores neighbors in order: right (`→`), down (`↓`), left (`←`), up (`↑`).
   - Backtracks if a dead end is reached.
   - Marks valid path with directional arrows.

### Step 3: Output
- Prints initial map and solved map (if a path exists).

---
## 7. Inventory managment
Una tienda quiere gestionar su inventario de productos. Para ello, debes implementar un sistema
en Python que permita:
Crear productos, cada uno con un nombre, precio y cantidad en stock.
Actualizar la cantidad en stock cuando se venden productos.
Mostrar la información de un producto con su disponibilidad.
Calcular el valor total del inventario (precio × cantidad de cada producto).

#### Inventory Management System Explanation
For this problem I made a program that implements an object-oriented inventory management system for a store. It allows users to interactively manage products through
a menu-driven interface while maintaining core object-oriented programming (OOP) principles.

#### Core Components
### Product Class:
  Acts as a blueprint for all products
  Stores name, price, and stock attributes
  ##### Contains methods to:
  Process sales (reduce stock)
  Display product information
  Calculate total value of current stock

### Inventory Class
  Manages the collection of products
  #### Implements key operations:
  Adding new products to inventory
  Processing sales by product name
  Displaying product details
  Calculating total inventory value

### Main Program Loop
 #### Provides continuous menu with 5 options:
  Add  products with validated price/stock inputs
  Sell products with stock availability checks
  View product details by name search
  Calculate total inventory monetary value
  Exit the system

  # Final word

  The software is really really simple, however its so extense because of the interactyive menu and the possible clashes within,
  I used OOP and FP making is super compact in everything but the user operations.

