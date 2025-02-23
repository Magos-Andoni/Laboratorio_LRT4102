# Explanation of Python Codes
Done by ANDONI DIAZ CASTELLANOS 169292

DISCLAIMER

The most recent code related to navigation and this file's formatting (excluding the content) was developed with assistance from ChatGPT.

I encountered numerous challenges while trying to refactor the logic to utilize objects, as the code was originally written using functions only. To meet my supervisor's requirements for this "project," I sought help from ChatGPT. They suggested organizing the functions with shared objectives into classes and using instance variables like self.n.

Additionally, ChatGPT helped rephrase my notes, transforming them from fragmented thoughts into more coherent text.

## 1. Sum of the first `n` natural numbers

```python
n= int (input("What's the value of n?"))
print (n)
suma = sum(range(1, n + 1))
print (suma)
```

### Explanation:
- The user is prompted to enter an integer `n`.
- `n` is printed.
- The sum of all natural numbers from `1` to `n` is calculated using the `sum()` function and the `range()` function.
- Finally, the sum result is printed.

---

## 2. Salary calculation based on worked hours

```python
horas= int (input("Aight man, tell me how many hours did you punch in today? "))
money= int (input ("Bet man, now tell me how much gold ya making out per hour... "))
salario = horas * money

print ("Those suits owe ya ",salario ,"dollars man")
```

### Explanation:
- The user is asked for two integer values: `horas` (worked hours) and `money` (hourly wage).
- The salary is calculated by multiplying `horas` by `money`.
- The salary is printed with a message.

---

## 3. Salary calculation for employees using a dictionary

```python
employees = {
  "Sara": {
  "hourly wage": 27,
  "hours": 4
  }
}

def salary():
  for name, data in employees.items():
    sal = data["hourly wage"] * data["hours"]
    print (f"The salary of {name} is: $ {sal} ")

salary()
```

### Explanation:
- A dictionary `employees` is defined, where each key is an employee's name and its value is another dictionary containing the `hourly wage` and `hours` worked.
- A function `salary()` iterates through the `employees` dictionary, calculating each employee's salary by multiplying `hourly wage` by `hours`.
- The salary of each employee is printed with a message.
- Finally, the function `salary()` is called to execute the calculation.

---

## 4. Calculating the average of even numbers and the product of multiples of 3

```python
import statistics
import math

numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
even_nums = [num for num in numbers if num % 2 == 0]
avg = statistics.mean(even_nums)
print (avg)
odd_nums = [num for num in numbers if num % 3 == 0]
prod = math.prod(odd_nums)
print (prod)
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

```python
import random

def guess_game():
  secret_number = random.randint(1, 10)
  attempts = 0
  got_it = False

  print("Ight man this one ain't easy....")
  print("Guess the number I'm thinking of, it's between 1 and 10")

  while not got_it:
    try:
      guess = int(input("Say a number, lil' man..."))
      attempts += 1

      if guess < secret_number:
        print("Oof! Too low, lil' gangster. Try again!")
      elif guess > secret_number:
        print("Too high, brother! Go lower.")
      else:
        got_it = True
        print(f"Bull's eye, brother! It only took you {attempts} tries... dayum.")
    except ValueError:
      print("No no, don't try to hustle me. That ain't an integer...")

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

# 1. `Mapa` Class
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

# 2. `BuscadorCamino` Class
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

# Step 1: Map Creation
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

