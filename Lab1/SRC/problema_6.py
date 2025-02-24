import random

class Mapa:
    def __init__(self, n):
        """
        Initializes the map with a given size `n x n`.
        Generates the map with obstacles and ensures the start and end points are free.
        """
        self.n = n
        self.matriz = self.generar_mapa()

    def generar_mapa(self):
        """
        Generates an `n x n` grid with approximately 25% obstacles ('X').
        Ensures the start (0, 0) and end (n-1, n-1) cells are always free.
        """
        matriz = [['o' for _ in range(self.n)] for _ in range(self.n)]  # Initialize grid with 'o'
        num_obstaculos = (self.n * self.n) // 4  # Approximately 25% of the grid as obstacles

        for _ in range(num_obstaculos):
            x, y = random.randint(0, self.n-1), random.randint(0, self.n-1)
            # Ensure the start and end points are not blocked
            if (x, y) not in [(0, 0), (self.n-1, self.n-1)] and matriz[x][y] == 'o':
                matriz[x][y] = 'X'  # Place obstacle

        return matriz

    def imprimir_matriz(self):
        """
        Prints the grid in two formats:
        1. As a list of lists.
        2. As a formatted grid with spaces between cells.
        """
        print("Mapa inicial (formato de lista):")
        for fila in self.matriz:
            print(fila)

        print("\nMapa inicial (formato de grid):")
        for fila in self.matriz:
            print(" ".join(fila))
        print()

class BuscadorCamino:
    def __init__(self, mapa):
        """
        Initializes the pathfinder with the map and possible movements.
        """
        self.mapa = mapa
        self.n = len(mapa.matriz)
        self.movimientos = [(0, 1, '→'), (1, 0, '↓'), (0, -1, '←'), (-1, 0, '↑')]  # Right, Down, Left, Up
        self.visited = set()  # Tracks visited cells
        self.camino = []  # Stores the path sequence

    def dfs(self, x, y):
        """
        Depth-First Search (DFS) algorithm to find a path from (x, y) to (n-1, n-1).
        Marks the path with directional arrows.
        """
        if (x, y) == (self.n-1, self.n-1):  # Base case: reached the destination
            return True

        self.visited.add((x, y))  # Mark current cell as visited
        for dx, dy, dir in self.movimientos:  # Explore all possible movements
            nx, ny = x + dx, y + dy
            # Check if the new cell is within bounds, free, and not visited
            if 0 <= nx < self.n and 0 <= ny < self.n and self.mapa.matriz[nx][ny] == 'o' and (nx, ny) not in self.visited:
                self.camino.append((nx, ny, dir))  # Add to path
                if self.dfs(nx, ny):  # Recursively explore
                    return True
                self.camino.pop()  # Backtrack if no path is found

        return False

    def encontrar_camino(self):
        """
        Initiates DFS from the start point (0, 0) and marks the path on the grid if found.
        """
        if self.dfs(0, 0):  # Start DFS from (0, 0)
            for x, y, dir in self.camino:  # Mark the path with directional arrows
                self.mapa.matriz[x][y] = dir
            return True
        return False

def main():
    """
    Main function to execute the program.
    """
    n = 10  # Size of the grid
    mapa = Mapa(n)
    print("Mapa inicial:")
    mapa.imprimir_matriz()

    buscador = BuscadorCamino(mapa)
    if buscador.encontrar_camino():
        print("Ruta encontrada:")
        mapa.imprimir_matriz()
    else:
        print("Imposible llegar al destino")

if __name__ == "__main__":
    main()
