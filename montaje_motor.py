import numpy as np
import matplotlib.pyplot as plt

# Simulación del entorno
class Environment:
    def __init__(self):
        self.block_position = np.array([0, 0])  # Posición inicial del block
        self.target_position = np.array([5, 5])  # Posición objetivo para el montaje

    def move_block(self, delta):
        self.block_position += delta

    def get_block_position(self):
        return self.block_position

# Simulación del brazo robotizado
class RobotArm:
    def __init__(self):
        self.position = np.array([0, 0])  # Posición inicial del brazo

    def move_to_position(self, target):
        self.position = target

    def get_position(self):
        return self.position

# Función de distancia heurística (distancia euclidiana)
def heuristic_distance(current, target):
    return np.linalg.norm(current - target)

# Algoritmo A* simplificado
def a_star_search(env, arm):
    current_position = arm.get_position()
    target_position = env.get_block_position()

    # Implementación simplificada del algoritmo A*
    while not np.array_equal(current_position, target_position):
        # Movimiento hacia la posición objetivo
        delta = target_position - current_position
        arm.move_to_position(current_position + delta)

        current_position = arm.get_position()

    print("Montaje completado en la posición:", current_position)

# Función para visualizar la simulación
def visualize(env, arm):
    block_position = env.get_block_position()
    arm_position = arm.get_position()

    plt.figure()
    plt.plot(block_position[0], block_position[1], 'rs', label="Block")
    plt.plot(arm_position[0], arm_position[1], 'bo', label="Brazo Robotizado")
    plt.plot([block_position[0], arm_position[0]], [block_position[1], arm_position[1]], 'k--', label="Ruta del Brazo")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Simulación del Montaje del Motor")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Simulación
def main():
    env = Environment()
    arm = RobotArm()

    # Simulamos el movimiento del block
    env.move_block(np.array([3, 4]))

    # Ejecutamos el algoritmo A*
    a_star_search(env, arm)

    # Visualizamos la simulación
    visualize(env, arm)

if __name__ == "__main__":
    main()
