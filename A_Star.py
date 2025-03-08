import heapq
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec
import time

# Funcion Heuristica
def Heuristica(x1,x2,y1,y2):
    h=np.sqrt((x1-x2)**2 + (y1-y2)**2)
    return h

# Creación de clase nodo
class Nodo:
    def __init__(self, posicion, padre=None, g=float('inf')):
        self.posicion = posicion
        self.padre = padre
        self.g = g  # Costo acumulado
        self.f = float('inf')  # Costo total (g + h)

    def __lt__(self, otro):
        return self.f < otro.f  # Para ordenar la cola de prioridad

# Función para obtener el camino desde el nodo objetivo hasta el inicio
def obtener_camino(nodo):
    camino = []
    actual = nodo
    while actual:
        camino.append(actual.posicion)
        actual = actual.padre
    return camino[::-1]  # Invertir para obtener el orden correcto

# Función de verificación de colisión
def es_colision(mapa, x, y, tamano_robot):
    ancho, alto = tamano_robot
    
    if x <= 0 or x + ancho > mapa.shape[0] or y <= 0 or y + alto > mapa.shape[1]:
        return True
    for i in range(int(ancho/2)):
        for j in range(int(ancho/2)):
            if mapa[x + i, y + j] != 0 or mapa[x - i, y - j] != 0:
                return True
    return False

# Algoritmo A* optimizado
def A_star(mapa, inicio, objetivo, tamano_robot=(200, 200), paso=200):
    nodos_abiertos = []
    nodos_cerrados = set()
    nodos_explorados = []
    
    nodo_inicio = Nodo(inicio, None, g=0)
    nodo_inicio.f = Heuristica(*inicio, *objetivo)
    
    heapq.heappush(nodos_abiertos, nodo_inicio)
    costos = {inicio: 0}
    nodo_inicio.padres = {inicio: None}

    nodo_inicio.start = time.time()

    while nodos_abiertos:
        nodo_actual = heapq.heappop(nodos_abiertos)
        if nodo_actual.posicion in nodos_cerrados:
            continue

        nodos_cerrados.add(nodo_actual.posicion)

        if nodo_actual.posicion == objetivo:
            return obtener_camino(nodo_actual), nodos_explorados

        (x, y) = nodo_actual.posicion
        vecinos = [(x - paso, y), (x + paso, y), (x, y - paso), (x, y + paso)]

        for siguiente in vecinos:
            (nx, ny) = siguiente

            if not es_colision(mapa, nx, ny, tamano_robot) and siguiente not in nodos_cerrados:
                g_nuevo = nodo_actual.g + 1
                h = Heuristica(nx, objetivo[0], ny, objetivo[1])
                f_nuevo = g_nuevo + h

                if siguiente not in costos or g_nuevo < costos[siguiente]:
                    costos[siguiente] = g_nuevo
                    nodo_siguiente = Nodo(siguiente, nodo_actual, g=g_nuevo)
                    nodo_siguiente.f = f_nuevo
                    heapq.heappush(nodos_abiertos, nodo_siguiente)
                    nodos_explorados.append(siguiente)
    
    return None, nodos_explorados



mapa = np.zeros((2800, 2800))
'''
# Paredes horizontales
mapa[1199:1201, 0:800] = 1
mapa[399:401, 400:1200] = 1
mapa[400*3:400*3+3, 400*3:400*4] = 1
mapa[400*3:400*3+3, 400*6:400*7] = 1
mapa[400*2:400*2+3, 400*3:400*6] = 1
mapa[400*1:400*1+3, 400*4:400*6] = 1

mapa[400*4:400*4+3, 400*1:400*5] = 1
mapa[400*5:400*5+3, 400*2:400*7] = 1
mapa[400*6:400*6+3, 400*1:400*3] = 1
mapa[400*6:400*6+3, 400*4:400*6] = 1

# Paredes verticales
mapa[400*1:400*2+1, 400*1:400*1+3] = 1
mapa[400*2:400*3, 400*2:400*2+3] = 1
mapa[400*0:400*3, 400*3:400*3+3] = 1
mapa[400*0:400*1, 400*4:400*4+3] = 1
mapa[400*2:400*4, 400*5:400*5+3] = 1
mapa[400*3:400*4, 400*6:400*6+3] = 1
mapa[400*4:400*6, 400*1:400*1+3] = 1
mapa[400*5:400*6, 400*4:400*4+3] = 1

# inicio = (2600, 400*3+200)
inicio = (200 + 400*0, 400*2+200)
objetivo = (200 + 400*0, 200 + 400*4)
'''



mapa[400*6, 400:400]=1
mapa[400*6, 400*2:400*3]=1
mapa[400*5, 400:400*2]=1
mapa[400*5, 400*3:400*4]=1
mapa[400*4, 400*2:400*3]=1
mapa[400*4, 400*4:400*5]=1
mapa[400*3, 400*3:400*4]=1
mapa[400*3, 400*5:400*6]=1
mapa[400*2, 400*4:400*5]=1
mapa[400*1, 400*5:400*6]=1
mapa[400*2, 0:400]=1
mapa[400*1, 0:400]=1
mapa[400*1, 400*3:400*4]=1
mapa[400*2, 400*2:400*3]=1
mapa[400*6, 400*4:400*6]=1
mapa[400*5, 400*5:400*6]=1

# Paredes verticales
mapa[400:400*2, 400*5]=1
mapa[400:400*2, 400*5]=1
mapa[400*3:400*4, 400*5]=1
mapa[400*2:400*3, 400*6]=1
mapa[400*2:400*3, 400*4]=1
mapa[400*4:400*6, 400*4]=1
mapa[400*3:400*4, 400*3]=1
mapa[400*5:400*6, 400*3]=1
mapa[400*4:400*5, 400*2]=1
mapa[400*6:400*7, 400*2]=1
mapa[400*4:400*6, 400*1]=1
mapa[400*2:400*4, 400*1]=1
mapa[400*1:400*3, 400*2]=1
mapa[400*1:400*2, 400*3]=1
mapa[400*4:400*6, 400*6]=1

# inicio = (2600, 400*3+200)
inicio = (200+400*3, 200+400*2)
objetivo = (200+400*2, 200+400*0)

T_Inicio = time.time()
camino, nodos_explorados = A_star(mapa, inicio, objetivo)
T_Final = time.time()

# Configuración para visualización
fig = plt.figure(figsize=(15, 10))  # Tamaño de la figura
gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1])  # Mapa a la izquierda, mapa con Dijkstra a la derecha

# Subgráfico para el mapa original
ax0 = plt.subplot(gs[0])
ax0.imshow(mapa, cmap='Greys', origin='upper')
ax0.set_title('Mapa Original')

# Subgráfico para el mapa con el camino de Dijkstra
ax1 = plt.subplot(gs[1])
ax1.imshow(mapa, cmap='Greys', origin='upper')

# Visualización del camino en el mapa con Dijkstra
if camino:
    camino_x, camino_y = zip(*camino)
    ax1.plot(camino_y, camino_x, 'bo-', markersize=10, label='Camino')

# Visualización de los nodos explorados
if nodos_explorados:
    exp_x, exp_y = zip(*nodos_explorados)
    ax1.plot(exp_y, exp_x, 'ro', markersize=5, label='Nodos Explorados')

# Impresión de los puntos del camino
instruction=""
if camino:
    print("Puntos del camino:")
    for punto in camino:
        #print(f"({punto[1]}, {punto[0]})")
        instruction+=(f"{{{(punto[0]-200)/1000}, {(punto[1]-200)/1000}, 0}}, ")
        if punto == camino[-1]:
            print(instruction)


#Funcion para obtener el desplazamiendo gracias al camino
CantPuntos = len(camino)


def EntregaDesplazamiento(camino):
    # Si los puntos son 2D, se añade 0 para la rotación
    if len(camino[0]) == 2:
        camino = [(p[0], p[1], 0) for p in camino]
    CantPuntos = len(camino)
    Desplazamiento = np.zeros((3, CantPuntos - 1))
    for i in range(CantPuntos - 1):
        Desplazamiento[0, i] = (camino[i + 1][0] - camino[i][0]) * -1  # Δx
        Desplazamiento[1, i] = (camino[i + 1][1] - camino[i][1]) * -1 # Δy
        Desplazamiento[2, i] = camino[i + 1][2] - camino[i][2]  # Δrot
    return Desplazamiento

def EntregaVelocidad(Desplazamiento):
    num_disp = Desplazamiento.shape[1]
    vSeq = np.zeros((3, num_disp))
    for i in range(num_disp):  # Iteramos sobre todos los segmentos
        # Verificamos en x si hubo desplazamiento para asignar la velocidad
        if Desplazamiento[0, i] != 0:
            if Desplazamiento[0, i] > 0:
                vSeq[0, i] = 0.25
            else:
                vSeq[0, i] = -0.25
        # Verificamos en y si hubo desplazamiento para asignar la velocidad
        if Desplazamiento[1, i] != 0:
            if Desplazamiento[1, i] > 0:
                vSeq[1, i] = 0.25
            else:
                vSeq[1, i] = -0.25
        # Asignamos 0 para la rotación
        vSeq[2, i] = 0
    # Eliminamos la asignación final que forzaba 0
    # vSeq[:, -1] = 0
    return vSeq


T = np.zeros(CantPuntos - 1)
for i in range(CantPuntos - 1): 
    T[i] = 0.8

Desplazamiento = EntregaDesplazamiento(camino)
Velocidad = EntregaVelocidad(Desplazamiento)



print("Desplazamiento:")

print("{")
for i in range(3):  # Hay 3 filas en la matriz Desplazamiento
    fila = "    {"  # Iniciar fila
    for j in range(CantPuntos - 1):
        fila += f"{Desplazamiento[i, j]:.2f},"  # Agregar cada valor formateado con 2 decimales
    fila = fila.rstrip(",")  # Eliminar la coma al final
    fila += "},"  # Cerrar fila y agregar coma al final
    print(fila)
print("};")

print("Velocidad:")
print("{")
for i in range(3):  # Hay 3 filas en la matriz Desplazamiento
    fila = "    {"  # Iniciar fila
    for j in range(CantPuntos - 1):
        fila += f"{Velocidad[i, j]:.2f}, "  # Agregar cada valor formateado con 2 decimales
    fila = fila.rstrip(",")  # Eliminar la coma al final
    fila += "},"  # Cerrar fila y agregar coma al final
    print(fila)
print("};")

print("Tiempos:")
print("{", end="")
for i in range(len(T)):
    if i != len(T) - 1:
        print(f"{T[i]:.2f}, ", end="")
    else:
        print(f"{T[i]:.2f}", end="")
print("};")

#imprimo cant de puntos:
print("Cantidad de puntos:")
print(CantPuntos)


# Marcamos el inicio y el objetivo
ax1.plot(inicio[1], inicio[0], 'gs', markersize=10, label='Inicio')
ax1.plot(objetivo[1], objetivo[0], 'ms', markersize=10, label='Objetivo')

# Leyenda y etiquetas
ax1.legend()
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('Mapa con A*')


print(f"Tiempo de ejecución: {T_Final - T_Inicio} segundos")
plt.tight_layout()
plt.show()