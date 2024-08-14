from grafo import Grafo
from collections import deque

#-----------------------------------------------------------------#
#                      FUNCIONES COMPARTIDAS
#-----------------------------------------------------------------#

def bfs(grafo, origen):
    """
    Recorrido BFS (Breadth First Search) de un grafo, partiendo de un origen especifico
    pasado por parametro. Complejidad: O(P + L).
    PRE: Recibe un grafo a recorrer y un vertice origen.
    POST: Se hizo un recorrido BFS del grafo y devuelve los diccionarios de los padres
    y el orden, para poder reconstruir el recorrido.
    """
    visitados = set()
    padres = {}
    orden = {}
    padres[origen] = None
    orden[origen] = 0
    visitados.add(origen)
    q = deque()
    q.append(origen)
    while q:
        v = q.popleft()
        for w in grafo.adyacentes(v):
            if w not in visitados:
                padres[w] = v
                orden[w] = orden[v] + 1
                visitados.add(w)
                q.append(w)
    return padres, orden

def recorrido_reconstruir(padres, destino):
    """
    Reconstruye un recorrido con el diccionario de padres y un vertice destino.
    PRE: Recibe un diccionario de padres y un vertice destino.
    POST: Devuelve en una lista el recorrido resultante.
    """
    recorrido = []
    recorrido.append(destino)
    padre = padres[destino]
    while padre:
        recorrido.append(padre)
        padre = padres[padre]
    recorrido.reverse()
    return recorrido

def recorrido_imprimir(recorrido, costo = None):
    """
    Muestra por pantalla el recorrido, y si se desea, el costo del mismo.
    PRE: Recibe una lista con un recorrido y opcionalmente, un numero que representa el costo.
    POST: Se imprime el recorrido y el costo si se paso como parametro.
    """
    print(recorrido[0], end = "")
    for i in range(1, len(recorrido)):
        print(" ->", recorrido[i], end = "", flush = True)
    print()
    if costo:
        print("Costo:", costo)


#-----------------------------------------------------------------#
#                            COMANDOS
#-----------------------------------------------------------------#

#-----------------------------------------------------------------#
#                            CAMINO
#-----------------------------------------------------------------#

def camino(grafo, origen, destino):
    """
    Comando Camino mas corto
    Complejidad: O(P + L)
    Recibe un grafo, un vertice origen y uno destino, y devuelve una lista con
    el camino de la pagina origen a la pagina destino, navegando lo menos posible.
    PRE: Recibe un grafo, un vertice origen y un vertice destino.
    POST: Devuelve en una lista el minimo recorrido entre los vertices.
    """
    padres, costos = bfs(grafo, origen)
    if destino not in padres:
        return None, -1
    camino = recorrido_reconstruir(padres, destino)
    return camino, costos[destino]

#-----------------------------------------------------------------#
#                            CONECTADOS
#-----------------------------------------------------------------#
def componentes_fuertemente_conexas(grafo, v, visitados, apilados, todas_cfc, orden, mb, pila, orden_contador):
    """
    Componentes Fuertemente Conexas 
    Implementado para grafos dirigidos con el Algoritmo de Tarjan, se basa en un recorrido DFS.
    Complejidad = O(V + E)
    """
    visitados.add(v)
    pila.appendleft(v)
    apilados.add(v)
    mb[v] = orden[v]

    for w in grafo.adyacentes(v):
        if w not in visitados:
            orden[w] = orden_contador + 1
            orden_contador += 1
            visitados, apilados, todas_cfc, orden, mb, pila, orden_contador = componentes_fuertemente_conexas(grafo, w, visitados, apilados, todas_cfc, orden, mb, pila, orden_contador)
            mb[v] = min(mb[v], mb[w])
        elif w in apilados:
            mb[v] = min(mb[v], orden[w])

    if orden[v] == mb[v]:
        nueva_cfc = []
        while True:
            w = pila.popleft()
            apilados.remove(w)
            nueva_cfc.append(w)
            if w == v:
                break
        todas_cfc.append(nueva_cfc)
    return visitados, apilados, todas_cfc, orden, mb, pila, orden_contador


def conectados(grafo, pagina):
    """
    Comando Conectividad
    Complejidad: O(P + L) si se ejecuta por primera vez, O(1) si se vuelve a ejecutar.
    Obtiene todas las paginas (vertices) a las que se puede llegar desde la pagina
    pasada por parametro y que tambien puedan llegar a esa pagina. Es decir que
    obtiene la componente fuertemente conexa (CFC) a la que pertenece la pagina.
    PRE: Recibe un grafo y una pagina.
    POST: Devuelve todas las paginas que pertenecen a la CFC de la pagina pasada por parametro.
    """
    visitados = set()
    apilados = set()
    todas_cfc = []
    orden = {}
    orden[pagina] = 0
    mb = {}
    pila = deque()
    orden_contador = 0

    visitados, apilados, todas_cfc, orden, mb, pila, orden_contador = componentes_fuertemente_conexas(grafo, pagina, visitados, apilados, todas_cfc, orden, mb, pila, orden_contador)

    resultado = set()
    for cfc in todas_cfc:
        if pagina in cfc:
            for elemento in cfc:
                resultado.add(elemento)
    return resultado

#-----------------------------------------------------------------#
#                            CICLO
#-----------------------------------------------------------------#

def _ciclo(grafo, pagina, largo, recorrido, visitados, v_actual):
    """
    Algoritmo de backtracking utilizado para calcular el ciclo de largo n.
    Funcion recursiva que intenta obtener un ciclo valido.
    Si el recorrido no es valido, devuelve False y se realiza la poda de esa rama.
    PRE: Recibe un grafo, el vertice origen y destino, el largo del ciclo, el recorrido hasta ahora,
    los vertices visitados y el vertice actual en ese punto de la recursividad.
    POST: Devuelve True si el recorrido es valido hasta ese punto, en caso contrario False.
    """
    largo_ciclo = len(recorrido)
    if largo_ciclo == largo:
        if v_actual != pagina:
            return False
        recorrido.append(pagina)
        return True
    if largo_ciclo > largo:
        return False
    if v_actual in visitados:
        return False
    visitados.add(v_actual)
    recorrido.append(v_actual)
    for v in grafo.adyacentes(v_actual):
        if _ciclo(grafo, pagina, largo, recorrido, visitados, v):
            return True
    recorrido.pop()
    return False

def ciclo(grafo, pagina, largo):
    """
    Comando Ciclo de n articulos
    Complejidad: O(P^n)
    Obtiene un ciclo que comienza y termina en la pagina indicada, con el largo deseado.
    PRE: Recibe un grafo, un vertice origen y el largo del ciclo.
    POST: Devuelve el ciclo de largo n.
    """
    recorrido = []
    visitados = set()
    if not _ciclo(grafo, pagina, largo, recorrido, visitados, pagina):
        return None
    return recorrido

#-----------------------------------------------------------------#
#                            LECTURA
#-----------------------------------------------------------------#

def _dfs_modificado(grafo, v, visitados, visitando, cola):
    """
    Recorrido DFS (Depth First Search) de un grafo, partiendo de un vertice v.
    Complejidad = O(P + L).
    Este recorrido es utilizado por el comando de Lectura para obtener el orden
    deseado. Es un DFS modificado ya agrega los elementos del recorrido a una cola
    y devuelve False si encuentra algun ciclo.
    PRE: Recibe un grafo, un vertice v, un conjunto de visitados y otro de visitando,
    que corresponde a los vertices que se estan visitando en la recursividad, y una cola.
    POST: Si no hay ciclos, agrega los vertices del recorrido a la cola y devuelve True.
    Si hay ciclos, devuelve False.
    """
    visitando.add(v)
    for w in grafo.adyacentes(v):
        if w in visitando:
            return False
        if w not in visitados:
            if not _dfs_modificado(grafo, w, visitados, visitando, cola):
                return False
    cola.append(v)
    visitando.remove(v)
    visitados.add(v)
    return True

def topologico_dfs_modificado(grafo):
    """
    Orden topologico por DFS modificado para el comando lectura.
    Se obtiene un orden topologico "invertido" con respecto a un orden
    topologico estandar, para obtener el orden de lectura deseado.
    PRE: Recibe un grafo.
    POST: Devuelve una lista con el orden de lectura deseado.
    """
    visitados = set()
    visitando = set()
    cola = deque()
    for v in grafo:
        if v not in visitados and v not in visitando:
            if not _dfs_modificado(grafo, v, visitados, visitando, cola):
                return []
    return cola_a_lista(cola)

def cola_a_lista(cola):
    """
    Transfiere los elementos de una cola a una lista.
    PRE: Recibe una cola con elementos.
    POST: Devuelve una lista con los elementos de la cola.
    """
    lista = []
    while cola:
        lista.append(cola.popleft())
    return lista

def lectura_imprimir(orden_lectura):
    """
    Muestra por pantalla el orden de lectura.
    PRE: Recibe una lista con el orden de lectura.
    POST: Se imprime el orden de las paginas.
    """
    print(orden_lectura[0], end = "")
    for i in range(1, len(orden_lectura)):
        print(",", orden_lectura[i], end = "", flush = True)
    print()

def lectura(grafo, paginas):
    """
    Comando Lectura a las 2 a.m.
    Complejidad: O(n + Ln)
    Dado un grafo y un listado de paginas (vertices), obtiene un orden
    de lectura valido para leer esas paginas.
    PRE: Recibe un grafo y una lista de paginas o vertices.
    POST: Devuelve una lista con las paginas en el orden de lectura.
    """
    grafo_auxiliar = Grafo(True)
    for v in paginas:
        grafo_auxiliar.agregar_vertice(v)
        for w in grafo.adyacentes(v):
            if w in paginas:
                grafo_auxiliar.agregar_vertice(w)
                grafo_auxiliar.agregar_arista(v, w, 0)
    return topologico_dfs_modificado(grafo_auxiliar)

#-----------------------------------------------------------------#
#                            DIAMETRO
#-----------------------------------------------------------------#

def diametro(grafo):
    """
    Comando Diametro
    Complejidad: O(P(P + L))
    Calcula el diametro de toda la red, es decir el camino minimo mas grande del grafo.
    PRE: Recibe un grafo.
    POST: Devuelve el valor del diametro y el recorrido del camino minimo.
    """
    diametro = 0
    vertice_diam_max = None
    padres_diam_max = {}
    for v in grafo.obtener_vertices():
        padres, distancias = bfs(grafo, v)
        for w in distancias:
            if distancias[w] > diametro:
                diametro = distancias[w]
                vertice_diam_max = w
                padres_diam_max = padres
    recorrido = recorrido_reconstruir(padres_diam_max, vertice_diam_max)
    return recorrido, diametro

#-----------------------------------------------------------------#
#                            RANGO
#-----------------------------------------------------------------#

def rango(grafo, pagina, n_links):
    """
    Comando Todos en Rango
    Complejidad: O(P + L)
    Calcula la cantidad de paginas que se encuentran a n links o saltos respecto
    a la pagina indicada.
    PRE: Recibe un grafo, una pagina (vertice) y la cantidad de links o saltos.
    POST: Devuelve el rango, la cantidad de paginas a n links o saltos.
    """
    padres, distancias = bfs(grafo, pagina)
    rango = 0
    for v in distancias:
        if distancias[v] == n_links:
            rango += 1
    return rango

#-----------------------------------------------------------------#
#                            NAVEGACION
#-----------------------------------------------------------------#

def navegacion(grafo, origen):
    """
    Comando Navegacion por primer link
    Complejidad: O(n)
    Navega por los primeros links desde la pagina origen hasta que no haya
    mas primeros links, o hasta que se haya navegado por 20 paginas.
    PRE: Recibe un grafo y una pagina (vertice) origen.
    POST: Devuelve una lista con el recorrido de navegar por los primeros links.
    """
    MAX_PAGINAS_NAV = 20
    recorrido = []
    recorrido.append(origen)
    for pagina in range(MAX_PAGINAS_NAV):
        ady = grafo.adyacentes(origen)
        if not ady:
            break
        origen = ady[0]
        recorrido.append(origen)
    return recorrido

#-----------------------------------------------------------------#
#                            CLUSTERING
#-----------------------------------------------------------------#

def calcular_clustering(grafo, vertice):
    """
    Calcula el Coeficiente de Clustering de un vertice en particular.
    PRE: Recibe un grafo, y un vertice.
    POST: Devuelve el coeficiente de clustering de ese vertice.
    """
    ady = set(grafo.adyacentes(vertice))
    if vertice in ady:
        ady.remove(vertice)
    cantidad_ady = len(ady)
    if cantidad_ady < 2:
        return 0
    aristas = 0
    for v in ady:
        for w in grafo.adyacentes(v):
            if v == w: continue
            if w in ady:
                aristas += 1
    return aristas / (cantidad_ady*(cantidad_ady - 1))

def clustering (grafo, pagina = None):
    """
    Comando Coeficiente de Clustering
    Complejidad: en caso de recibir una pagina es O(1), sino es O(P + L)^2
    Imprime el coeficiente de clustering de una pagina en particular o el de toda la red.
    PRE: Recibe un grafo, y puede recibir una pagina (vertice) o no.
    POST: Devuelve el coeficiente de clustering.
    """
    if pagina:
        return calcular_clustering(grafo, pagina)
    total = 0
    for v in grafo:
        total += calcular_clustering(grafo, v)
    return (1 / len(grafo)) * total
