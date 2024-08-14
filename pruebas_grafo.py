from grafo import Grafo

from collections import deque

def _fronteras():
    PAISES = ["ARG", "BRA", "URU", "CHI", "PER", "PAR", "BOL", "ECU", "VEN", "COL", "SUR", "GUY", "GUF"]
    g = Grafo(False, PAISES)
    g.agregar_arista("ARG", "URU")
    g.agregar_arista("ARG", "CHI")
    g.agregar_arista("ARG", "BOL")
    g.agregar_arista("ARG", "BRA")
    g.agregar_arista("ARG", "PAR")
    g.agregar_arista("BRA", "URU")
    g.agregar_arista("BRA", "PAR")
    g.agregar_arista("BRA", "BOL")
    g.agregar_arista("BRA", "SUR")
    g.agregar_arista("BRA", "GUF")
    g.agregar_arista("BRA", "GUY")
    g.agregar_arista("BRA", "VEN")
    g.agregar_arista("BRA", "COL")
    g.agregar_arista("BRA", "PER")
    g.agregar_arista("CHI", "BOL")
    g.agregar_arista("CHI", "PER")
    g.agregar_arista("PAR", "BOL")
    g.agregar_arista("PER", "BOL")
    g.agregar_arista("ECU", "PER")
    g.agregar_arista("ECU", "COL")
    g.agregar_arista("COL", "PER")
    g.agregar_arista("COL", "VEN")
    g.agregar_arista("VEN", "GUY")
    g.agregar_arista("SUR", "GUY")
    g.agregar_arista("SUR", "GUF")
    return g

def no_adyacentes(grafo, n):
    vertices = grafo.obtener_vertices()
    puestos = set()
    return _no_adyacentes(grafo, vertices, 0, puestos, n)

def _no_adyacentes(grafo, vertices, v_actual, puestos, n):
    if v_actual == len(grafo):
        return False
    if len(puestos) == n:
        return es_compatible(grafo, puestos)
    if not es_compatible(grafo, puestos):
        return False
    puestos.add(vertices[v_actual])
    if _no_adyacentes(grafo, vertices, v_actual + 1, puestos, n):
        return True
    puestos.remove(vertices[v_actual])
    return _no_adyacentes(grafo, vertices, v_actual + 1, puestos, n)

def es_compatible(grafo, puestos):
    for v in puestos:
        for w in puestos:
            if v == w:
                continue
            if w in grafo.adyacentes(v):
                return False
    return True

def diametro(grafo):
    max_min_dist = 0
    for v in grafo:
        distancias = caminos_minimos(grafo, v)
        for w in distancias:
            if distancias[w] > max_min_dist:
                max_min_dist = distancias[w]
    return max_min_dist

def caminos_minimos(grafo, origen):
    q = deque()
    visitados = set()
    distancia = {}
    distancia[origen] = 0
    visitados.add(origen)
    q.append(origen)
    while q:
        v = q.popleft()
        for w in grafo.adyacentes(v):
            if w not in visitados:
                distancia[w] = distancia[v] + 1
                q.append(w)
                visitados.add(w)
    return distancia

def colorear(grafo, n):
    coloreados = {}
    vertices = grafo.obtener_vertices()
    return coloreo(grafo, 0, coloreados, n, vertices)

def coloreo(grafo, v, coloreados, colores, vertices):
    if len(coloreados) == len(grafo):
        return True
    for c in range(colores):
        coloreados[v] = c
        if not es_valido(grafo, v, coloreados, vertices):
            coloreados.pop(v)
            continue
        if coloreo(grafo, v + 1, coloreados, colores, vertices):
            return True
    if v in coloreados:
        coloreados.pop(v)
    return False

def es_valido(grafo, v, coloreados, vertices):
    for w in grafo.adyacentes(vertices[v]):
        w_indice = vertices.index(w)
        if (w_indice in coloreados and coloreados[w_indice] == coloreados[v]):
            return False
    return True

def es_bipartito(grafo):
    color = {}
    VERDE = 1
    ROJO = 2
    vertices = grafo.obtener_vertices()
    if not vertices:
        return True
    for v in vertices:
        color[v] = None
    vertice_random = vertices[0]
    color[vertice_random] = VERDE
    cola = deque()
    cola.append(vertice_random)

    while cola:
        vertice = cola.popleft()
        for v in grafo.adyacentes(vertice):
            if color[vertice] == color[v]:
                return False
            if color[v] == None:
                if color[vertice] == VERDE: color[v] = ROJO
                else: color[v] = VERDE
                cola.append(v)
    return True

def grado_vertices(grafo):
    grado = {}
    for v in grafo.obtener_vertices():
        grado[v] = len(grafo.adyacentes(v))
    return grado

def obtener_aristas(grafo):
    aristas = []
    visitados = set()
    for v in grafo:
        visitados.add(v)
        for w in grafo.adyacentes(v):
            if w not in visitados:
                aristas.append((v, w))
    return aristas

def _dfs_conexo(grafo, v, visitados):
    visitados.add(v)
    for w in grafo.adyacentes(v):
        if w not in visitados:
            _dfs_conexo(grafo, w, visitados)


def es_conexo(grafo):
    visitados = set()
    _dfs_conexo(grafo, grafo.vertice_aleatorio(), visitados)
    return len(visitados) == len(grafo)


def main():
    fronteras = _fronteras()
    print(es_conexo(fronteras))
    print(grado_vertices(fronteras))
    print(es_bipartito(fronteras))
    print(colorear(fronteras, 4))
    print(diametro(fronteras))
    print(no_adyacentes(fronteras, 7))

if __name__ == "__main__":
    main()
