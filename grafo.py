import random

# Implementacion del TDA Grafo
class Grafo:

    # Definicion de los datos del grafo.
    # Puede ser dirigido o no dirigido, segun el parametro "dirigido".
    # Se puede inicializar con vertices particulares con el parametro de "lista_vertices".
    def __init__(self, dirigido = True, lista_vertices = None):
        self.vertices = {}
        self.es_dirigido = dirigido
        if lista_vertices:
            for v in lista_vertices:
                self.vertices[v] = {}

    # Iterador del grafo.
    def __iter__(self):
        return iter(self.vertices)

    # Para poder utilizar len(grafo).
    def __len__(self):
        return len(self.vertices)

    # Agrega un vertice al grafo.
    # Devuelve True si el vertice se agrego al grafo.
    # Devuelve False si el vertice ya estaba en el grafo.
    def agregar_vertice(self, v):
        if v in self.vertices:
            return False
        else:
            self.vertices[v] = {}
            return True

    # Borra un vertice del grafo.
    # Devuelve True si el vertice fue borrado del grafo.
    # Devuelve False si el vertice no se encuentra en el grafo.
    def borrar_vertice(self, v):
        if v not in self.vertices:
            return False
        self.vertices.pop(v)
        for w in self.vertices:
            if v in self.vertices[w]:
                self.vertices[w].pop(v)
        return True

    # Agrega una arista entre dos vertices al grafo.
    # Con el parametro opcional "peso" se puede indicar el peso de la arista (por defecto es 1).
    # Devuelve True si la arista fue agregada al grafo.
    # Devuelve False si alguno de los vertices de la arista no se encuentra en el grafo.
    def agregar_arista(self, v, w, peso = 1):
        if v not in self.vertices or w not in self.vertices:
            return False
        self.vertices[v][w] = peso
        if not self.es_dirigido:
            self.vertices[w][v] = peso
        return True

    # Borra una arista del grafo.
    # Devuelve True si la arista fue borrada del grafo.
    # Devuelve False si alguno de los vertices de la arista no se encuentra en el grafo.
    def borrar_arista(self, v, w):
        if v not in self.vertices or w not in self.vertices:
            return False
        self.vertices[v].pop(w)
        if self.es_dirigido:
            self.vertices[w].pop(v)
        return True

    # Indica si los vertice v y w estan unidos.
    # Devuelve True si estan unidos.
    # Devuelve False si no estan unidos o si v no se encuentran en el grafo.
    def estan_unidos(self, v, w):
        if v not in self.vertices or w not in self.vertices:
            return False
        return w in self.vertices[v]

    # Devuelve el peso de la arista entre los vertices v y w.
    def peso_arista(self, v, w):
        if v not in self.vertices or w not in self.vertices:
            return
        if w in self.vertices[v]:
            return self.vertices[v][w]

    # Devuelve una lista con los vertices del grafo.
    def obtener_vertices(self):
        vertices = []
        for v in self.vertices:
            vertices.append(v)
        return vertices

    # Devuelve un vertice random del grafo.
    def vertice_aleatorio(self):
        return random.choice(list(self.vertices))

    # Devuelve una lista con los vertices adyacentes a v.
    def adyacentes(self, v):
        if v not in self.vertices:
            return []
        return list(self.vertices[v])
