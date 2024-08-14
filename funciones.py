#RECORRIDOS
#BFS ORDEN: O(V + E) siempre y cuando sea con lista de ady con diccionarios o diccionarios de diccionarios, MA O(V*V)
def bfs (grafo,origen):
	visitados = set() #tiene q ser conjunto no lista!!
	padres = {}
	orden = {}
	padres[origen] = None
	orden[origen] = 0
	visitados.add(origen)
	q = cola ()
	q.encolar(origen)
	while not q.esta_vacia():
		v = q.desencolar()
		for w in grafo.adyacentes(v): #esta primitiva es la q más cambia el orden.. DD /LD O(E)... MA O(V)
			if w not in visitados:
				padres[w] = v
				orden[w] = orden[v] + 1
				visitados.add(w)
				q.enolar(w)
	return padres, orden 
#DFS ORDEN:
def _dfs(grafo, v, visitados, padres, orden):
	for w in grafo.adyacentes(v):
		if w not in visitados:
			visitados.add(w)
			padres[w] = v
			orden[w] = orden[v] + 1
			_dfs(grafo,w,visitados,padres,orden)

def dfs (grafo,origen):
	padres = {}
	orden = {}
	visitados = set()
	padres[origen] = None
	orden[origen] = 0
	visitados.add(origen)
	_dfs(grafo,origen,visitados,padres,orden)
	return padres,orden
#esto es un recorrido empezando de origen en particular

#este es completo q empieza de cualquier vertice
def recorrido_dfs_completo (grafo):
	visitados = set()
	padres = {}
	orden = {}
	for v in grafo:
		if v not in visitados:
			visitados.add(v)
			padre[v] = None
			orden[v] = 0
			_dfs(grafo,v,visitados,padre,orden)
	return padres,orden

#Obtener aristas
def obtener_aristas (grafo):
	aristas = []
	visitados = set() //grafo no dirigido
	for v in grafo:
		for w in grafo.adyacentes(v):
			if w in visitados: //ND
				continue
			aristas.append((v , w))
		visitados.add(v) //ND
	return aristas

def _dfs(grafo, v, visitados):
    for w in grafo.adyacentes(v):
        if w not in visitados:
        	visitados.add(w)
            _dfs(grafo, w, visitados)

def componentes_conexas(grafo):
    componentes = 0
    visitados = set()
    for v in visitados:
    	componentes += 1
    	_dfs(grafo,v,visitados)
    return componentes

#Implementar un algoritmo que, dado un grafo no dirigido, nos devuelva un ciclo dentro del mismo, si es que los tiene. 
#Indicar el orden del algoritmo:  O(V + 2 E) = O(V + E) = O(V + E).
def reconstruir_ciclo(padre, inicio, fin):
  v = fin
  camino = []
  while v != inicio:
    camino.append(v)
    v = padre[v]
  camino.append(inicio)
  return camino.invertir()

#FORMA RECORRIDO BFS
def obtener_ciclo_bfs(grafo):
  visitados = {}
  for v in grafo:
    if v not in visitados:
      ciclo = bfs_ciclo(grafo, v, visitados)
      if ciclo is not None:
        return ciclo
  return None
def bfs_ciclo(grafo, v, visitados):
  q = Cola()
  q.encolar(v)
  visitados[v] = True
  padre = {}  # Para poder reconstruir el ciclo
  padre[v] = None

  while not q.esta_vacia():
    v = q.desencolar()
    for w in grafo.adyacentes(v):
      if w in visitados:
        ##Si w fue visitado y es padre de v, entonces es la arista de donde vengo (no es ciclo).
        #Si no es su padre, esta arista (v, w) cierra un ciclo que empieza en w.
        if w != padre[v]:
          return reconstruir_ciclo(padre, w, v)
      else:
        q.encolar(w)
        visitados[w] = True
        padre[w] = v

  #Si llegamos hasta acá es porque no encontramos ningún ciclo.
  return Non

#FORMA RECORRIDO DFS
def obtener_ciclo_dfs(grafo):
  visitados = {}
  padre = {}
  for v in grafo:
    if v not in visitados:
      ciclo = dfs_ciclo(grafo, v, visitados, padre)
      if ciclo is not None:
        return ciclo
  return None

def dfs_ciclo(grafo, v, visitados, padre):
  visitados[v] = True
  for w in grafo.adyacentes(v):
    if w in visitados:
      // Si w fue visitado y es padre de v, entonces es la arista de donde vengo (no es ciclo).
      // Si no es su padre, esta arista (v, w) cierra un ciclo que empieza en w.
      if w != padre[v]:
        return reconstruir_ciclo(padre, w, v)
    else:
      padre[w] = v
      ciclo = dfs_ciclo(grafo, w, visitados, padre)
      if ciclo is not None:
        return ciclo

  #Si llegamos hasta acá es porque no encontramos ningún ciclo.
  return None

#ORDEN TOPOLOGICO

#DE GRADOS DE ENTRADA ORDEN: O(V + E)
def grados_entrada(grafo):
    g_ent = {}
    for v in grafo:
        g_ent[v] = 0
    for v in grafo:
        for w in grafo.adyacentes(v):
            g_ent[w] += 1
    return g_ent
    // O(V + E)
def topologico_grados(grafo):
    g_ent = grados_entrada(grafo) #// O (V + E)
    q = Cola()
    for v in grafo: // O(V)
        if g_ent[v] == 0:
            q.encolar(v)
    resultado = []
    while not q.esta_vacia(): #//O(V´E)
        v = q.desencolar()
        resultado.append(v)
        for w in grafo.adyacentes(v):
            g_ent[w] -= 1
            if g_ent[w] == 0:
                q.encolar(w)

    if len(resultado) != len(grafo) //hay un ciclo!!!
    	return None

    return resultado

#DE DFS ORDEN: O(V + E)
def _dfs(grafo, v, visitados, pila):
    visitados.add(v)
    for w in grafo.adyacentes(v):
        if w not in visitados:
            _dfs(grafo, w, visitados, pila)
    pila.apilar(v)
def topologico_dfs(grafo):
    visitados = set()
    pila = Pila()
    for v in grafo:
        if v not in visitados:
            _dfs(grafo, v, visitados, pila)
    return pila_a_lista(pila)
def pila_a_lista(pila):
    lista = []
    while not pila.esta_vacia():
        lista.append(pila.desapilar())
    return lista


def componentes_fuertemente_conexas(v):
  global orden_contador #m esta mal??

  visitados.add(v)
  orden[v] = orden_contador
  mb[v] = orden[v]
  orden_contador += 1
  pila.appendleft(v)
  apilados.add(v)

  for w in ady[v]:
    if w not in visitados:
      componentes_fuertemente_conexas(w)

    if w in apilados:
      if mb[v] > mb[w]:
      mb[v] = min(mb[v], mb[w])
    
  if orden[v] == mb[v] and len(pila) > 0:
    nueva_cfc = []
    while True:
      w = pila.popleft()
      apilados.remove(w)
      nueva_cfc.append(w)
      if w == v:
        break
    
    todas_cfc.append(nueva_cfc)

#Grafo no pesado-> Usa una cola para mantener el orden de los elementos, complejidad O(V + E) recorre vertice y x cada uno encola una arista
def camino_minimo(grafo,origen):
	distancia, padre, visitado = {}, {}, {}
	for v in grafo:
		distancia[v] = inf
	distancia[origen] = 0
	padre[origen] = None
	visitado[origen] = True
	q = cola_crear()
	q.encolar(origen)
	while not q.esta_vacia():
		v = q.desencolar()
		for w in grafo.adyacentes(v):
			if (v not in visitado):
				distancia[w] += distancia[v] + 1
				padre[w] = v
				visitado[w] = True
				q.encolar(w)
	return padre,distancia

def obtener_aristas(grafo):
	aristas = []
	for v in grafo:	
		for w in grafo.adyacentes(v):
			aristas.append((v,w,grafo.peso(v,w)))
	return aristas

# CENTRALIDAD -> Determinar cuan importante es un vertice dentro de una red
#en particular
#	-de grado: el grado del vertice en cuestion
#	-cercania: promedio de las distancias del vertice a todos los demas
#	-de autovector: cuan influyente es un nodo en el resto..
#	-Betweeness!!!: frecuencia con la que aparece el vertice en todos los caminos
#	minimos del grado (en los que no es extremo)

#	-usando Dijkstra O(V^2 (E log V))
#	-usando BFS O(V^2 (E + V)) -es no pesado-

def centralidad(grafo):
	cent = {}
	for v in grafo: cent[v] = 0 //doble for es V^2
	for v in grafo: //para cada nodo buscamos camino minimo a todo el resto 
		for w in grafo:
			if v == w: continue //no se calcula camino min consigo mismo
			distancia, padre = camino minimo(grafo, v, w)
			if padre[w] is NULL: continue //no existe camino minimo
			actual = padre[w] //recorre de destino para atras
			while actual != v:
				cent[actual] += 1
				actual = padre[actual]
	return cent

def mas_centrales(grafo):
	cent = centralidad(grafo)
	return max(cent)

#diametro -> de una red (grafo, etc.) es la distancia más grande entre todos los caminos mínimos que hayan. 
#Por lo tanto, será necesario obtener todas las distancias mínimas y quedarnos con el máximo valor.
#COMPLEJIDAD: O(V(V+E)) 
def caminos_minimos(grafo, origen)
    q = Cola()
    visitados = set()
    distancia = {}
    distancia[origen] = 0
    visitados.add(origen)
    q.encolar(origen)

    while not q.esta_vacia():
        v = q.desencolar()
        for w in grafo.adyacentes(v):
            if w not in visitados:
                distancia[w] = distancia[v] + 1
                q.encolar(w)
                visitados.add(w)
    return distancia
def diametro(grafo):
    max_min_dist = 0
    for v in grafo:
        distancias = caminos_minimos(grafo, v)
        for w in distancias:
            if distancias[w] > max_min_dist:
                max_min_dist = distancias[w]
    return max_min_dist

