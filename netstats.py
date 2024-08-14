#!/usr/bin/python3

import sys
import csv

from grafo import Grafo
from netstats_comandos import *

sys.setrecursionlimit(100000)

#-----------------------------------------------------------------#
#                      FUNCIONES AUXILIARES
#-----------------------------------------------------------------#

def listar_operaciones():
    """
    Imprime un listado con los comandos disponibles.
    """
    print("camino")
    print("conectados")
    print("ciclo")
    print("lectura")
    print("diametro")
    print("rango")
    print("navegacion")
    print("clustering")

def cargar_grafo(grafo):
    """
    Lee un archivo, cuya ruta fue pasada como parametro del programa, y carga
    el grafo con los datos contenidos en este archivo.
    PRE: Recibe un grafo.
    POST: El grafo recibido fue cargado con los datos.
    """
    wiki = open(sys.argv[1], encoding = "utf-8")
    tsv = csv.reader(wiki, delimiter = '\t')
    for linea in tsv:
        grafo.agregar_vertice(linea[0])
        for i in range(1, len(linea)):
            grafo.agregar_vertice(linea[i])
            grafo.agregar_arista(linea[0], linea[i], peso = 1)
    wiki.close()

def validar_params():
    """
    Chequea que los parametros pasados al programa por consola sean validos.
    """
    if len(sys.argv) != 2:
        print("Error: parametros invalidos")
        return False
    return True
    
def pasar_a_archivo(resultado):
    """
        Pasa el resultado a un archivo, y lo imprime.
    """
    with open('resultado.txt','w') as archivo:
        for item in resultado:
            archivo.write("%s," % item)
    file = open('resultado.txt','r')
    print(file.read())

#-----------------------------------------------------------------#
#                           NETSTATS
#-----------------------------------------------------------------#

def netstats():
    """
    Crea un grafo, y lo carga con el contenido de Wikipedia.
    Ejecuta los comandos ingresados por entrada estandar.
    """
    grafo = Grafo(dirigido = True, lista_vertices = None)
    cargar_grafo(grafo)
    diam_grafo = None

    for linea in sys.stdin:
        if linea == "\n":
            break
        linea_aux = linea.rstrip('\n').split(' ', 1)
        if len(linea_aux) > 1:
            parametros = linea_aux[1].split(',')
        if linea_aux[0] == "listar_operaciones":
            listar_operaciones()
        elif linea_aux[0] == "camino":
            camino_obtenido, costo = camino(grafo, parametros[0], parametros[1])
            if camino_obtenido:
                recorrido_imprimir(camino_obtenido, costo)
            else:
                print("No se encontro recorrido")
         elif linea_aux[0] == "conectados":
            if not resultado or parametros[0] not in resultado:
                resultado = conectados(grafo, parametros[0])
                pasar_a_archivo(resultado)
                print(len(resultado))
            else:
                pasar_a_archivo(resultado)
                print(len(resultado))
        elif linea_aux[0] == "ciclo":
            ciclo_obtenido = ciclo(grafo, parametros[0], int(parametros[1]))
            if ciclo_obtenido:
                recorrido_imprimir(ciclo_obtenido)
            else:
                print("No se encontro recorrido")
        elif linea_aux[0] == "lectura":
            orden_lectura = lectura(grafo, parametros)
            if len(orden_lectura) == 0:
                print("No existe forma de leer las paginas en orden")
            else:
                lectura_imprimir(orden_lectura)
        elif linea_aux[0] == "diametro":
            if not diam_grafo:
                diam_grafo = diametro(grafo)
            recorrido_imprimir(diam_grafo[0], diam_grafo[1])
        elif linea_aux[0] == "rango":
            print(rango(grafo, parametros[0], int(parametros[1])))
        elif linea_aux[0] == "navegacion":
            recorrido_imprimir(navegacion(grafo, parametros[0]))
        elif linea_aux[0] == "clustering":
            if len(linea_aux) == 1:
                print("{:.3f}".format(clustering(grafo)))
            else:
                print("{:.3f}".format(clustering(grafo, parametros[0])))
        else:
            print("Comando invalido")

if __name__ == "__main__":
    if not validar_params():
        exit()
    netstats()
