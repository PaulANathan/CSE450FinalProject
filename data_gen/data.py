import random
import visualize
from visualize import visualize

def make_some_test_data():

    vertices = []
    edges = [[0 for i in range(10)] for j in range(10)] #edges = [][]

    # Generate sample vertices
    for i in range(1,11):
        x = random.randint(1,21)
        y = random.randint(1,21)
        vertex = [x,y]
        # add new pair to graph
        vertices.append(vertex)
    print vertices

    # Generate sample edge costs
    for v1 in range(len(vertices)): #for v1 in vertices:
        for v2 in range(len(vertices)): #for v2 in vertices:
            cost = random.randint(1,11)
            edges[v1][v2] = cost
    print edges

    visualize.visual_some_stuff(vertices, edges)
    return None
    #return {vertices, edges}