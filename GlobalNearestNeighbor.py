import random
import math

class Node:
    def __init__(self, id, tp, dem, xx, yy):
        self.id = id
        self.type = tp
        self.demand = dem
        self.x = xx
        self.y = yy
        self.isRouted = False


class Route:
    def __init__(self, dp, cap):
        self.sequenceOfNodes = []
        self.sequenceOfNodes.append(dp)
        self.cost = 0
        self.capacity = cap
        self.load = 0

all_nodes = []
service_locations = []
depot = Node(0, 0, 0, 50, 50)
all_nodes.append(depot)
random.seed(1)

for i in range(0, 200):
    id = i + 1
    tp = random.randint(1,3)
    dem = random.randint(1,5) * 100
    xx = random.randint(0, 100)
    yy = random.randint(0, 100)
    serv_node = Node(id, tp, dem, xx, yy)
    all_nodes.append(serv_node)
    service_locations.append(serv_node)

dist_matrix = [[0.0 for j in range(0, len(all_nodes))] for k in range(0, len(all_nodes))]
for i in range(0, len(all_nodes)):
    for j in range(0, len(all_nodes)):
        source = all_nodes[i]
        target = all_nodes[j]
        dx_2 = (source.x - target.x)**2
        dy_2 = (source.y - target.y) ** 2
        dist = round(math.sqrt(dx_2 + dy_2))
        dist_matrix[i][j] = dist / 35 + 5/60 + (all_nodes[j].type - 1) * 10/60


routes=[]
rt = Route(depot, 3000)
routes.append(rt)

def GlobalNearest():
    for i in range(0, len(all_nodes)):
       mininsertion=1000
       routemin=0
       for r in range(0,len(routes)):
           idoflast=routes[r].sequenceOfNodes[-1].id
           for j in range(1, len(all_nodes)):
               if all_nodes[j].isRouted is False and routes[r].load+all_nodes[j].demand<3000:
                   if dist_matrix[idoflast][j]<mininsertion:
                       mininsertion=dist_matrix[idoflast][j]
                       routemin=r
                       neighborid=j
       if mininsertion!=1000:
           all_nodes[neighborid].isRouted=True
           routes[routemin].sequenceOfNodes.append( all_nodes[neighborid])
           routes[routemin].load= routes[routemin].load +all_nodes[neighborid].demand
           routes[routemin].cost= routes[routemin].cost + mininsertion
           if len(routes)<25:
                   routes.append(Route(depot,3000))


GlobalNearest()




for i in range(0, len(routes)):
            rt = routes[i]
            for j in range (0, len(rt.sequenceOfNodes)):
                print(rt.sequenceOfNodes[j].id, end=' ',)
            print(rt.cost)
            print("\n")

