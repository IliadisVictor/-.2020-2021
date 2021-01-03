import random
import math

class Node:
    def __init__(self, id, tp, dem, xx, yy):
        self.id = id
        self.type = tp
        self.demand = dem
        self.x = xx
        self.y = yy


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



# Clark Heuristic

def savingslist():
    a = []
    for i in range(1, len(all_nodes)):
        for j in range(1 + i, len(all_nodes)):
            d = dist_matrix[0][j] - dist_matrix[i][j]
            a.append([i, j, d])
    a = sorted(a, key=lambda x: x[-1], reverse=True)
    return a

def FindRouteWithNode(routeid,routes):
    for a in routes:
        for i in a.sequenceOfNodes:
            if routeid==i.id:
                return a

def FirstOrLast(routeid,route):
    sequence=route.sequenceOfNodes
    if routeid==sequence[-1].id or routeid==sequence[1].id:
        return True
    else:
        return False

def BreachCapacity(routei,routej,routes1):
    if routei.load+routej.load <routei.capacity:
        routei.load=routei.load+routej.load
        routei.cost = routei.cost + routej.cost+dist_matrix[routei.sequenceOfNodes[-1].id][routej.sequenceOfNodes[1].id]-dist_matrix[0][routej.sequenceOfNodes[1].id]
        for i in range(1, len(routej.sequenceOfNodes)):
            routei.sequenceOfNodes.append(routej.sequenceOfNodes[i])
        routes1.remove(routej)

def returnmax(routes):
    max=0
    for i in range(0, len(routes)):
        rt = routes[i]
        if max < rt.cost:
            max = rt.cost
    return max

def CreateRoutes():
 routes=[]
 for i in range(1,201):
    rt=Route(depot,3000)
    rt.sequenceOfNodes.append(all_nodes[i])
    rt.load=all_nodes[i].demand
    rt.cost=dist_matrix[0][i]
    routes.append(rt)
 return  routes


def Clarke(routes,savingslist):
    for k in savingslist:
        i = k[0]
        j = k[1]
        routei = FindRouteWithNode(i, routes)
        routej = FindRouteWithNode(j, routes)
        Condition2 = FirstOrLast(i, routei) + FirstOrLast(j, routej)
        if Condition2 == 2 and routei != routej:
            BreachCapacity(routei, routej,routes)



a=savingslist()

routes=CreateRoutes()
Clarke(routes,a)

for i in range(0, len(routes)):
            rt = routes[i]
            for j in range (0, len(rt.sequenceOfNodes)):
                print(rt.sequenceOfNodes[j].id, end=' ',)
            print("\n")

print(returnmax(routes))