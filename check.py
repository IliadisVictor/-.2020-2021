import math
import pprint
import random
import copy
import matplotlib.pyplot as plt


class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []

class SolDrawer:
    @staticmethod
    def get_cmap(n, name='hsv'):
        return plt.cm.get_cmap(name, n)

    @staticmethod
    def draw(itr, sol, nodes):
        plt.clf()
        SolDrawer.drawPoints(nodes)
        SolDrawer.drawRoutes(sol)
        plt.savefig(str(itr))

    @staticmethod
    def drawPoints(nodes:list):
        x = []
        y = []
        for i in range(len(nodes)):
            n = nodes[i]
            x.append(n.x)
            y.append(n.y)
        plt.scatter(x, y, c="blue")

    @staticmethod
    def drawRoutes(sol):
        cmap = SolDrawer.get_cmap(len(sol.routes))
        if sol is not None:
            for r in range(0, len(sol.routes)):
                rt = sol.routes[r]
                for i in range(0, len(rt.sequenceOfNodes) - 1):
                    c0 = rt.sequenceOfNodes[i]
                    c1 = rt.sequenceOfNodes[i + 1]
                    plt.plot([c0.x, c1.x], [c0.y, c1.y], c=cmap(r))
class Node:
    def __init__(self, id, tp, dem, xx, yy):
        self.id = id
        self.type = tp
        self.demand = dem
        self.x = xx
        self.y = yy


all_nodes = []
service_locations = []
depot = Node(0, 0, 0, 50, 50)
all_nodes.append(depot)
random.seed(1)
for i in range(0, 200):
    id = i + 1
    tp = random.randint(1, 3)
    dem = random.randint(1, 5) * 100
    xx = random.randint(0, 100)
    yy = random.randint(0, 100)
    serv_node = Node(id, tp, dem, xx, yy)
    all_nodes.append(serv_node)
    service_locations.append(serv_node)

dist_matrix = [[0 for j in range(0, len(all_nodes))] for k in range(0, len(all_nodes))]
for i in range(0, len(all_nodes)):
    for j in range(1, len(all_nodes)):
        source = all_nodes[i]
        target = all_nodes[j]
        dx_2 = (source.x - target.x) ** 2
        dy_2 = (source.y - target.y) ** 2
        dist = round(math.sqrt(dx_2 + dy_2))
        dist_matrix[i][j] = dist
# dict with delivery time between nodes
deliverytime = {}
for i in range(0, len(all_nodes)):
    timetonodes = []
    for j in range(0, len(service_locations)):
        timetonode = 0
        if i != j + 1:
            timetonode = (dist_matrix[i][j + 1]) / 35 + 0.083 + ((service_locations[j].type - 1) * 10) * 0.0167
        timetonodes.append(timetonode)
    deliverytime[all_nodes[i].id] = timetonodes


class Route:
    def __init__(self, dp, cap):
        self.sequenceOfNodes = []
        self.sequenceOfNodes.append(dp)
        self.time = 0
        self.capacity = cap
        self.load = 0


# pprint.pprint(deliverytime)


"""
import sys
def prt_width(myint):
    sys.stdout.write('|' + str(myint) + '|' + ' '*(3 - len(str(myint))) + '  ')
    sys.stdout.flush()

for node in all_nodes:
    prt_width(node.id)
    prt_width(node.x)
    prt_width(node.y)
    prt_width(node.demand)
    prt_width(node.type)
    print('\n')
"""


def sweepMethod():
    anglingNodes = []
    polarsOfAngling = []

    def convertCoordinates():
        # KANE KAI TIS 2 SUNTETAGMENES OLWN TWN SHMEIWN -50
        for i in range(0, len(all_nodes)):
            currentnode = all_nodes[i]
            currentnode.x = currentnode.x - 50
            currentnode.y = currentnode.y - 50
            anglingNodes.append(currentnode)

    def findPolarAngle():
        for i in range(0, len(all_nodes)):
            currentnode = anglingNodes[i]
            x = currentnode.x
            y = currentnode.y
            # tsekarw tetarthmoria
            quadrant = 1
            if x >= 0 and y > 0:
                quadrant = 1
            if x < 0 and y >= 0:
                quadrant = 2
            if x <= 0 and y < 0:
                quadrant = 3
            if x > 0 and y < 0:
                quadrant = 4
            if x == 0:
                polar = 90.0
            else:
                polar = math.degrees(math.atan(y / x))
            if x == 0 and y == 0:
                polar = 0.0
            # diorthwseis vash tetarthmoriwn
            if quadrant == 2 or quadrant == 3:
                polar += 180
            if quadrant == 4:
                polar += 360
            polarangle = (currentnode.id, polar)
            polarsOfAngling.append(polarangle)
        polarsOfAngling.sort(key=lambda tup: tup[1])

    convertCoordinates()
    findPolarAngle()

    def findNodeWithID(id):
        for node in all_nodes:
            if node.id == id:
                return node

    """
    pprint.pprint(polarsOfAngling)
    values=[]
    def findPrefValue():
        for i in range(len(anglingNodes)-1):
            nodeA=polarsOfAngling[i][0]
            nodeB=polarsOfAngling[i+1][0]
            angleA = polarsOfAngling[i][1]
            angleB = polarsOfAngling[i+1][1]
            distanceAB = dist_matrix[nodeA][nodeB]
            depotDistA = dist_matrix[0][nodeA]
            depotDistB = dist_matrix[0][nodeB]
            value = 10000*(angleB-angleA)+10000*(distanceAB+min(depotDistA,depotDistB))
            temp = (nodeA, value)
            values.append(temp)
        values.sort(key=lambda tup: tup[1])

    findPrefValue()
    """
    visited = []
    routes = []
    uvispol = polarsOfAngling.copy()
    pprint.pprint(uvispol)
    uvispol.pop(0)
    timebarrier = 4.66
    yaw = [1]
    while len(visited) < 200:
        cap = 3000
        time = 0
        load = 0
        croute = Route(depot, cap)
        croute.sequenceOfNodes.append(findNodeWithID(uvispol[0][0]))

        visited.append(uvispol[0][0])
        load += service_locations[uvispol[0][0] - 1].demand
        time += deliverytime[0][uvispol[0][0] - 1]
        uvispol.pop(0)
        while load < croute.capacity and time < timebarrier and min(yaw) != 500:
            yaw = dist_matrix[croute.sequenceOfNodes[-1].id].copy()
            yaw.pop(0)
            yaw[croute.sequenceOfNodes[-1].id - 1] = 500
            for i in range(1, 201):
                if i in visited:
                    yaw[i - 1] = 500

            ind = yaw.index(min(yaw))
            if min(yaw) != 500:
                load += service_locations[ind].demand
                time += deliverytime[croute.sequenceOfNodes[-1].id][ind]
                iwannatrymore = True
                tries = 1
            while iwannatrymore is True and tries <= 6 and uvispol and min(yaw) != 500:
                # print(ind + 1, load, time)
                if load <= croute.capacity and time <= timebarrier:
                    # print("firstif")
                    croute.sequenceOfNodes.append(findNodeWithID(ind + 1))
                    visited.append(ind + 1)
                    # print(ind+1)
                    uvispol.remove([tuple(tuple_tbr) for tuple_tbr in uvispol if tuple_tbr[0] == ind + 1][-1])
                    if load == croute.capacity or time == timebarrier:
                        croute.load = load
                        croute.time = time
                        routes.append(croute)
                    iwannatrymore = False
                else:
                    # print("else")
                    # print("firstelse")
                    if load > cap and tries <= 5:
                        # print("sif")
                        load -= service_locations[ind].demand
                        time -= deliverytime[croute.sequenceOfNodes[-1].id][ind]
                        yaw[ind] = 500
                        if min(yaw) != 500:
                            ind = yaw.index(min(yaw))
                            load += service_locations[ind].demand
                            time += deliverytime[croute.sequenceOfNodes[-1].id][ind]
                        tries += 1
                    else:
                        # print("selse")
                        load -= service_locations[ind].demand
                        time -= deliverytime[croute.sequenceOfNodes[-1].id][ind]
                        croute.load = load
                        croute.time = time
                        routes.append(croute)
                        load = croute.capacity
                        ccluster = []
                        iwannatrymore = False
            # print([node.id for node in croute.sequenceOfNodes], ind + 1, load, time)

    croute.load = load
    croute.time = time
    routes.append(croute)

    return routes


routesfromfunction = sweepMethod()


def returnmax(routesll):
    max=0
    for i in range(0, len(routesll)):
        rt = routesll[i]
        if max < rt.time:
            max = rt.time
    return max




dist_matrix = [[0.0 for j in range(0, len(all_nodes))] for k in range(0, len(all_nodes))]
for i in range(0, len(all_nodes)):
    for j in range(0, len(all_nodes)):
        source = all_nodes[i]
        target = all_nodes[j]
        dx_2 = (source.x - target.x)**2
        dy_2 = (source.y - target.y) ** 2
        dist = round(math.sqrt(dx_2 + dy_2))
        dist_matrix[i][j] = dist / 35 + 5/60 + (all_nodes[j].type - 1) * 10/60



def calcroutecost(rt):
    routecost=0
    for j in range(0, len(rt.sequenceOfNodes) - 1):
        k = rt.sequenceOfNodes[j].id
        l = rt.sequenceOfNodes[j + 1].id
        routecost = routecost + dist_matrix[k][l]
    return routecost

def MaxSwapLocalSearch(inputroutes):
        maxroute = 0
        maxvalue = 0
        for i in range(0, len(inputroutes)):
            rt = inputroutes[i]
            if maxvalue < rt.time:
                maxroute = i
                maxvalue = rt.time
        movereducal = 0
        foundreplacement = False
        for secondRouteIndex in range(0, len(inputroutes)):
            rt2: Route = inputroutes[secondRouteIndex]
            if inputroutes[maxroute] == rt2:
                continue
            for firstNodeIndex in range(1, len(inputroutes[maxroute].sequenceOfNodes)):
                for secondNodeIndex in range(1, len(rt2.sequenceOfNodes)):
                    b1 = inputroutes[maxroute].sequenceOfNodes[firstNodeIndex]
                    b2 = rt2.sequenceOfNodes[secondNodeIndex]
                    if inputroutes[maxroute].load - b1.demand + b2.demand > 3000:
                        continue
                    if rt2.load - b2.demand + b1.demand > 3000:
                        continue

                    origincost = 0
                    targetcost = 0

                    a1 = inputroutes[maxroute].sequenceOfNodes[firstNodeIndex - 1]
                    a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
                    if firstNodeIndex != len(inputroutes[maxroute].sequenceOfNodes) - 1:
                        c1 = inputroutes[maxroute].sequenceOfNodes[firstNodeIndex + 1]
                        origincost = origincost - dist_matrix[b1.id][c1.id] + dist_matrix[b2.id][c1.id]
                    origincost = origincost - dist_matrix[a1.id][b1.id] + dist_matrix[a1.id][b2.id]

                    if secondNodeIndex != len(rt2.sequenceOfNodes) - 1:
                        c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
                        targetcost = targetcost + dist_matrix[b1.id][c2.id] - dist_matrix[b2.id][c2.id]
                    targetcost = targetcost + dist_matrix[a2.id][b1.id] - dist_matrix[a2.id][b2.id]
                    if origincost < movereducal and rt2.time + targetcost < inputroutes[maxroute].time:
                        foundreplacement = True
                        keeporiginnode = firstNodeIndex
                        keeptargetnode = secondNodeIndex
                        targetrtindex = secondRouteIndex
                        originrtindex = maxroute
                        keeporigincost = origincost
                        keeptargetcost = targetcost
        if foundreplacement == True:
            rt1 = inputroutes[originrtindex]
            rt2 = inputroutes[targetrtindex]
            b1 = rt1.sequenceOfNodes[keeporiginnode]
            b2 = rt2.sequenceOfNodes[keeptargetnode]
            rt1.sequenceOfNodes[keeporiginnode] = b2
            rt2.sequenceOfNodes[keeptargetnode] = b1
            rt1.time += keeporigincost
            rt2.time += keeptargetcost
            rt1.load = rt1.load - b1.demand + b2.demand
            rt2.load = rt2.load + b1.demand - b2.demand
            # fixtaboolist(rt1,rt2,b1,b2,"maxswap")


def relocatefrommax(relroutes):
    maxroute = 0
    maxvalue = 0
    for i in range(0, len(relroutes)):
        rt = relroutes[i]
        if maxvalue < rt.time:
            maxroute = i
            maxvalue = rt.time
    movereducal = 0
    foundreplacement = False
    for targetRouteIndex in range(0, len(relroutes)):
        rt2 = relroutes[targetRouteIndex]
        for originNodeIndex in range(1, len(relroutes[maxroute].sequenceOfNodes)):
            if relroutes[maxroute].sequenceOfNodes[originNodeIndex].demand + rt2.load > 3000:
                continue
            for targetNodeIndex in range(0, len(rt2.sequenceOfNodes)):

                A = relroutes[maxroute].sequenceOfNodes[originNodeIndex - 1]
                B = relroutes[maxroute].sequenceOfNodes[originNodeIndex]
                origincost = -dist_matrix[A.id][B.id]
                if originNodeIndex != len(relroutes[maxroute].sequenceOfNodes) - 1:
                    C = relroutes[maxroute].sequenceOfNodes[originNodeIndex + 1]
                    origincost = -dist_matrix[A.id][B.id] - dist_matrix[B.id][C.id] + dist_matrix[A.id][C.id]
                F = rt2.sequenceOfNodes[targetNodeIndex]
                targetcost = dist_matrix[F.id][B.id]
                if targetNodeIndex != len(rt2.sequenceOfNodes) - 1:
                    G = rt2.sequenceOfNodes[targetNodeIndex + 1]
                    targetcost = dist_matrix[F.id][B.id] + dist_matrix[B.id][G.id] - dist_matrix[F.id][G.id]

                if origincost < movereducal and rt2.time + targetcost < relroutes[maxroute].time:
                    movereducal=origincost
                    foundreplacement = True
                    keeporiginnode = originNodeIndex
                    keeptargetnode = targetNodeIndex
                    targetrtindex = targetRouteIndex
                    keeporigincost = origincost
                    keeptargetcost = targetcost
    if foundreplacement == True:
        originroute = relroutes[maxroute]
        targetroute = relroutes[targetrtindex]
        originroute.time = originroute.time + keeporigincost
        targetroute.time= targetroute.time + keeptargetcost
        B = originroute.sequenceOfNodes[keeporiginnode]
        originroute.load -= B.demand
        targetroute.load += B.demand
        del originroute.sequenceOfNodes[keeporiginnode]
        targetroute.sequenceOfNodes.insert(keeptargetnode + 1, B)


def OriginalRelocation(relroutes):
    movecost = 0
    foundreplacement = False
    for originRouteIndex in range(0, len(relroutes)):
        rt1 = relroutes[originRouteIndex]
        for targetRouteIndex in range(0, len(relroutes)):
            rt2 = relroutes[targetRouteIndex]
            for originNodeIndex in range(1, len(rt1.sequenceOfNodes)):
                if rt1.sequenceOfNodes[originNodeIndex].demand + rt2.load > 3000:
                    continue
                for targetNodeIndex in range(0, len(rt2.sequenceOfNodes)):
                    if originRouteIndex == targetRouteIndex and (
                            targetNodeIndex == originNodeIndex or targetNodeIndex == originNodeIndex - 1):
                        continue
                    if Istaboo(originRouteIndex,targetRouteIndex,originNodeIndex,targetNodeIndex,"rlorigin"):
                        continue

                    A = rt1.sequenceOfNodes[originNodeIndex - 1]
                    B = rt1.sequenceOfNodes[originNodeIndex]
                    origincost = -dist_matrix[A.id][B.id]
                    costAdded=0
                    costRemoved=dist_matrix[A.id][B.id]
                    if originNodeIndex != len(rt1.sequenceOfNodes) - 1:
                        C = rt1.sequenceOfNodes[originNodeIndex + 1]
                        origincost = -dist_matrix[A.id][B.id] - dist_matrix[B.id][C.id] + dist_matrix[A.id][C.id]
                        costAdded=dist_matrix[A.id][C.id]+costAdded
                        costRemoved=costRemoved+dist_matrix[B.id][C.id]
                    F = rt2.sequenceOfNodes[targetNodeIndex]
                    targetcost = dist_matrix[F.id][B.id]
                    costAdded=dist_matrix[F.id][B.id]+costAdded
                    if targetNodeIndex != len(rt2.sequenceOfNodes) - 1:
                        G = rt2.sequenceOfNodes[targetNodeIndex + 1]
                        costRemoved = costRemoved + dist_matrix[F.id][G.id]
                        targetcost = dist_matrix[F.id][B.id] + dist_matrix[B.id][G.id] - dist_matrix[F.id][G.id]
                        costAdded = dist_matrix[B.id][G.id] + costAdded

                    if movecost > costAdded - costRemoved:

                        movecost =costAdded - costRemoved
                        foundreplacement = True
                        keeporiginnode = originNodeIndex
                        keeptargetnode = targetNodeIndex
                        originrtindex = originRouteIndex
                        targetrtindex = targetRouteIndex
                        keeporigincost = origincost
                        keeptargetcost = targetcost
    if foundreplacement == True:
        fixtaboolist(originrtindex,targetrtindex,keeporiginnode,keeptargetnode,"rlorigin")
        originRt = relroutes[originrtindex]
        targetRt = relroutes[targetrtindex]
        B = originRt.sequenceOfNodes[keeporiginnode]

        if originRt == targetRt:
            del originRt.sequenceOfNodes[  keeporiginnode]
            if (  keeporiginnode <   keeptargetnode):
                targetRt.sequenceOfNodes.insert(  keeptargetnode, B)
            else:
                targetRt.sequenceOfNodes.insert(  keeptargetnode + 1, B)

            originRt.time += movecost
        else:
            del originRt.sequenceOfNodes[  keeporiginnode]
            targetRt.sequenceOfNodes.insert(  keeptargetnode + 1, B)
            originRt.time +=   keeporigincost
            targetRt.time +=  keeptargetcost
            originRt.load -= B.demand
            targetRt.load += B.demand




def Opt2move(optroutes):
    # while True:
        movereducal=0
        foundreplacement=False
        for originRouteIndex in range(0,len(optroutes)):
            rt1=Route = optroutes[originRouteIndex]
            for targetRouteIndex in range(0, len(optroutes)):
                rt2: Route = optroutes[targetRouteIndex]
                for originNodeIndex in range(0, len(rt1.sequenceOfNodes) - 1):
                    start2 = 0
                    if (rt1 == rt2):
                        start2 = originNodeIndex + 2

                    for targetNodeIndex in range(start2, len(rt2.sequenceOfNodes) - 1):
                        if Istaboo(originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, "opt"):
                            continue
                        A = rt1.sequenceOfNodes[originNodeIndex]
                        B = rt1.sequenceOfNodes[originNodeIndex + 1]
                        K = rt2.sequenceOfNodes[targetNodeIndex]
                        L = rt2.sequenceOfNodes[targetNodeIndex + 1]

                        if rt1 == rt2:
                            if originNodeIndex == 0 and targetNodeIndex  == len(rt1.sequenceOfNodes) - 2:
                                continue
                            tryroyte=TwoOptroutesequal(optroutes,originRouteIndex,originNodeIndex,targetNodeIndex)
                            moveCost=tryroyte.time-rt1.time
                        else:
                            if originNodeIndex == 0 and targetNodeIndex == 0:
                                continue
                            if originNodeIndex == len(rt1.sequenceOfNodes) - 2 and targetNodeIndex== len(rt2.sequenceOfNodes) - 2:
                                continue

                            if CapacityIsViolated(rt1, originNodeIndex, rt2, targetNodeIndex):
                                continue
                            costAdded = dist_matrix[A.id][L.id] + dist_matrix[K.id][B.id]
                            costRemoved = dist_matrix[A.id][B.id] + dist_matrix[K.id][L.id]
                            moveCost = costAdded - costRemoved
                        if moveCost < movereducal and abs(moveCost) > 0.0001:
                            movereducal=moveCost
                            foundreplacement=True
                            keeprt1=originRouteIndex
                            keeprt2=targetRouteIndex
                            keepnode1=originNodeIndex
                            keepnode2=targetNodeIndex
        if foundreplacement==True:
                # print(movereducal,keeprt1,keeprt2,routes[keeprt1].sequenceOfNodes[keepnode1].id,routes[keeprt2].sequenceOfNodes[keepnode2].id)
                Apply2opt(keeprt1,keeprt2,keepnode1,keepnode2,optroutes,movereducal)
        #         foundreplacement=False
                fixtaboolist(keeprt1, keeprt2, keepnode1, keepnode2, "opt")


        # else:
        #     break



def opt2movemax(optroutes):
    # while True:
            maxroute = 0
            maxvalue = 0
            for i in range(0, len(optroutes)):
                rt = optroutes[i]
                if maxvalue < rt.time:
                    maxroute = i
                    maxvalue = rt.time
            movereducal = 0
            foundreplacement=False
            rt1=optroutes[maxroute]
            for targetRouteIndex in range(0, len(optroutes)):
                rt2: Route = optroutes[targetRouteIndex]
                for originNodeIndex in range(0, len(rt1.sequenceOfNodes) - 1):
                    start2 = 0
                    if (rt1 == rt2):
                        start2 = originNodeIndex + 2

                    for targetNodeIndex in range(start2, len(rt2.sequenceOfNodes) - 1):
                        moveCost=1000
                        A = rt1.sequenceOfNodes[originNodeIndex]
                        B = rt1.sequenceOfNodes[originNodeIndex + 1]
                        K = rt2.sequenceOfNodes[targetNodeIndex]
                        L = rt2.sequenceOfNodes[targetNodeIndex + 1]
                        if rt1 == rt2:
                            if originNodeIndex == 0 and targetNodeIndex == len(rt1.sequenceOfNodes) - 2:
                                continue
                            tryroyte = TwoOptroutesequal(optroutes, maxroute, originNodeIndex, targetNodeIndex)
                            moveCost = tryroyte.time - rt1.time
                        else:
                            if originNodeIndex == 0 and targetNodeIndex == 0:
                                continue
                            if originNodeIndex == len(rt1.sequenceOfNodes) - 2 and targetNodeIndex == len(
                                    rt2.sequenceOfNodes) - 2:
                                continue

                            if CapacityIsViolated(rt1, originNodeIndex, rt2, targetNodeIndex):
                                continue
                            tryrout1= TwoOptroutenotqeual(optroutes,maxroute,targetRouteIndex,originNodeIndex,targetNodeIndex,1)
                            tryrout2 = TwoOptroutenotqeual(optroutes, maxroute, targetRouteIndex, originNodeIndex,targetNodeIndex, 2)
                            if tryrout2.time < rt1.time and tryrout1.time <rt1.time:
                                moveCost=tryrout1.time-rt1.time


                        if moveCost < movereducal and abs(moveCost) > 0.0001:
                            movereducal = moveCost
                            foundreplacement = True
                            keeprt1 = maxroute
                            keeprt2 = targetRouteIndex
                            keepnode1 = originNodeIndex
                            keepnode2 = targetNodeIndex
            if foundreplacement == True:
                print(movereducal, keeprt1, keeprt2, routes[keeprt1].sequenceOfNodes[keepnode1].id,
                      routes[keeprt2].sequenceOfNodes[keepnode2].id)
                Apply2opt(keeprt1, keeprt2, keepnode1, keepnode2, optroutes, movereducal)
                # fixtaboolist(keeprt1, keeprt2, keepnode1, keepnode2, "optmax")
            #     foundreplacement = False
            # else:
            #         break

def TwoOptroutenotqeual(rts, rt1index, rt2index, node1index, node2index, choose):
        rt1k: Route = rts[rt1index]
        rt2k: Route = rts[rt2index]
        if choose == 1:
            routetoreturn = copy.deepcopy(rt1k)
            relocatedSegmentOfRt2 = rt2k.sequenceOfNodes[node2index + 1:]

            del routetoreturn.sequenceOfNodes[node1index + 1:]
            routetoreturn.sequenceOfNodes.extend(relocatedSegmentOfRt2)

            routetoreturn.time = CalcCost(routetoreturn)
            routetoreturn.load = CalcCapacity(routetoreturn)

            return routetoreturn


        else:
            routetoreturn = copy.deepcopy(rt2k)
            relocatedSegmentOfRt1 = rt1k.sequenceOfNodes[node1index + 1:]

            del routetoreturn.sequenceOfNodes[node2index + 1:]
            routetoreturn.sequenceOfNodes.extend(relocatedSegmentOfRt1)

            routetoreturn.time = CalcCost(routetoreturn)
            routetoreturn.load = CalcCapacity(routetoreturn)

            return routetoreturn




def Apply2opt(rt1ind,rt2ind,node1,node2,rts,mc):
    rt1k: Route = rts[rt1ind]
    rt2k: Route = rts[rt2ind]

    if rt1k == rt2k:
        # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
        reversedSegment = reversed(rt1k.sequenceOfNodes[node1+ 1: node2 + 1])
        # lst = list(reversedSegment)
        # lst2 = list(reversedSegment)
        rt1k.sequenceOfNodes[node1 + 1: node2 + 1] = reversedSegment

        # reversedSegmentList = list(reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1]))
        # rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegmentList

        rt1k.time +=mc
    else:
        # slice with the nodes from position top.positionOfFirstNode + 1 onwards
        relocatedSegmentOfRt1 = rt1k.sequenceOfNodes[node1 + 1:]

        # slice with the nodes from position top.positionOfFirstNode + 1 onwards
        relocatedSegmentOfRt2 = rt2k.sequenceOfNodes[node2 + 1:]

        del rt1k.sequenceOfNodes[node1 + 1:]
        del rt2k.sequenceOfNodes[node2 + 1:]

        rt1k.sequenceOfNodes.extend(relocatedSegmentOfRt2)
        rt2k.sequenceOfNodes.extend(relocatedSegmentOfRt1)

        rt1k.time = CalcCost(rt1k)
        rt1k.load = CalcCapacity(rt1k)
        rt2k.time = CalcCost(rt2k)
        rt2k.load = CalcCapacity(rt2k)



def TwoOptroutesequal(rts,rt1index,node1index,node2index):
    rt1k: Route = rts[rt1index]
    routetoreturn=copy.deepcopy(rt1k)
    reversedSegment = reversed(rt1k.sequenceOfNodes[node1index + 1: node2index + 1])
    routetoreturn.sequenceOfNodes[node1index + 1: node2index + 1] = reversedSegment
    routetoreturn.time=CalcCost(routetoreturn)
    return routetoreturn



def CapacityIsViolated(rt1l, nodeInd1l, rt2l, nodeInd2l):
    rt1FirstSegmentLoad = 0
    for i in range(0, nodeInd1l + 1):
        n = rt1l.sequenceOfNodes[i]
        rt1FirstSegmentLoad += n.demand
    rt1SecondSegmentLoad = rt1l.load - rt1FirstSegmentLoad

    rt2FirstSegmentLoad = 0
    for i in range(0, nodeInd2l+ 1):
        n = rt2l.sequenceOfNodes[i]
        rt2FirstSegmentLoad += n.demand
    rt2SecondSegmentLoad = rt2l.load - rt2FirstSegmentLoad

    if (rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1l.capacity):
        return True
    if (rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2l.capacity):
        return True
    return False




def CalcCost(rt1k):
    routecost=0
    for j in range(0, len(rt1k.sequenceOfNodes) - 1):
        k = rt1k.sequenceOfNodes[j].id
        l = rt1k.sequenceOfNodes[j + 1].id
        routecost = routecost + dist_matrix[k][l]
    return routecost

def CalcCapacity(routec):
    routeload=0
    for j in range(0, len(routec.sequenceOfNodes)):
        routeload = routeload + all_nodes[routec.sequenceOfNodes[j].id].demand
    return routeload








def fixtaboolist(banrt1,banrt2,bannd1,bannd2,movetype):
    # print(banrt1,banrt2,bannd1,bannd2,movetype)
    taboolist.append([banrt1,banrt2,bannd1,bannd2,movetype])
    if len(taboolist) >15:
        taboolist.pop(0)



def Istaboo(rout1,rout2,nnd1,nnd2,movetype):
    for i in taboolist:
        # print(i)
        # print(i)
        if i[0]==rout1 and i[1]==rout2 and i[2]==nnd1 and i[3]==nnd2 and movetype==i[4]:
            return True
    return False


















routes=routesfromfunction
totalnodes=0
max=10
taboolist=[]
number=0
for i in range(0,300):
    # print(i,"Ekane epanalhpsh")
    OriginalRelocation(routes)
    Opt2move(routes)
    relocatefrommax(routes)
    opt2movemax(routes)
    MaxSwapLocalSearch(routes)
    # relocatefrommax(routes)
    # MaxSwapLocalSearch(routes)
    # relocatefrommax(routes)

    number+=1
    # print("Times without improving",number)
    # print(returnmax(routes))
    if returnmax(routes) <max:
        print(returnmax(routes),"Brhkeeeee")
        number=0
        routesmax=copy.deepcopy(routes)
        max=returnmax(routes)

#
# for i in range (0,30):
#     opt2movemax(routes)
#     MaxSwapLocalSearch(routes)
#     relocatefrommax(routes)



totalcost=0
print(taboolist)

#
# for i in range (0,150):
#     relocatefrommax(routesmax)
#     MaxSwapLocalSearch(routesmax)










for i in range(0, len(routesmax)):
            rt = routesmax[i]
            weight=0
            cost=0
            for j in range (0, len(rt.sequenceOfNodes)):
                id1=rt.sequenceOfNodes[j].id
                weight=weight+all_nodes[id1].demand
                print(id1, end=' ',)
            for j in range (0, len(rt.sequenceOfNodes)-1):
                id1=rt.sequenceOfNodes[j].id
                id2 = rt.sequenceOfNodes[j+1].id
                cost=cost+dist_matrix[id1][id2]
            totalcost+=cost
            print(cost,rt.time)

print(returnmax(routesmax))




sol = Solution()
sol.routes=routes

drawer=SolDrawer()
drawer.draw(5,sol,all_nodes)






