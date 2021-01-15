import copy
import math
import pprint
import random
from Solution_Drawer import SolDrawer
from tests import testsol

class Node:
 def __init__(self, id, tp, dem, xx, yy):
  self.id = id
  self.type = tp
  self.demand = dem
  self.x = xx
  self.y = yy

class Model:
    def __init__(self):
        self.all_nodes = []
        self.service_locations = []
        self.dist_matrix = []
        self.distMatrix = []
        self.deliverytime = {}
        self.capacity = -1


    def BuildModel(self):

        depot = Node(0, 0, 0, 50, 50)
        self.all_nodes.append(depot)
        random.seed(1)
        self.capacity = 3000
        for i in range(0, 200):
             id = i + 1
             tp = random.randint(1,3)
             dem = random.randint(1,5) * 100
             xx = random.randint(0, 100)
             yy = random.randint(0, 100)
             serv_node = Node(id, tp, dem, xx, yy)
             self.all_nodes.append(serv_node)
             self.service_locations.append(serv_node)

        self.dist_matrix = [[0 for j in range(0, len(self.all_nodes))] for k in range(0, len(self.all_nodes))]
        for i in range(0, len(self.all_nodes)):
            for j in range(1, len(self.all_nodes)):
              source = self.all_nodes[i]
              target = self.all_nodes[j]
              dx_2 = (source.x - target.x)**2
              dy_2 = (source.y - target.y) ** 2
              dist = round(math.sqrt(dx_2 + dy_2))
              self.dist_matrix[i][j] = dist
        #dict with delivery time between nodes
        for i in range(0, len(self.all_nodes)):
            timetonodes = []
            for j in range(0, len(self.service_locations)):
                timetonode = 0
                if i != j+1:
                    timetonode = (self.dist_matrix[i][j+1])/35 + 0.083 + ((self.service_locations[j].type - 1)*10)*0.0167
                timetonodes.append(timetonode)
            self.deliverytime[self.all_nodes[i].id] = timetonodes

        self.distMatrix = [[0.0 for j in range(0, len(self.all_nodes))] for k in range(0, len(self.all_nodes))]
        for i in range(0, len(self.all_nodes)):
            for j in range(0, len(self.all_nodes)):
                source = self.all_nodes[i]
                target = self.all_nodes[j]
                dx_2 = (source.x - target.x) ** 2
                dy_2 = (source.y - target.y) ** 2
                dist = round(math.sqrt(dx_2 + dy_2))
                self.distMatrix[i][j] = dist / 35 + 5 / 60 + (self.all_nodes[j].type - 1) * 10 / 60


class Route:
    def __init__(self, dp, cap):
        self.sequenceOfNodes = []
        self.sequenceOfNodes.append(dp)
        self.time = 0
        self.capacity = cap
        self.load = 0

#pprint.pprint(deliverytime)



class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []

class Solver:

    def __init__(self, m):
        self.all_nodes = m.all_nodes
        self.service_locations = m.service_locations
        self.deliverytime = m.deliverytime
        self.depot = m.all_nodes[0]
        self.dist_matrix = m.dist_matrix
        self.capacity = m.capacity
        self.sol = None
        self.distanceMatrix = m.distMatrix






    def sweepMethod(self, method):
        anglingNodes = []
        polarsOfAngling = []
        def convertCoordinates():
            # KANE KAI TIS 2 SUNTETAGMENES OLWN TWN SHMEIWN -50
            for i in range(0, len(self.all_nodes)):
                currentnode = self.all_nodes[i]
                currentnode.x = currentnode.x-50
                currentnode.y = currentnode.y-50
                anglingNodes.append(currentnode)

        def findPolarAngle():
            for i in range(0, len(self.all_nodes)):
                currentnode = anglingNodes[i]
                x=currentnode.x
                y=currentnode.y
                #tsekarw tetarthmoria
                quadrant=1
                if x>=0 and y>0:
                    quadrant = 1
                if x<0 and y>=0:
                    quadrant=2
                if x<=0 and y<0:
                    quadrant=3
                if x>0 and y<0:
                    quadrant=4
                if x==0:
                    polar = 90.0
                else:
                    polar = math.degrees(math.atan(y/x))
                if x == 0 and y == 0:
                    polar = 0.0
                #diorthwseis vash tetarthmoriwn
                if quadrant==2 or quadrant==3:
                    polar+=180
                if quadrant==4:
                    polar+=360
                polarangle = (currentnode.id,polar)
                polarsOfAngling.append(polarangle)
            polarsOfAngling.sort(key=lambda tup: tup[1])



        convertCoordinates()
        findPolarAngle()

        """
        def preferenceValues(id):
            for node in polarsOfAngling:
                if math.fabs(polarsOfAngling[node][1] - polarsOfAngling[id][1]):
                        
        """

        def findNodeWithID(id):
            for node in self.all_nodes:
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
        yaw = [0]
        visited = []
        routes = []
        uvispol = polarsOfAngling.copy()
        uvispol.pop(0)
        while len(visited) < 200:
            cap = self.capacity
            time = 0
            load = 0
            croute = Route(self.depot, cap)
            croute.sequenceOfNodes.append(findNodeWithID(uvispol[0][0]))

            visited.append(uvispol[0][0])
            load += self.service_locations[uvispol[0][0]-1].demand
            time += self.deliverytime[0][uvispol[0][0]-1]
            uvispol.pop(0)
            if method == "nearest":
                timebarrier = 4.66
                while load < croute.capacity and time < timebarrier and min(yaw) != 500:
                    yaw = self.dist_matrix[croute.sequenceOfNodes[-1].id].copy()
                    yaw.pop(0)
                    yaw[croute.sequenceOfNodes[-1].id - 1] = 500
                    for i in range(1, 201):
                        if i in visited:
                            yaw[i - 1] = 500

                    ind = yaw.index(min(yaw))
                    if min(yaw) != 500:
                        load += self.service_locations[ind].demand
                        time += self.deliverytime[croute.sequenceOfNodes[-1].id][ind]
                        iwannatrymore = True
                        tries = 1
                    while iwannatrymore is True and tries <= 6 and uvispol and min(yaw) != 500:
                        #print(ind+1, load, time)
                        if load <= croute.capacity and time <= timebarrier:
                            #print("firstif")
                            croute.sequenceOfNodes.append(findNodeWithID(ind+1))
                            visited.append(ind+1)
                            #print(ind+1)
                            uvispol.remove([tuple(tuple_tbr) for tuple_tbr in uvispol if tuple_tbr[0] == ind+1][-1])
                            if load == croute.capacity or time == timebarrier:
                                croute.load = load
                                croute.time = time
                                routes.append(croute)
                            iwannatrymore = False
                        else:
                            #print("else")
                            #print("firstelse")
                            if load > cap and tries <= 5:
                                #print("sif")
                                load -= self.service_locations[ind].demand
                                time -= self.deliverytime[croute.sequenceOfNodes[-1].id][ind]
                                yaw[ind] = 500
                                if min(yaw) != 500:
                                    ind = yaw.index(min(yaw))
                                    load += self.service_locations[ind].demand
                                    time += self.deliverytime[croute.sequenceOfNodes[-1].id][ind]
                                tries += 1
                            else:
                                #print("selse")
                                load -= self.service_locations[ind].demand
                                time -= self.deliverytime[croute.sequenceOfNodes[-1].id][ind]
                                croute.load = load
                                croute.time = time
                                routes.append(croute)
                                load = croute.capacity
                                iwannatrymore = False
                    #print([node.id for node in croute.sequenceOfNodes], ind+1, load, time)
            elif method == "polar":
                timebarrier = 7
                while len(visited) < 200:
                    croute.sequenceOfNodes.append(findNodeWithID(uvispol[0][0]))
                    visited.append(uvispol[0][0])
                    load += self.service_locations[uvispol[0][0] - 1].demand
                    time = self.calculateRouteTime(croute)
                    uvispol.pop(0)
                    if uvispol:
                        next_time = time + self.deliverytime[croute.sequenceOfNodes[-1].id][uvispol[0][0] - 1]
                        #if len(routes) == 11:
                            #print(time, next_time)
                        next_load = load + self.service_locations[uvispol[0][0] - 1].demand
                    if next_load > croute.capacity or next_time > timebarrier:
                            croute.load = load
                            croute.time = time
                            routes.append(croute)
                            break
            """
            else:
                timebarrier = 10
                while len(visited) < 200:
                    croute.sequenceOfNodes.append(findNodeWithID(self.dist_matrix.index(min(self.dist_matrix[0]))[1]))
                    visited.append(uvispol[0][0])
                    load += self.service_locations[uvispol[0][0] - 1].demand
                    time = self.calculateRouteTime(croute)
                    uvispol.pop(0)
                    if uvispol:
                        next_time = time + self.deliverytime[croute.sequenceOfNodes[-1].id][uvispol[0][0] - 1]
                        #if len(routes) == 11:
                            #print(time, next_time)
                        next_load = load + self.service_locations[uvispol[0][0] - 1].demand
                    if next_load > croute.capacity or next_time > timebarrier:
                            croute.load = load
                            croute.time = time
                            routes.append(croute)
                            break
            """


        croute.load = load
        croute.time = time
        routes.append(croute)
        return routes




    def calculateRouteTime(self, route):
        caltime = 0
        nodid=[]
        span = len(route.sequenceOfNodes)
        for i in range(span-1):
            thisid = route.sequenceOfNodes[i].id
            nextid = route.sequenceOfNodes[i + 1].id
            caltime += self.deliverytime[thisid][nextid - 1]
            #print(self.deliverytime[thisid][nextid - 1])
        return caltime

    def twoOptSwap(self, route, i, k):
        routepiecetoreverse = []
        routepiecereversed = []
        for j in range(i, k):
            routepiecetoreverse.append(route.sequenceOfNodes[j])
        for node in reversed(routepiecetoreverse):
            routepiecereversed.append(node)
        ri = 0
        for j in range(i, k):
            route.sequenceOfNodes[j] = copy.deepcopy(routepiecereversed[ri])
            ri += 1
        return route

    def createRTs(self, rt1, rt2, ind1, ind2, routetoreturn):
        routetobertrnd = Route(self.depot, 3000)
        if (routetoreturn == 1):
            for i in range(0, ind1):
                node = rt1.sequenceOfNodes[i+1]
                routetobertrnd.sequenceOfNodes.append(node)
            for j in range(ind2+1, len(rt2.sequenceOfNodes)):
                node = rt2.sequenceOfNodes[j]
                routetobertrnd.sequenceOfNodes.append(node)
        if (routetoreturn == 2):
            for i in range(0, ind2):
                node = rt2.sequenceOfNodes[i + 1]
                routetobertrnd.sequenceOfNodes.append(node)
            for j in range(ind1 + 1, len(rt1.sequenceOfNodes)):
                node = rt1.sequenceOfNodes[j]
                routetobertrnd.sequenceOfNodes.append(node)
        return routetobertrnd


    def calculateLoad(self, route):
        load = 0
        for node in route.sequenceOfNodes:
            load += node.demand
        return load


    def twoOptMove(self, routes):
        minmovecost=0
        for rt1ind in range(0, len(routes)):
            rt1 = routes[rt1ind]
            for rt2ind in range(rt1ind, len(routes)):
                rt2 = routes[rt2ind]
                for node1ind in range(1, len(rt1.sequenceOfNodes)- 1):
                    start2 = 0
                    # if (rt1 == rt2):
                    #     start2 = node1ind + 2
                    for node2ind in range(start2, len(rt2.sequenceOfNodes)-1):
                        moveCost = 200402030400
                        A = rt1.sequenceOfNodes[node1ind]
                        B = rt1.sequenceOfNodes[node1ind + 1]
                        K = rt2.sequenceOfNodes[node2ind]
                        L = rt2.sequenceOfNodes[node2ind + 1]

                        if (rt1 == rt2):
                            if node1ind == 0 and node2ind == len(rt1.sequenceOfNodes) - 2:
                                continue
                            costAdded = self.distanceMatrix[A.id][K.id] + self.distanceMatrix[B.id][L.id]
                            costRemoved = self.distanceMatrix[A.id][B.id] + self.distanceMatrix[K.id][L.id]
                            moveCost = costAdded - costRemoved

                        if (rt1 !=rt2):
                            if node1ind==0 and node2ind==0:
                                continue
                            if node1ind==len(rt1.sequenceOfNodes)-2 and node2ind==len(rt2.sequenceOfNodes)-2:
                                continue
                            newrt1 = self.createRTs(rt1, rt2, node1ind,node2ind,1)
                            newrt2 = self.createRTs(rt1, rt2, node1ind,node2ind,2)
                            newrt1.load = self.calculateLoad(newrt1)
                            newrt2.load = self.calculateLoad(newrt2)
                            if (newrt1.load <= 3000 and newrt2.load <= 3000):
                                costAdded = self.distanceMatrix[A.id][L.id] + self.distanceMatrix[K.id][B.id]
                                costRemoved = self.distanceMatrix[A.id][B.id] + self.distanceMatrix[K.id][L.id]
                                moveCost = costAdded - costRemoved

                        if (moveCost < 0 and abs(moveCost) > 0.0001):

                            if rt1 != rt2 :
                                newrt1.time = self.calculateRouteTime(newrt1)
                                newrt2.time = self.calculateRouteTime(newrt2)
                                routes[rt1ind]=newrt1
                                routes[rt2ind]=newrt2

                            # if rt1 ==rt2 :
                            #     rt1.time += moveCost
                            #     routes[rt1ind] = self.twoOptSwap(rt1, node1ind, node2ind)









    def calcroutecost(self, rt):
        routecost = 0
        for j in range(0, len(rt.sequenceOfNodes) - 1):
            k = rt.sequenceOfNodes[j].id
            l = rt.sequenceOfNodes[j + 1].id
            routecost = routecost + self.deliverytime[k][l-1]
        return routecost

    def swapLocalSearch(self, inputroutes):
        for i in range(0, 10):
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
                            origincost = origincost - self.deliverytime[b1.id][c1.id - 1] + self.deliverytime[b2.id][c1.id - 1]
                        origincost = origincost - self.deliverytime[a1.id][b1.id - 1] + self.deliverytime[a1.id][b2.id - 1]

                        if secondNodeIndex != len(rt2.sequenceOfNodes) - 1:
                            c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
                            targetcost = targetcost + self.deliverytime[b1.id][c2.id - 1] - self.deliverytime[b2.id][c2.id - 1]
                        targetcost = targetcost + self.deliverytime[a2.id][b1.id - 1] - self.deliverytime[a2.id][b2.id - 1]
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
        return inputroutes


    def relocateFromMax(self, relroutes):
        for i in range(0, 1):
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
                if rt2 == relroutes[maxroute]:
                    continue
                for originNodeIndex in range(1, len(relroutes[maxroute].sequenceOfNodes)):
                    if relroutes[maxroute].sequenceOfNodes[originNodeIndex].demand + rt2.load > 3000:
                        continue
                    for targetNodeIndex in range(0, len(rt2.sequenceOfNodes)):
                        A = relroutes[maxroute].sequenceOfNodes[originNodeIndex - 1]
                        B = relroutes[maxroute].sequenceOfNodes[originNodeIndex]
                        origincost = - self.distanceMatrix[A.id][B.id]
                        if originNodeIndex != len(relroutes[maxroute].sequenceOfNodes) - 1:
                            C = relroutes[maxroute].sequenceOfNodes[originNodeIndex + 1]
                            origincost = - self.distanceMatrix[A.id][B.id] - self.distanceMatrix[B.id][C.id] + self.distanceMatrix[A.id][C.id]
                        F = rt2.sequenceOfNodes[targetNodeIndex]
                        targetcost = self.distanceMatrix[F.id][B.id]
                        if targetNodeIndex != len(rt2.sequenceOfNodes) - 1:
                            G = rt2.sequenceOfNodes[targetNodeIndex + 1]
                            targetcost = self.distanceMatrix[F.id][B.id] + self.distanceMatrix[B.id][G.id] - self.distanceMatrix[F.id][G.id]
                        if origincost < movereducal and rt2.time + targetcost < relroutes[maxroute].time:
                            foundreplacement = not foundreplacement
                            keeporiginnode = originNodeIndex
                            keeptargetnode = targetNodeIndex
                            targetrtindex = targetRouteIndex
                            keeporigincost = origincost
                            keeptargetcost = targetcost
            if foundreplacement == True:
                originroute = relroutes[maxroute]
                targetroute = relroutes[targetrtindex]
                originroute.time = originroute.time + keeporigincost
                targetroute.time = targetroute.time + keeptargetcost
                B = originroute.sequenceOfNodes[keeporiginnode]
                originroute.load -= B.demand
                targetroute.load += B.demand
                del originroute.sequenceOfNodes[keeporiginnode]
                targetroute.sequenceOfNodes.insert(keeptargetnode + 1, B)
        return relroutes

    def swapMoveAllChanges(self, route_of_max_cost, sol):

        rt1: Route = route_of_max_cost
        for second_route in range(0, len(sol.routes)):
            rt2: Route = sol.routes[second_route]
            for first_route_node in range(1, len(rt1.sequenceOfNodes)):
                fn = rt1.sequenceOfNodes[first_route_node]
                startOfSecondNodeIndex = 1
                if rt1 == rt2:
                    if first_route_node != len(rt1.sequenceOfNodes) - 1:
                        startOfSecondNodeIndex = first_route_node + 1
                    else:
                        break
                for second_route_node in range(startOfSecondNodeIndex, len(rt2.sequenceOfNodes)):
                    sn = rt2.sequenceOfNodes[second_route_node]
                    A = rt1.sequenceOfNodes[first_route_node - 1]
                    if first_route_node == len(rt1.sequenceOfNodes) - 1:
                        B = fn
                    else:
                        B = rt1.sequenceOfNodes[first_route_node + 1]
                    C = rt2.sequenceOfNodes[second_route_node - 1]
                    if second_route_node == len(rt2.sequenceOfNodes) - 1:
                        D = sn
                    else:
                        D = rt2.sequenceOfNodes[second_route_node + 1]
                    rt1_removed_demand = fn.demand
                    rt1_added_demand = sn.demand
                    rt2_removed_demand = sn.demand
                    rt2_added_demand = fn.demand
                    if rt1 == rt2:
                        rt1_removed_demand = fn.demand + sn.demand
                        rt1_added_demand = fn.demand + sn.demand
                        rt2_removed_demand = fn.demand + sn.demand
                        rt2_added_demand = fn.demand + sn.demand
                    rt1_capacity = rt1.capacity - rt1_removed_demand + rt1_added_demand
                    rt2_capacity = rt2.capacity - rt2_removed_demand + rt2_added_demand
                    rt1_load = rt1.load - rt1_removed_demand + rt1_added_demand
                    rt2_load = rt2.load - rt2_removed_demand + rt2_added_demand
                    if rt1_capacity + rt1_load == 3000 and rt2_capacity + rt2_load == 3000:
                        rt1_cost_before = rt1.time
                        rt_cost_after = rt1_cost_before
                        if rt1 == rt2:
                            rt_cost_before = rt1.time
                            if first_route_node < second_route_node and B.id == sn.id and C.id == fn.id:
                                rt_removed_cost = self.distanceMatrix[A.id][fn.id] + self.distanceMatrix[sn.id][
                                    D.id] + self.distanceMatrix[fn.id][sn.id]
                                rt_added_cost = self.distanceMatrix[A.id][sn.id] + self.distanceMatrix[fn.id][
                                    D.id] + self.distanceMatrix[sn.id][fn.id]
                                rt_cost_after = rt_cost_before - rt_removed_cost + rt_added_cost
                                rt1_cost_after = rt_cost_after
                                rt2_cost_after = rt_cost_after
                            # if first_route_node > second_route_node and A.id == sn.id and D.id == fn.id:
                            #     rt_removed_cost = self.distanceMatrix[C.id][sn.id] + self.distanceMatrix[fn.id][B.id]
                            #     rt_added_cost = self.distanceMatrix[C.id][fn.id] + self.distanceMatrix[sn.id][B.id]
                            #     rt_cost_after = rt_cost_before - rt_removed_cost + rt_added_cost
                            #     rt1_cost_after = rt_cost_after
                            #     rt2_cost_after = rt_cost_after
                        else:
                            rt1_removed_cost = self.distanceMatrix[A.id][fn.id] + self.distanceMatrix[fn.id][B.id]
                            rt1_added_cost = self.distanceMatrix[A.id][sn.id] + self.distanceMatrix[sn.id][B.id]
                            rt1_cost_after = rt1_cost_before - rt1_removed_cost + rt1_added_cost
                            rt2_cost_before = rt2.time
                            rt2_removed_cost = self.distanceMatrix[C.id][sn.id] + self.distanceMatrix[sn.id][D.id]
                            rt2_added_cost = self.distanceMatrix[C.id][fn.id] + self.distanceMatrix[fn.id][D.id]
                            rt2_cost_after = rt2_cost_before - rt2_removed_cost + rt2_added_cost
                        if rt1_cost_before > rt1_cost_after > rt2_cost_after or rt_cost_after < rt1_cost_before:
                            for i in range(len(sol.routes)):

                                if sol.routes[i] == rt1:
                                    sol.routes[i].sequenceOfNodes[first_route_node] = sn
                                    sol.routes[i].cost = rt1_cost_after
                                    sol.routes[i].capacity = sol.routes[
                                                                      i].capacity - rt1_removed_demand + rt1_added_demand
                                    sol.routes[i].load = sol.routes[
                                                                  i].load - rt1_removed_demand + rt1_added_demand
                                if sol.routes[i] == rt2:
                                    sol.routes[i].sequenceOfNodes[second_route_node] = fn
                                    sol.routes[i].cost = rt2_cost_after
                                    sol.routes[i].capacity = sol.routes[
                                                                      i].capacity - rt2_removed_demand + rt2_added_demand
                                    sol.routes[i].load = sol.routes[
                                                                  i].load - rt2_removed_demand + rt2_added_demand
        return sol


    def twoOpt(self, routesgiven):
        tryingroutes = routesgiven.copy()
        newroutes = []
        loo = 1
        for route in tryingroutes:
            noimprovement = False
            eligible = len(route.sequenceOfNodes)
            existing_route = copy.deepcopy(route)
            while noimprovement is False:
                existing_route_time = self.calculateRouteTime(existing_route)
                toBeBroken = False
                for i in range(1, eligible):
                    if toBeBroken is True:
                        break
                    for k in range(i + 1, eligible + 1):
                        new_route = self.twoOptSwap(copy.deepcopy(existing_route), i, k)
                        new_route_time = self.calculateRouteTime(new_route)
                        if (new_route_time < existing_route_time):
                            #print("it happened")
                            existing_route = new_route
                            existing_route_time = new_route_time
                            existing_route.time = self.calculateRouteTime(new_route)
                            noimprovement = False
                            toBeBroken = True
                            break
                        else:
                            noimprovement = True
            #print("2 opt route", loo)
            #for nodes in existing_route.sequenceOfNodes:
                #print(nodes.id)
            #print(existing_route.time, existing_route_time)
            loo += 1
            newroutes.append(existing_route)
        return newroutes


# MAIN

drawer = SolDrawer()


def localSearchApplier(latestsol, method):
    if method == "2opt":
        twopt = Solution()
        twopt.routes = s.twoOpt(copy.deepcopy(latestsol.routes))
        solutionCostComputer(twopt)
        drawer.draw("opt2_fig", twopt, m.all_nodes)
        latestsol = copy.deepcopy(twopt)
        return latestsol

    elif method == "relocation":
        rl_sol = Solution()
        rl_sol.routes = s.relocateFromMax(copy.deepcopy(latestsol.routes))
        solutionCostComputer(rl_sol)
        drawer.draw("rel_fig", rl_sol, m.all_nodes)
        latestsol = copy.deepcopy(rl_sol)
        return latestsol

    elif method == "swapmax":
        sol_swapmax = Solution()
        sol_swapmax.routes = s.swapLocalSearch(copy.deepcopy(latestsol.routes))
        solutionCostComputer(sol_swapmax)
        drawer.draw("swapmax_fig", sol_swapmax, m.all_nodes)
        latestsol = copy.deepcopy(sol_swapmax)
        return latestsol

    elif method == "swapall":
        for i in range(400):
            maxroute = findMaxRoute(latestsol)
            latestsol = s.swapMoveAllChanges(maxroute, copy.deepcopy(latestsol))
        solutionCostComputer(latestsol)
        drawer.draw("swapall_fig", latestsol, m.all_nodes)
        return latestsol

    elif method == "two opt":
        s.twoOptMove(latestsol.routes)
        drawer.draw("twooptmovefig", latestsol, m.all_nodes)
        solutionCostComputer(latestsol)
        return latestsol







def findMaxRoute(sol1):
    times = []
    for route in sol1.routes:
        times.append(s.calculateRouteTime(route))
    sol1.cost = max(times)
    ind = (times.index(max(times)))
    return sol1.routes[ind]

def solutionCostComputer(sol):
    times = []
    load = []
    for route in sol.routes:
        times.append(s.calculateRouteTime(route))
        load.append(route.load)
        print(times[-1], load[-1], route.sequenceOfNodes[1].id, route.sequenceOfNodes[-1].id)
    sol.cost = max(times)
    ind = (times.index(max(times)))
    print("ROUTE MAX", sol.cost, ind + 1, len(times))

m = Model()
m.BuildModel()
s = Solver(m)
sol = Solution()
sol.routes = s.sweepMethod("nearest")
solutionCostComputer(sol)
drawer.draw("base", sol, m.all_nodes)
latestsol = copy.deepcopy(sol)
#testsol(latestsol.routes, m)
#latestsol = localSearchApplier(latestsol, "2opt")
#testsol(latestsol.routes, m)
#latestsol = localSearchApplier(latestsol, "relocation")
#testsol(latestsol.routes, m)
#latestsol = localSearchApplier(latestsol, "swapmax")
#testsol(latestsol.routes, m)
latestsol = localSearchApplier(latestsol, "two opt")


print("0-25-73", m.deliverytime[0][24] + m.deliverytime[25][72], "0-73", m.deliverytime[0][72])

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



        loo = 0
        times = []

        for croute in routes:
            print(croute.time, croute.load)
            loo += len(croute.sequenceOfNodes) - 1
            times.append(croute.time)
            for i in range(len(croute.sequenceOfNodes) - 1):
                thisid = croute.sequenceOfNodes[i].id
                nextid = croute.sequenceOfNodes[i + 1].id
                print("from", thisid, "to", nextid)
                print(self.dist_matrix[thisid][nextid], "km distance in", self.deliverytime[thisid][nextid - 1], "hours", '\n')

        print(len(routes))
        print(loo)
        print(max(times))




import sys
def prt_width(myint):
    sys.stdout.write('|' + str(myint) + '|' + ' '*(3 - len(str(myint))) + '  ')
    sys.stdout.flush()

for node in anglingNodes:
    prt_width(node.id)
    prt_width(node.x)
    prt_width(node.y)
    prt_width(node.demand)
    prt_width(node.type)
    print('\n')
"""

"""
new_sol = Solution()
new_sol.routes = s.twoOpt(sol.routes)

times = []
for newroute in new_sol.routes:
    #for nodes in newroute.sequenceOfNodes:
        #print(nodes.id)
    #print("route time", route.time)
    #print(s.calculateRouteTime(route))
    tim = newroute.time
    print(newroute.sequenceOfNodes[1].id)
    #print("time new route", tim)
    times.append(tim)
new_sol.cost = max(times)
ind = (times.index(max(times)))
print(new_sol.cost, ind+1, len(times))


i = 1
for route in new_sol.routes:
    print("route", i, "   ", route.load, route.time)
    for node in route.sequenceOfNodes:
        print(node.id)
    i +=1

rltimes = []
for route in rl_sol.routes:
    rltimes.append(s.calculateRouteTime(route))
    #print(route.sequenceOfNodes[1].id)

    #print(route.time)
rl_sol.cost = max(rltimes)
ind = (rltimes.index(max(rltimes)))
print(rl_sol.cost, ind+1, len(rltimes))
"""