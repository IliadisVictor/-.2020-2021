import math
import random



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

class Route:
    def __init__(self, dp, cap):
        self.sequenceOfNodes = []
        self.sequenceOfNodes.append(dp)
        self.time = 0
        self.capacity = cap
        self.load = 0

#pprint.pprint(deliverytime)



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
class Solver:
    def __init__(self, m):
        self.all_nodes = m.all_nodes
        self.service_locations = m.service_locations
        self.deliverytime = m.deliverytime
        self.depot = m.all_nodes[0]
        self.dist_matrix = m.dist_matrix
        self.capacity = m.capacity
        self.sol = None





    def sweepMethod(self):
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
        visited = []
        routes = []
        uvispol = polarsOfAngling.copy()
        uvispol.pop(0)
        timebarrier = 4.66
        yaw = [1]
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
                    print(ind+1, load, time)
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
                        print("else")
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
                            ccluster = []
                            iwannatrymore = False
                #print([node.id for node in croute.sequenceOfNodes], ind+1, load, time)


        croute.load = load
        croute.time = time
        routes.append(croute)
        """
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
        """
        self.sol = routes
        return self.sol

m = Model()
m.BuildModel()
s = Solver(m)
sol = s.sweepMethod()





"""




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
