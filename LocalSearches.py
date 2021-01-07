def SwapLocalSearch(inputroutes):
    for i in range(0, 200):
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
                if inputroutes[maxroute]==rt2:
                    continue
                for firstNodeIndex in range(1, len(inputroutes[maxroute].sequenceOfNodes) ):
                    for secondNodeIndex in range(1, len(rt2.sequenceOfNodes)):
                        b1 = inputroutes[maxroute].sequenceOfNodes[firstNodeIndex]
                        b2 = rt2.sequenceOfNodes[secondNodeIndex]
                        if inputroutes[maxroute].load - b1.demand + b2.demand > 3000:
                                continue
                        if rt2.load - b2.demand + b1.demand > 3000:
                                continue

                        origincost=0
                        targetcost=0

                        a1 = inputroutes[maxroute].sequenceOfNodes[firstNodeIndex - 1]
                        a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
                        if firstNodeIndex !=len(inputroutes[maxroute].sequenceOfNodes)-1:
                            c1 = inputroutes[maxroute].sequenceOfNodes[firstNodeIndex + 1]
                            origincost = origincost-dist_matrix[b1.id][c1.id]+dist_matrix[b2.id][c1.id]
                        origincost=origincost-dist_matrix[a1.id][b1.id]+dist_matrix[a1.id][b2.id]

                        if secondNodeIndex!=len(rt2.sequenceOfNodes)-1:
                            c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]
                            targetcost=targetcost+dist_matrix[b1.id][c2.id]-dist_matrix[b2.id][c2.id]
                        targetcost = targetcost + dist_matrix[a2.id][b1.id] - dist_matrix[a2.id][b2.id]
                        if origincost < movereducal and rt2.time + targetcost < inputroutes[maxroute].time:
                            foundreplacement = True
                            keeporiginnode = firstNodeIndex
                            keeptargetnode = secondNodeIndex
                            targetrtindex = secondRouteIndex
                            originrtindex = maxroute
                            keeporigincost = origincost
                            keeptargetcost = targetcost
        if foundreplacement==True:
                    rt1 = inputroutes[originrtindex]
                    rt2 = inputroutes[targetrtindex]
                    b1 = rt1.sequenceOfNodes[keeporiginnode]
                    b2 = rt2.sequenceOfNodes[keeptargetnode]
                    rt1.sequenceOfNodes[keeporiginnode] = b2
                    rt2.sequenceOfNodes[keeptargetnode] = b1
                    rt1.time +=keeporigincost
                    rt2.time +=keeptargetcost
                    rt1.load = rt1.load - b1.demand + b2.demand
                    rt2.load = rt2.load + b1.demand - b2.demand

def relocatefrommax(relroutes):
    for i in range (0,200):
        maxroute=0
        maxvalue=0
        for i in range(0, len(relroutes)):
            rt=relroutes[i]
            if maxvalue <rt.time:
                maxroute=i
                maxvalue=rt.time
        movereducal=0
        foundreplacement=False
        for targetRouteIndex in range(0, len(relroutes)):
            rt2 = relroutes[targetRouteIndex]
            if rt2==relroutes[maxroute]:
                continue
            for originNodeIndex in range(1, len(relroutes[maxroute].sequenceOfNodes)):
                if relroutes[maxroute].sequenceOfNodes[originNodeIndex].demand+rt2.load>3000:
                    continue
                for targetNodeIndex in range(0, len(rt2.sequenceOfNodes)):
                    A = relroutes[maxroute].sequenceOfNodes[originNodeIndex - 1]
                    B = relroutes[maxroute].sequenceOfNodes[originNodeIndex]
                    origincost=-dist_matrix[A.id][B.id]
                    if originNodeIndex!=len(relroutes[maxroute].sequenceOfNodes)-1:
                        C = relroutes[maxroute].sequenceOfNodes[originNodeIndex + 1]
                        origincost=-dist_matrix[A.id][B.id]-dist_matrix[B.id][C.id]+dist_matrix[A.id][C.id]
                    F = rt2.sequenceOfNodes[targetNodeIndex]
                    targetcost=dist_matrix[F.id][B.id]
                    if targetNodeIndex!=len(rt2.sequenceOfNodes)-1:
                        G = rt2.sequenceOfNodes[targetNodeIndex + 1]
                        targetcost=dist_matrix[F.id][B.id]+dist_matrix[B.id][G.id]-dist_matrix[F.id][G.id]
                    if origincost<movereducal and rt2.time+targetcost<relroutes[maxroute].time:
                        foundreplacement=True
                        keeporiginnode=originNodeIndex
                        keeptargetnode=targetNodeIndex
                        targetrtindex=targetRouteIndex
                        keeporigincost=origincost
                        keeptargetcost=targetcost
        if foundreplacement==True:
            originroute = relroutes[maxroute]
            targetroute = relroutes[targetrtindex]
            B = originroute.sequenceOfNodes[keeporiginnode]
            if maxroute==targetrtindex:
                del originroute.sequenceOfNodes[ keeporiginnode]
                if (keeporiginnode <  keeptargetnode):
                    targetroute.sequenceOfNodes.insert( keeptargetnode, B)
                else:
                    targetroute.sequenceOfNodes.insert( keeptargetnode + 1, B)
                targetroute.cost=calcroutecost(targetroute)
            else:
                originroute.time=originroute.time+keeporigincost
                targetroute.time=targetroute.time+keeptargetcost
                originroute.load -=B.demand
                targetroute.load += B.demand
                del originroute.sequenceOfNodes[keeporiginnode]
                targetroute.sequenceOfNodes.insert(keeptargetnode + 1, B)
                
 def calcroutecost(rt):
    routecost=0
    for j in range(0, len(rt.sequenceOfNodes) - 1):
        k = rt.sequenceOfNodes[j].id
        l = rt.sequenceOfNodes[j + 1].id
        routecost = routecost + dist_matrix[k][l]
    return routecost



dist_matrix = [[0.0 for j in range(0, len(all_nodes))] for k in range(0, len(all_nodes))]
for i in range(0, len(all_nodes)):
    for j in range(0, len(all_nodes)):
        source = all_nodes[i]
        target = all_nodes[j]
        dx_2 = (source.x - target.x)**2
        dy_2 = (source.y - target.y) ** 2
        dist = round(math.sqrt(dx_2 + dy_2))
        dist_matrix[i][j] = dist / 35 + 5/60 + (all_nodes[j].type - 1) * 10/60
