for i in range (0,100):
    maxroute=0
    maxvalue=0
    for i in range(0, len(routes)):
        rt=routes[i]
        if maxvalue <rt.cost:
            maxroute=i
            maxvalue=rt.cost
    movereducal=0
    foundreplacement=False
    for targetRouteIndex in range(0, len(routes)):
        rt2 = routes[targetRouteIndex]
        if rt2==routes[maxroute]:
            continue
        for originNodeIndex in range(1, len(routes[maxroute].sequenceOfNodes)):
            if routes[maxroute].sequenceOfNodes[originNodeIndex].demand+rt2.load>3000:
                continue
            for targetNodeIndex in range(1, len(rt2.sequenceOfNodes)):
                A = routes[maxroute].sequenceOfNodes[originNodeIndex - 1]
                B = routes[maxroute].sequenceOfNodes[originNodeIndex]
                origincost=-dist_matrix[A.id][B.id]
                if originNodeIndex!=len(routes[maxroute].sequenceOfNodes)-1:
                    C = routes[maxroute].sequenceOfNodes[originNodeIndex + 1]
                    origincost=-dist_matrix[A.id][B.id]-dist_matrix[B.id][C.id]+dist_matrix[A.id][C.id]
                F = rt2.sequenceOfNodes[targetNodeIndex]
                targetcost=dist_matrix[F.id][B.id]
                if targetNodeIndex!=len(rt2.sequenceOfNodes)-1:
                    G = rt2.sequenceOfNodes[targetNodeIndex + 1]
                    targetcost=dist_matrix[F.id][B.id]+dist_matrix[B.id][G.id]-dist_matrix[F.id][G.id]
                total=origincost+targetcost
                if total>movereducal and rt2.cost+targetcost<routes[maxroute].cost:
                    foundreplacement=True
                    movereducal=total
                    originroute=routes[maxroute]
                    targetroute=rt2
                    originnode=originNodeIndex
                    targetnode=targetNodeIndex
                    keeporigincost=origincost
                    keeptargetcost=targetcost
    if foundreplacement==True:
        originroute.cost=originroute.cost+keeporigincost
        targetroute.cost=targetroute.cost+keeptargetcost
        targetroute.load=targetroute.load+originroute.sequenceOfNodes[originnode].demand
        originroute.load=originroute.load-originroute.sequenceOfNodes[originnode].demand
        targetroute.sequenceOfNodes.insert(targetnode,originroute.sequenceOfNodes[originnode])
        del originroute.sequenceOfNodes[originnode]