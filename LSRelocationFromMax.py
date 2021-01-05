def relocatefrommax(relroutes):
    for i in range (0,100):
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
                        foundreplacement=not foundreplacement
                        keeporiginnode=originNodeIndex
                        keeptargetnode=targetNodeIndex
                        targetrtindex=targetRouteIndex
                        keeporigincost=origincost
                        keeptargetcost=targetcost
        if foundreplacement==True:
            originroute=relroutes[maxroute]
            targetroute=relroutes[targetrtindex]
            originroute.time=originroute.time+keeporigincost
            targetroute.time=targetroute.time+keeptargetcost
            B=originroute.sequenceOfNodes[keeporiginnode]
            originroute.load -=B.demand
            targetroute.load += B.demand
            del originroute.sequenceOfNodes[keeporiginnode]
            targetroute.sequenceOfNodes.insert(keeptargetnode + 1, B)