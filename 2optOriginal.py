def Opt2move(optroutes):
    while True:
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
                        A = rt1.sequenceOfNodes[originNodeIndex]
                        B = rt1.sequenceOfNodes[originNodeIndex + 1]
                        K = rt2.sequenceOfNodes[targetNodeIndex]
                        L = rt2.sequenceOfNodes[targetNodeIndex + 1]
                        if rt1 == rt2:
                            if originNodeIndex == 0 and targetNodeIndex  == len(rt1.sequenceOfNodes) - 2:
                                continue
                            tryroyte=TwoOptroutesequal(optroutes,originRouteIndex,originNodeIndex,targetNodeIndex)
                            moveCost=tryroyte.cost-rt1.cost
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
                print(movereducal,keeprt1,keeprt2,routes[keeprt1].sequenceOfNodes[keepnode1].id,routes[keeprt2].sequenceOfNodes[keepnode2].id)
                Apply2opt(keeprt1,keeprt2,keepnode1,keepnode2,optroutes,movereducal)
                foundreplacement=False
        else:
            break

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

        rt1k.cost +=mc
    else:
        # slice with the nodes from position top.positionOfFirstNode + 1 onwards
        relocatedSegmentOfRt1 = rt1k.sequenceOfNodes[node1 + 1:]

        # slice with the nodes from position top.positionOfFirstNode + 1 onwards
        relocatedSegmentOfRt2 = rt2k.sequenceOfNodes[node2 + 1:]

        del rt1k.sequenceOfNodes[node1 + 1:]
        del rt2k.sequenceOfNodes[node2 + 1:]

        rt1k.sequenceOfNodes.extend(relocatedSegmentOfRt2)
        rt2k.sequenceOfNodes.extend(relocatedSegmentOfRt1)

        rt1k.cost = CalcCost(rt1k)
        rt1k.load = CalcCapacity(rt1k)
        rt2k.cost = CalcCost(rt2k)
        rt2k.load = CalcCapacity(rt2k)



def TwoOptroutesequal(rts,rt1index,node1index,node2index):
    rt1k: Route = rts[rt1index]
    routetoreturn=copy.deepcopy(rt1k)
    reversedSegment = reversed(rt1k.sequenceOfNodes[node1index + 1: node2index + 1])
    routetoreturn.sequenceOfNodes[node1index + 1: node2index + 1] = reversedSegment
    routetoreturn.cost=CalcCost(routetoreturn)
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