# Routes indicates the list that contains Route objects of your solution
routes=[]

# Prints all of your routes .
for i in range(0, len(routes)):
            rt = routes[i]
            for j in range (0, len(rt.sequenceOfNodes)):
                print(rt.sequenceOfNodes[j].id, end=' ',)
            print("\n")

# Returns object function value , ( The maximum cost of your routes)

def returnmax(routes):
    max=0
    for i in range(0, len(routes)):
        rt = routes[i]
        if max < rt.cost:
            max = rt.cost
    return max


#  Calculates each route cost and load specificaly based on ids.
for i in range(0, len(routes)):
            rt = routes[i]
            routecost=0
            routeload=0
            for j in range (0, len(rt.sequenceOfNodes)):
                routeload=routeload+all_nodes[rt.sequenceOfNodes[j].id].demand
                print(rt.sequenceOfNodes[j].id, end=' ',)
            for j in range(0, len(rt.sequenceOfNodes)-1):
                k=rt.sequenceOfNodes[j].id
                l=rt.sequenceOfNodes[j+1].id
                routecost=routecost+dist_matrix[k][l]
            print(routecost,routeload)
            print("\n")


# Check if All nodes are included in your solution , if false there are some missing

def CheckAllnodes(routes):
    checknodes=[]
    for i in range(0,201):
        checknodes.append(0)
    for i in range(0,len(routes)):
        rt=routes[i]
        for j in range (1, len(rt.sequenceOfNodes)):
            id=rt.sequenceOfNodes[j].id
            checknodes[id]=checknodes[id]+1
    checknodes[0]=1
    for i in range(0,201):
        if checknodes[i] !=1:
            return False
    return True