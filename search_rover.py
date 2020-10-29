import heapq

of = open('output.txt','w+')

handle = open('input.txt','r+')
algorithm_string = (handle.readline()).split()     #algorithm to be implemented
dimension = (handle.readline()).split()   #reading matrix dimensions
W = int(dimension[0])
H = int(dimension[1])
start_coordinates = (handle.readline()).split() 
Y,X = int(start_coordinates[0]),int(start_coordinates[1])
elevation_difference_list = (handle.readline()).split()
elevation_difference = int(elevation_difference_list[0])
Number_of_target_locations = (handle.readline()).split()
N = int(Number_of_target_locations[0])  #number of goal state to search
l = []
for i in range(N):      #read multiple target locations
    landing_coordinates = (handle.readline()).split()
    lc = [int(j) for j in landing_coordinates]
    l.append(lc)
matrix = []
for j in range(H):
    line = handle.readline().split()
    mat = [int(i) for i in line]
    matrix.append(mat)

explored_set = dict()      #store the explored coordinates
explored_set_cost = []
parent = dict()     #store parents
index = []
oldCost = 0
wholelist = dict()

#Search funtion definitions
def bfs(x,y):
    index.clear()
    explored_set.clear()
    parent.clear()
    Flag = False        #false till finds a solution
    if X == x and Y == y:   #insert initial state into explored set and check if it is goal state
        Flag = True
    else:
        #print(y,x)
        index.append([X,Y])    #frontier node having coordinates
        explored_set.update({(X,Y) : 1}) #add to explored set
        while len(index) > 0:
            current = index.pop(0)
            currentX,currentY = current[0],current[1]
            for a in range(currentX-1,currentX+2):
                for b in range(currentY-1,currentY+2):
                    if a > -1 and b > -1 and a < H and b < W \
                        and (currentX,currentY) != (a,b)\
                        and (a,b) not in explored_set.keys()\
                        and abs(matrix[currentX][currentY] - matrix[a][b]) <= elevation_difference:
                        index.append([a,b])                                
                        explored_set.update({(a,b) : 1}) #add to explored set
                        parent.update({(b,a) : (currentY,currentX)})
                        #print(parent)
                        if (a,b) == (x,y):
                            #print("solutionsssssssssssssss")
                            Flag = True
                            break
    path = [(y,x)]
    output = ""
    try:
        while path[-1] != (Y,X):
            path.append(parent[path[-1]])
        path.reverse()
        if Flag:
            for i in path:
                output = output + " " + str(i[0]) + "," + str(i[1])
            of.write(output.lstrip())
            of.write("\n")
            of.close()
        else:
            output = output + "\n"
            of.write(output)
            of.close()
    except:
        output = output + "\n"
        of.write(output)
        of.close()

def ucs(x,y):
    explored_set.clear()
    index.clear()
    parent.clear()
    wholelist.clear()
    Flag = False
    path_cost = 0
    if X == x and Y == y:   #insert initial state into explored set and check if it is goal state
        output = ""
        output += str(y) + "," + str(x)
        output += "\n"
        of.write(output)
        Flag = True
    else:
        heapq.heappush(index,[path_cost,X,Y])      #frontier node having coordinates
        explored_set.update({(X,Y):1})#add to explored set
        wholelist.update({(X,Y) : path_cost})
        while len(index) != 0:
            current = heapq.heappop(index)
            currentCost,currentX,currentY = current[0],current[1],current[2]
            if (currentY,currentX) == (y,x):
                path = [(y,x)]
                Flag = True
                break
            if (currentX,currentY) in wholelist.keys():
                del wholelist[currentX,currentY]
            for a in range(currentX-1,currentX+2):
                for b in range(currentY-1,currentY+2):
                    if (a,b) not in wholelist.keys():
                        f = 1
                    else:
                        oldCost = wholelist[a,b]
                        f = 0
                    if (a > -1 and b > -1) and (a < H and b < W) \
                        and (a,b) not in explored_set.keys()\
                        and (currentX,currentY) != (a,b)\
                        and abs(matrix[currentX][currentY] - matrix[a][b]) <= elevation_difference: 
                        if (a == currentX or b == currentY):
                            newCost = currentCost + 10
                            heapq.heappush(index,[newCost,a,b])
                            explored_set.update({(a,b) : 1}) #add to explored set
                            wholelist.update({(a,b) : newCost})
                            parent.update({(a,b) : (currentY,currentX)})
                        else:
                            newCost = currentCost + 14
                            heapq.heappush(index,[newCost,a,b])
                            explored_set.update({(a,b) : newCost}) #add to explored set
                            wholelist.update({(a,b) : newCost})
                            parent.update({(b,a) : (currentY,currentX)})
                            
                    elif f == 0:
                        if (a == currentX or b == currentY):
                            newCost = currentCost + 10                            
                        if oldCost > newCost:                          
                                heapq.heappush(index,[newCost,a,b])
                                wholelist.update({(a,b) : newCost})
                                parent.update({(b,a) : (currentY,currentX)})
                        else:
                            newCost = currentCost + 14
                            if oldCost> newCost:
                                print(oldCost,newCost)
                                heapq.heappush(index,[newCost,a,b])
                                wholelist.update({(a,b) : newCost})
                                parent.update({(b,a) : (currentY,currentX)})
                            
    path = [(y,x)]
    output = ""
    try:
        while path[-1] != (Y,X):
            path.append(parent[path[-1]])
        path.reverse()
        print(path)
        if Flag:
            for i in path:
                output += " " + str(i[0]) + "," + str(i[1])
            output += "\n"
            of.write(output.lstrip())
            of.close()
        else:
            output = "FAIL"
            output += "\n"
            of.write(output)
    except:
        output = "FAIL"
        output = "\n"
        of.write(output)

def ass(x,y):
    Flag = False
    path_cost = 0
    if X == x and Y == y:   #insert initial state into explored set and check if it is goal state
        #print("goal state")
        #print(X,Y)
        Flag = True
    else:
        index = []
        heapq.heappush(index,[path_cost,X,Y])      #frontier node having coordinates
        #print(index)
        explored_set.update({(X,Y) : path_cost})#add to explored set
        explored_set_cost.append(path_cost)
        while len(index) != 0:
            current = heapq.heappop(index)
            #print(current)
            currentCost,currentX,currentY = current[0],current[1],current[2]
            #print("cost : ",currentCost)
            #print("X,Y : ",currentX,currentY)
            if (currentY,currentX) == (y,x):
                path = [(y,x)]
                #print("solutionsssssssssssss")
                Flag = True
                break
            for a in range(currentX-1,currentX+2):
                for b in range(currentY-1,currentY+2):
                    if (a > -1 and b > -1) and (a < H and b < W) \
                        and [a,b] not in explored_set\
                        and (currentX,currentY) != (a,b)\
                        and abs(matrix[currentX][currentY] - matrix[a][b]) <= elevation_difference: 
                        if (a == currentX or b == currentY):
                            newCost = currentCost + 10
                            #print("new a and b",a,b)
                            #print(" child cost : ",newCost)
                            heapq.heappush(index,[newCost,a,b])
                            #print("after pushing in queue : ",index)
                            explored_set_cost.append(newCost)
                            explored_set.append({(a,b) : newCost}) #add to explored set
                            #print(explored_set)
                            parent[b,a] = (currentY,currentX)
                        else:
                            newCost = currentCost + 14
                            #print(a,b)
                            #print("dcost : ",newCost)
                            heapq.heappush(index,[newCost,a,b])
                            #print("after pushing diagonal element in queue : ",index)
                            explored_set_cost.append(newCost)
                            explored_set.update({(a,b) : newCost}) #add to explored set
                            parent[b,a] = (currentY,currentX)
                            
                    elif (a > -1 and b > -1) and (a < H and b < W) \
                        and (currentX,currentY) != (a,b) and (a,b) != (X,Y)\
                        and [a,b] in explored_set\
                        and abs(matrix[currentX][currentY] - matrix[a][b]) <= elevation_difference:
                        #print("already in queue : ")
                        z = explored_set.index([a,b])
                        oldCost = explored_set_cost[z]
                        if (a == currentX or b == currentY):
                            newCost = currentCost + 10                            
                            if oldCost > newCost:
                                explored_set_cost[z] = newCost                           
                                #print(a,b)
                                #print("new cost : ",newCost)
                                heapq.heapreplace(index,[newCost,a,b])
                                parent[b,a] = (currentY,currentX)                                
                        else:
                            newCost = currentCost + 14
                            if oldCost> newCost:
                                explored_set_cost[z]=newCost 
                                #print(a,b)
                                #print("new cost : ",newCost)
                                heapq.heapreplace(index,[newCost,a,b])
                                parent[b,a] = (currentY,currentX)
                                
    path = [(y,x)]
    output = ""
    try:
        while path[-1] != (Y,X):
            path.append(parent[path[-1]])
        path.reverse()
        print(path[-1])
        if Flag:
            for i in path:
                output = output + " " + str(i[0]) + "," + str(i[1])
            handle = open("output.txt",'w+')
            handle.write(output.lstrip())
            handle.write("\n")

        else:
            handle = open('output.txt','w+')
            handle.write("FAIL")
            #return "FAIL"
    except:
        handle = open('output.txt','w+')
        handle.write("FAIL")
        #print("FAIL")
        
if(algorithm_string[0] == "BFS"):      #BFS implementation
    #print("BFS implementation")
    for b in range(N):
        #print(b)
        y,x = l[b][0],l[b][1]
        #print(b)
        #print(y,x)
        bfs(x,y)

elif(algorithm_string[0] == "UCS"):    #UCS
    #print("UCS implementation")
    for b in range(N):
        y,x = l[b][0],l[b][1]
        #print(x,y)
        #print(N)
        ucs(x,y)
    
elif(algorithm_string[0] == "A*"):     #A* search
    #print("A* search implementation")
    for b in range(N):
        y,x = l[b][0],l[b][1]
        #print(x,y)
        #print(N)
        ass(x,y)
