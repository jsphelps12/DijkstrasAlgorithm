#!/usr/bin/python3
import sys
import math
from CS312Graph import *
import time


class NetworkRoutingSolver:
    def __init__( self):
        #global variables that save prev and dist from compute shortest hull
        globalPrev = []
        globalDist = []
        pass

    def initializeNetwork( self, network ):
        assert( type(network) == CS312Graph )
        self.network = network


    def getShortestPath( self, destIndex ):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        # USE RETURN VALUES FROM COMPUTE (PREV AND DIST) TO BUILD THE SHORTEST PATH
        path_edges = []
        total_length = 0
        #create the drawing of the shortest path
        i = destIndex
        while self.globalPrev[i] != None:
            x = 0
            for j in self.globalPrev[i].neighbors:
                if j.dest.node_id == i:
                    x = j
            path_edges.append((x.src.loc, x.dest.loc, '{:.0f}'.format(x.length) ))
            total_length += x.length
            i = x.src.node_id
        if(self.globalDist[destIndex] == math.inf):
            total_length = math.inf
        return {'cost':total_length, 'path':path_edges}

# this is makequeue for the heap
    def makeQueue(self):
        #use two dictionaries
        v = math.inf
        ids = [j.node_id for j in self.network.nodes]
        #create a dictionary mapping the node ids to the distances
        dist_map = {k: v for k in ids}
        dist_map[self.source] = 0
        #insert the source node first and then the rest of them
        heap = []
        heap.append(self.source)
        pos_map = {self.network.nodes[self.source].node_id: 0}
        for i in dist_map:
            if(dist_map[self.source] != dist_map[i]):
                heap.append(i)
            if (self.source < i):
                pos_map[i]  = i
            if (self.source > i):
                pos_map[i] = i+1


        return dist_map,heap, pos_map

    def deleteMin(self,map, queue,pos_map):
        # #return the node on top
        # u = queue.pop(0)
        # #if that was the last node, return
        # if len(queue) ==0:
        #     return u
        # #put the last node on top
        # queue.insert(0,queue.pop(-1))
        if (len(queue) == 1):
            u = queue.pop(0)
            return u
            # put the last node on top
        pos_map[queue[0]] = -1
        pos_map[queue[-1]] = 0
        temp = queue[0]
        queue[0] = queue[-1]
        queue[-1] = temp
        u = queue.pop(-1)
        i = 0
        #sort the top node down as needed, swapped with the lesser of its children
        keepGoing = True
        while keepGoing:
            if ((2*(i+1)-1 >= len(queue))):
                break;
            l = (2 * (i + 1)) - 1
            if((2*(i+1)) < len(queue)):
                r = 2*(i+1)
                if map[queue[i]] > min(map[queue[l]],map[queue[r]]):
                    x = 0
                    if(map[queue[l]] > map[queue[r]]):
                        x = r
                    else:
                        x = l
                    pos_map[queue[i]] = x
                    pos_map[queue[x]] = i
                    temp = queue[i]
                    queue[i] = queue[x]
                    queue[x] = temp
                    i = x
                else:
                    keepGoing = False
            #in case of only one child
            else:
                if map[queue[i]] > map[queue[l]]:
                    x = l
                    pos_map[queue[i]] = x
                    pos_map[queue[x]] = i
                    temp = queue[i]
                    queue[i] = queue[x]
                    queue[x] = temp
                    i = x
                else:
                    keepGoing = False
        return u

    def decreaseKey(self,map, queue, vertex, neighbor, length,pos_map):
        #update the key
        #map[neighbor.dest.node_id] = map[vertex.node_id] + length
        #if the key is less than its parent, swap
        i = pos_map[neighbor.dest.node_id]
        while i !=0:
            if i % 2 == 0:
                parent = (i//2)-1
            else:
                parent = ((i + 1) // 2) - 1
            if map[queue[i]] < map[queue[parent]]:
                pos_map[queue[i]] = parent
                pos_map[queue[parent]] = i
                temp = queue[i]
                queue[i] = queue[parent]
                queue[parent] = temp
                i = parent
            else:
                break

    def heapDijk(self):
        #initialize prev
        prev = [None] * len(self.network.nodes)
        #makequeue
        dist_map, queue, pos_map = self.makeQueue()
        # run dijkstras algo
        while len(queue) != 0:
            u = self.deleteMin(dist_map,queue,pos_map)
            vertex = self.network.nodes[u]
            for neighbor in vertex.neighbors:
                length = neighbor.length
                if dist_map[neighbor.dest.node_id] > dist_map[vertex.node_id] + length:
                    dist_map[neighbor.dest.node_id] = dist_map[vertex.node_id] + length
                    prev[neighbor.dest.node_id] = vertex
                    self.decreaseKey(dist_map,queue,vertex,neighbor, length,pos_map)
                    #queue[neighbor.dest.node_id] = dist_map[vertex.node_id] + length
        return prev, list(dist_map.values())


    def arrayDijk(self):
        #initialize prev and dist which will be returned
        prev = [None]*len(self.network.nodes)
        dist = [math.inf]*len(self.network.nodes)
        v = math.inf
        networkCopy = self.network
        ids = [j.node_id for j in self.network.nodes]
        #create the prioirty queue as a dictionary
        queue  = {k:v for k in ids}
        #initialize values for the source
        dist[self.source] = 0
        queue[self.source] = 0
        while len(queue) != 0:
            #returns value and then deletes the entry for the top priority
            u = min(queue, key = queue.get)
            queue.pop(min(queue, key = queue.get))
            vertex = self.network.nodes[u]
            for neighbor in vertex.neighbors:
                length = neighbor.length
                if dist[neighbor.dest.node_id] > dist[vertex.node_id]+ length:
                    dist[neighbor.dest.node_id] = dist[vertex.node_id] + length
                    prev[neighbor.dest.node_id] = vertex
                    queue[neighbor.dest.node_id] = dist[vertex.node_id] + length
        return prev, dist






    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()
        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)

        if use_heap:
            self.globalPrev, self.globalDist = self.heapDijk()

        else:
            self.globalPrev, self.globalDist = self.arrayDijk()

        t2 = time.time()
        return (t2-t1)

