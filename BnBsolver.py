#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 2022

@author: Avery Bodenstein

Branch and Bound Vertex Cover Solver
"""

import numpy as np
import time
import multiprocessing
import heapdict as hd

def parse_graph(filename):
    # opens .graph file and parses contents
    with open(filename, 'r') as graphFile:
        # Parse first line as nNodes, nEdges
        tempLine = graphFile.readline()
        graph_data = list(map(lambda x: int(x), tempLine.split()))
        assert(len(graph_data) == 3)
        nNodes, nEdges, weighted = graph_data[0], graph_data[1], graph_data[2]

        # Create empty list to hold values
        adjacent = [0]*nEdges
        # Create empty list to hold ID of each node in adjacent list
        nodeID = [0]*(nNodes+1)
        # List of node adjacent edges
        nodeEdges = [0]*nNodes

        # Parse remainder of file
        for ii in range(0,nNodes):
            tempLine = graphFile.readline()
            # parse edge and weight
            edge_data = list(map(lambda x: int(x), tempLine.split()))
            nodeEdges[ii] = len(edge_data)
            adjacent[nodeID[ii]:nodeID[ii]+nodeEdges[ii]] = edge_data
            nodeID[ii+1] = nodeID[ii] + nodeEdges[ii]
        # remove extra value from nodeID list
        del(nodeID[-1])
    return adjacent,nodeID,nodeEdges,nNodes,nEdges

def solveBnB(inputFile,outputFile,maxTime):
    # Start solution as a process
    manager = multiprocessing.Manager()
    returnDict = manager.dict()
    p = multiprocessing.Process(target=_solveBnB, name="SolveBnB", args=(inputFile,outputFile,returnDict))
    p.start()

    # Wait a maximum of maxTime seconds for solution to complete (if function returns it passes)
    p.join(maxTime)
    
    # If thread is still active
    if p.is_alive():
        print(f"Maximum Computation Time Exceeded, max time seen: {returnDict['timeSeen']}")
    
        # Terminate solver
        p.terminate()
        p.join()
    
    # Extract value from function
    #print(f"Solver returned {returnDict['fVal']}")
    
    
def _solveBnB(inputFile,outputFile,returnDict):
    # Read input file
    adjacent,nodeID,nodeEdges,nNodes,nEdges = parse_graph(inputFile)
    returnDict['adjacent'] = adjacent
    returnDict['nodeID'] = nodeID
    returnDict['nodeEdges'] = nodeEdges
    #print(f"adjacent: {adjacent}")
    #print(f"nodeID: {nodeID}")
    # Initialize current best cost to requiring all nodes
    bestCost = nNodes
    # Create first candidate with no nodes
    testDict = {
        "adjacent": [], 
        'nodes': [],
        'nodeEdges': nodeEdges,
        'lowerBound': lowerBound([],[],nodeEdges,nEdges)
    }
    # Create empty priority queue
    q = hd.heapdict()
    q[0] = (lowerBound([],[],nodeEdges,nEdges),[],[],nodeEdges) # lower bound on cost to go, included edges (unique), included nodes, nodeEdges
    # while queue is not empty
    while q:
        # Choose current candidate with best cost to go
        u = q.popitem()
        # Branch by either adding, or not adding that node
        withNode, withoutNode = splitNode(u[1],nodeID)
        # Check to see if all edges are now covered
        if len(withNode[1]) == nEdges:
            # If so, check cost vs current best
            if len(withNode[2]) < bestCost:
                bestCost = len(withNode[2])
        else:
            # If not check lower bound on cost to go
            if withNode[0] + len(withNode[2]) < bestCost:
                # if minimum cost to go is less than current best add to queue
                q[len(q)] = withNode
        # check lower bound on cost to go without node:
        if withoutNode[0] + len(withoutNode[2]) < bestCost:
            # if minimum cost to go is less than current best add to queue
            q[len(q)] = withoutNode
    
# NOTE: rewrite to "update lower bound" change input/output(?) to candidate tuple
def lowerBound(adjacent, nodeID, nodeEdges,nEdges):
    # Calculates minimum number of nodes required to cover all remaining edges
    nCovered = len(adjacent)
    ii = 0
    # Add largest (most adjacent edges) nodes until number of edges is reached
    while nCovered < nEdges:
        nCovered = nCovered + nodeEdges[ii]
        ii = ii + 1
    return ii
    
    
# NOTE: candidate is a tuple with the format:
# (lower bound on cost to go, included edges (unique), included nodes, nodeEdges)
def splitNode(inputCandidate,nodeID):
    # Create output with node selected
    withNode = inputCandidate
    # remove that nodes edges from nodeEdges
    
    # Create output without node selected
    withoutNode = withNode
    # add node to included nodes
    withNode[2][len(withNode[2])] = nodeID
    # for each unique edge from node
    for toNode in curNodeEdges:
        # remove nodeID from that nodes Edges
        pass
    # update lower bound on cost to go
    withNode[0] = lowerBound(adjacent, nodeID, nodeEdges,nEdges)
    # Return candidates
    return withNode,withoutNode

if __name__ == '__main__':
    testDict = dict()
    _solveBnB('./DATA-1/karate.graph','./test.out',testDict)
    