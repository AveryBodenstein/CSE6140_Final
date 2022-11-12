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
        nodeLabel = [0]*nNodes

        # Parse remainder of file
        for ii in range(0,nNodes):
            tempLine = graphFile.readline()
            # parse edge and weight
            edge_data = list(map(lambda x: int(x), tempLine.split()))
            nodeEdges[ii] = len(edge_data)
            adjacent[nodeID[ii]:nodeID[ii]+nodeEdges[ii]] = edge_data
            nodeID[ii+1] = nodeID[ii] + nodeEdges[ii]
            nodeLabel[ii] = ii+1
        # remove extra value from nodeID list
        del(nodeID[-1])
    return adjacent,nodeID,nodeEdges,nodeLabel,nNodes,nEdges

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
    adjacent,nodeID,nodeEdges,nodeLabel,nNodes,nEdges = parse_graph(inputFile)
    returnDict['adjacent'] = adjacent
    returnDict['nodeID'] = nodeID
    returnDict['nodeEdges'] = nodeEdges
    #print(f"adjacent: {adjacent}")
    #print(f"nodeID: {nodeID}")
    # Initialize current best cost to requiring all nodes
    bestCost = nNodes + 1
    bestCover = 0
    mostNodes = 0
    # Create first candidate with no nodes
    testDict = {
        'covered': 0,            # number of covered unique edges
        'nodes': [],             # included nodes
        'nodeLabel':nodeLabel,   # node labels
        'adjacent': adjacent,    # List of all edges covered by each node
        'nodeEdges': nodeEdges,  # number of uncovered edges covered by each node
        'nodeID':nodeID          # index into nodeEdges for each node
    }
    # Create empty priority queue
    q = hd.heapdict()
    q[0] = [lowerBound(testDict,nEdges),testDict]
    # while queue is not empty
    nIterations = 0
    while q:
        # Choose current candidate with best cost to go
        u = q.popitem()
        # DEBUG
        #print(u[1][1])
        
        #print(len(q))
        if (nIterations % 10000) == 0:
            print(nIterations)
        nIterations = nIterations + 1
        #if nIterations > 10000000:
        #    return
        # Branch by either adding, or not adding best node
        withNode, withoutNode = splitNode(u[1][1])
        # DEBUG
        if withNode['covered'] > bestCover:
            bestCover = withNode['covered']
            print(f"Best Cover: {withNode['covered']} : {withNode['nodes']}")
        # if len(withNode['nodes']) > mostNodes:
        #     print(f"Most nodes used: {withNode['nodes']}")
        #     mostNodes = len(withNode['nodes'])
        #print(withNode['nodes'])
        # Check to see if all edges are now covered
        if withNode['covered'] == nEdges:
            # If so, check cost vs current best
            if len(withNode['nodes']) < bestCost:
                bestCost = len(withNode['nodes'])
                print('NEW BEST COST FOUND!!!')
        else:
            # If not check lower bound on cost to go
            withCost = lowerBound(withNode,nEdges) + len(withNode['nodes'])
            if withCost < bestCost:
                # if minimum cost to go is less than current best add to queue
                q[nIterations] = (withCost,withNode)
        # check lower bound on cost to go without node:
        withoutCost = lowerBound(withoutNode,nEdges) + len(withoutNode['nodes'])
        if withoutCost < bestCost:
            # if minimum cost to go is less than current best add to queue
            q[nIterations+0.5] = (withoutCost,withoutNode)
    print('Done!')
    
# NOTE: rewrite to "update lower bound" change input/output(?) to candidate tuple
def lowerBound(inDict,nEdges):
    # Calculates minimum number of nodes required to cover all remaining edges
    nCovered = inDict['covered']
    ii = 0
    # Add largest (most adjacent edges) nodes until number of edges is reached
    maxEdges = len(inDict['nodeEdges'])
    while nCovered < nEdges and ii<maxEdges:
        nCovered = nCovered + inDict['nodeEdges'][ii]
        ii = ii + 1
    if nCovered < nEdges:
        # impossible to cover entire graph
        return nEdges
    else:
        return ii
    
    
def splitNode(inputDict):
    # Create output with node selected
    withNode = inputDict
    
    # Extract values for clarity
    fromNode = withNode['nodeLabel'].pop(0)
    fromEdges = withNode['nodeEdges'].pop(0)
    fromID = withNode['nodeID'].pop(0)
    
    # Create output without node selected
    withoutNode = {
        'covered': withNode['covered'],             # number of covered unique edges
        'nodes': withNode['nodes'].copy(),          # included nodes
        'nodeLabel':withNode['nodeLabel'].copy(),   # node labels
        'adjacent': withNode['adjacent'].copy(),    # List of all edges covered by each node
        'nodeEdges': withNode['nodeEdges'].copy(),  # number of uncovered edges covered by each node
        'nodeID': withNode['nodeID'].copy()         # index into nodeEdges for each node
    }
    
    # for each uncovered edge from node
    for toNode in withNode['adjacent'][fromID:fromID+fromEdges]:
        try:
            # find index of node within list
            nodeInd = withNode['nodeLabel'].index(toNode)
        except(ValueError):
            # node already deleted
            continue
        # find index of nodeID within that nodes adjacent edges
        edgeInd = withNode['adjacent'][withNode['nodeID'][nodeInd]:withNode['nodeID'][nodeInd]+withNode['nodeEdges'][nodeInd]].index(fromNode)
        # remove nodeLabel from that nodes Edges
        del(withNode['adjacent'][withNode['nodeID'][nodeInd]+edgeInd])
        # add in dummy value at end of current node edges to maintain list length
        withNode['adjacent'].insert(withNode['nodeID'][nodeInd]+withNode['nodeEdges'][nodeInd]-1,0)
        # NOTE: This *might* be able to be done faster by just shifting all edges after the one to be deleted to the left by 1
        # subtract 1 from toNode's edges
        withNode['nodeEdges'][nodeInd] = withNode['nodeEdges'][nodeInd] - 1
        # check if that node has no remaining unique edges?
        
    
    # add node to included nodes
    withNode['nodes'].append(fromNode)
    withNode['covered'] = withNode['covered'] + fromEdges
    
    # Return candidates
    return withNode,withoutNode

if __name__ == '__main__':
    testDict = dict()
    #_solveBnB('./DATA-1/dummy1.graph','./test.out',testDict)
    _solveBnB('./DATA-1/karate.graph','./test.out',testDict)
    #_solveBnB('./DATA-1/jazz.graph','./test.out',testDict)
    