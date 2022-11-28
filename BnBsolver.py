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
from commonLib import parse_graph

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
    adjacent,nodeEdges,nodeLabel,nNodes,nEdges = parse_graph(inputFile)
    returnDict['adjacent'] = adjacent
    returnDict['nodeEdges'] = nodeEdges
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
        'nodeEdges': nodeEdges
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
            # pass
            print(nIterations)
            print(len(q))
        nIterations = nIterations + 1
        # if nIterations > 50000:
        #     print('Done')
        #     return
        # Branch by either adding, or not adding best node
        withNode, withoutNode = splitNode(u[1][1])
        # DEBUG
        if withNode['covered'] > bestCover:
            bestCover = withNode['covered']
            #print(f"Best Cover: {withNode['covered']} : {withNode['nodes']}")
        # if len(withNode['nodes']) > mostNodes:
        #     print(f"Most nodes used: {withNode['nodes']}")
        #     mostNodes = len(withNode['nodes'])
        #print(withNode['nodes'])
        # Check to see if all edges are now covered
        if withNode['covered'] == nEdges:
            # If so, check cost vs current best
            if len(withNode['nodes']) < bestCost:
                bestCost = len(withNode['nodes'])
                print(f"NEW BEST COST FOUND!!! {len(withNode['nodes'])}")
        else:
            # If not check lower bound on cost to go
            # withCost = lowerBound(withNode,nEdges) + len(withNode['nodes'])
            withCost = lowerBound(withNode,nEdges) / (len(withNode['nodes'])+1)
            # NOTE: This favors longer chains first, hopefully either getting to solutions or eleminating nodes
            if withCost < bestCost:
                # make sure it's not empty
                if len(withNode['adjacent'][0]) != 0:
                    # if minimum cost to go is less than current best add to queue
                    q[nIterations] = (withCost,withNode)
                    
        # check lower bound on cost to go without node:
        # withoutCost = lowerBound(withoutNode,nEdges) + len(withoutNode['nodes'])
        withoutCost = lowerBound(withoutNode,nEdges) / (len(withoutNode['nodes'])+1)
        # NOTE: This favors longer chains first, hopefully either getting to solutions or eleminating nodes
        if withoutCost < bestCost:
            # make sure it's not empty
            if len(withoutNode['adjacent'][0]) != 0:
                # if minimum cost to go is less than current best add to queue
                q[nIterations+0.5] = (withoutCost,withoutNode)
    print('Done!')
    print(f"{nIterations} Iterations")
    
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
    adjacent = withNode['adjacent'].pop(0)
    
    # Create output without node selected
    withoutNode = {
        'covered': withNode['covered'],             # number of covered unique edges
        'nodes': withNode['nodes'].copy(),          # included nodes
        'nodeLabel':withNode['nodeLabel'].copy(),   # node labels
        'adjacent': [[]]*len(withNode['nodeLabel']),    # List of all edges covered by each node
        'nodeEdges': withNode['nodeEdges'].copy()
    }
    
    # ensure a true copy of each sublist is made
    for ii in range(0,len(withNode['nodeLabel'])):
        withoutNode['adjacent'][ii] = withNode['adjacent'][ii].copy()
    
    for toNode in adjacent:        
        try:
            # find index of node within list
            nodeInd = withNode['nodeLabel'].index(toNode)
        except(ValueError):
            # node already deleted
            continue
        # remove fromNode from that node's edges
        withNode['adjacent'][nodeInd].remove(fromNode)
        # subtract 1 from toNode's edges
        withNode['nodeEdges'][nodeInd] = withNode['nodeEdges'][nodeInd] - 1
    
    # sort by number of edges
    test = sorted(zip(withNode['nodeEdges'], withNode['nodeLabel'], withNode['adjacent']), key=lambda x: x[0],reverse=True)
    # 'unzip'
    withNode['nodeEdges'], withNode['nodeLabel'], withNode['adjacent'] = zip(*test)
    withNode['nodeEdges'] = list(withNode['nodeEdges'])
    withNode['nodeLabel'] = list(withNode['nodeLabel'])
    withNode['adjacent'] = list(withNode['adjacent'])
    
    # add node to included nodes
    withNode['nodes'].append(fromNode)
    withNode['covered'] = withNode['covered'] + fromEdges
    
    # Return candidates
    return withNode,withoutNode

if __name__ == '__main__':
    testDict = dict()
    #_solveBnB('./DATA-1/dummy1.graph','./test.out',testDict)
    # _solveBnB('./DATA-1/karate.graph','./test.out',testDict)
    _solveBnB('./DATA-1/jazz.graph','./test.out',testDict)
    # _solveBnB('./DATA-1/email.graph','./test.out',testDict)
    # _solveBnB('./DATA-1/star2.graph','./test.out',testDict)