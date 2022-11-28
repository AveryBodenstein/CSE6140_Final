#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 2022

@author: Avery Bodenstein

Approximate Vertex Cover Solver
"""

#import numpy as np
#import time
import multiprocessing
from commonLib import parse_graph
import heapdict as hd

def solveApprox(inputFile,outputFile,maxTime):
    # Start solution as a process
    manager = multiprocessing.Manager()
    returnDict = manager.dict()
    p = multiprocessing.Process(target=_solveApprox, name="SolveApprox", args=(inputFile,outputFile,returnDict))
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

# implement Greedy Independent Cover
def _solveApprox(inputFile,outputFile,returnDict):
    # parse graph
    adjacent,nodeEdges,nodeLabel,nNodes,nEdges = parse_graph(inputFile)
    # create heapdict of nodes
    q = hd.heapdict()
    for ii in range(0,nNodes):
        #print(f"{ii},{nodeLabel[ii]},{nodeEdges[ii]}")
        q[nodeLabel[ii]] = [nodeEdges[ii],adjacent[ii]]
    # while graph not covered
    coveredEdges = 0
    coverSet = []
    while coveredEdges < nEdges:
        #print(coveredEdges)
        # select vertex of minimum degree
        curVertex = q.peekitem()
        #print(f"Selected {curVertex[0]}")
        # extract values for clarity
        #curLabel = curVertex[0]
        #curNEdges = curVertex[1][0]
        curNeighbors = curVertex[1][1].copy()
        #print(f"neighbors: {curNeighbors}")
        # Add all neighbors to cover
        for node in curNeighbors:
            # add label to cover set
            coverSet.append(node)
            # pop that node from queue
            #print(f"removing {node}")
            tempNode = q.pop(node)
            # add that nodes unique edges to covered set
            coveredEdges = coveredEdges + tempNode[0]
            #print(coveredEdges)
            # remove edges from target nodes
            for subNode in tempNode[1]:
                # update number of nodes
                q[subNode][0] = q[subNode][0] - 1
                # remove origin node from neighbors
                #print(f"before: {subNode}: {q[subNode]}")
                q[subNode][1].remove(node)
                #print(f"after: {subNode}: {q[subNode]}")
        # remove original vertex
        #print(f"removing {curVertex[0]}")
        q.pop(curVertex[0])
    return coverSet
        
        
if __name__ == "__main__":
    testDict = dict()
    # _solveApprox('./DATA-1/dummy1.graph','./test.out',testDict)
    coverSet = _solveApprox('./DATA-1/karate.graph','./test.out',testDict)
    #_solveApprox('./DATA-1/jazz.graph','./test.out',testDict)
    # _solveApprox('./DATA-1/email.graph','./test.out',testDict)
    # _solveApprox('./DATA-1/star2.graph','./test.out',testDict)
    
    print(len(coverSet))