#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 2022

@author: Avery Bodenstein

Approximate Vertex Cover Solver
Implements greedy independent cover algorithm
"""

import time
import multiprocessing
from commonLib import parse_graph_hd, write_sol, log_sol
import heapdict as hd
import os

# main solver function called from template.py launches a thread to initiate solver
# with a maximum time limit
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
        print("Maximum Computation Time Exceeded}")
    
        # Terminate solver
        p.terminate()
        p.join()
    
    # Write final value to solution file
    if os.path.exists(f"{outputFile}.trace"):
        os.remove(f"{outputFile}.trace")
    write_sol(outputFile,returnDict['coverSet'])
    log_sol(outputFile,len(returnDict['coverSet']),returnDict['solTime'])

# function wrapper to parse file
def _solveApprox(inputFile,outputFile,returnDict):
    # log start time
    startTime = time.perf_counter()
    # parse graph
    q, nNodes, nEdges = parse_graph_hd(inputFile)
    # solve
    coverSet = _solveApprox_nofile(q,nNodes,nEdges)
    # stop timing
    elapsedTime = time.perf_counter() - startTime
    # set return dictionary value
    returnDict['coverSet'] = coverSet
    returnDict['solTime'] = elapsedTime
    return coverSet

# implement Greedy Independent Cover
def _solveApprox_nofile(q,nNodes,nEdges):
    # while graph not covered
    coveredEdges = 0
    coverSet = []
    while coveredEdges < nEdges:
        # select vertex of minimum degree
        curVertex = q.peekitem()
        # extract values for clarity
        curNeighbors = curVertex[1][1].copy()
        # Add all neighbors to cover
        for node in curNeighbors:
            # add label to cover set
            coverSet.append(node)
            # pop that node from queue
            tempNode = q.pop(node)
            # add that nodes unique edges to covered set
            coveredEdges = coveredEdges + tempNode[0]
            # remove edges from target nodes
            for subNode in tempNode[1]:
                # update number of nodes
                q[subNode][0] = q[subNode][0] - 1
                # remove origin node from neighbors
                q[subNode][1].remove(node)
        # remove original vertex
        q.pop(curVertex[0])
    return coverSet

# depreciated
def _solveApprox_nofile_withcheck(q,nNodes,nEdges):
    # while graph not covered
    coveredEdges = 0
    coverSet = []
    while coveredEdges < nEdges and len(q)>0:
        # select vertex of minimum degree
        curVertex = q.peekitem()
        curNeighbors = curVertex[1][1].copy()
        # Add all neighbors to cover
        for node in curNeighbors:
            if node in q:
                # add label to cover set
                coverSet.append(node)
                # pop that node from queue
                tempNode = q.pop(node)
                # add that nodes unique edges to covered set
                coveredEdges = coveredEdges + tempNode[0]
                # remove edges from target nodes
                for subNode in tempNode[1]:
                    if node in q:
                        # update number of nodes
                        q[subNode][0] = q[subNode][0] - 1
                        # remove origin node from neighbors
                        q[subNode][1].remove(node)
        # remove original vertex
        q.pop(curVertex[0])
    if coveredEdges >= nEdges:
        return coverSet
    else:
        return [-1]

# main (for debug)
if __name__ == "__main__":
    testDict = dict()
    # _solveApprox('./DATA-1/dummy1.graph','./test.out',testDict)
    #coverSet = _solveApprox('./DATA-1/karate.graph','./test.out',testDict)
    #coverSet = _solveApprox('./DATA-1/jazz.graph','./test.out',testDict)
    #coverSet =  _solveApprox('./DATA-1/email.graph','./test.out',testDict)
    coverSet = _solveApprox('./DATA-1/star2.graph','./test.out',testDict)
    
    print(len(coverSet))