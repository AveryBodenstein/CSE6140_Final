#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 27 15:32:56 2022

@author: Avery Bodenstein

library of common functions to be used across all solvers
"""

import heapdict as hd

# depreciated
def parse_graph(filename):
    # opens .graph file and parses contents
    with open(filename, 'r') as graphFile:
        # Parse first line as nNodes, nEdges
        tempLine = graphFile.readline()
        graph_data = list(map(lambda x: int(x), tempLine.split()))
        assert(len(graph_data) == 3)
        nNodes, nEdges, weighted = graph_data[0], graph_data[1], graph_data[2]

        # Create empty list to hold values
        adjacent = [[]]*nNodes
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
            adjacent[ii] = edge_data
            nodeID[ii+1] = nodeID[ii] + nodeEdges[ii]
            nodeLabel[ii] = ii+1
        # remove extra value from nodeID list
        del(nodeID[-1])
        # sort by number of edges
        test = sorted(zip(nodeEdges, nodeLabel, adjacent), key=lambda x: x[0],reverse=True)
        # 'unzip'
        nodeEdges, nodeLabel, adjacent = zip(*test)
        nodeEdges = list(nodeEdges)
        nodeLabel = list(nodeLabel)
        adjacent = list(adjacent)
        
    return adjacent,nodeEdges,nodeLabel,nNodes,nEdges

# returns a priority queue instead of sorted lists
def parse_graph_hd(filename):
    # opens .graph file and parses contents
    with open(filename, 'r') as graphFile:
        # Parse first line as nNodes, nEdges
        tempLine = graphFile.readline()
        graph_data = list(map(lambda x: int(x), tempLine.split()))
        assert(len(graph_data) == 3)
        nNodes, nEdges, weighted = graph_data[0], graph_data[1], graph_data[2]

        # Create empty priority queue to hold values
        q = hd.heapdict()

        # Parse remainder of file
        for ii in range(0,nNodes):
            tempLine = graphFile.readline()
            # parse edge and weight
            edge_data = list(map(lambda x: int(x), tempLine.split()))
            
            q[ii+1] = [len(edge_data),edge_data]
        
    return q,nNodes,nEdges

# writes solution to file
def write_sol(filename, coverSet):
    with open(f"{filename}.sol",'w') as outFile:
        # convert list to comma deliminated string
        s = ",".join(map(str, coverSet))
        outFile.write(f"{len(coverSet)}\n{s}")

# logs current best solution
def log_sol(filename, quality, timestamp):
    with open(f"{filename}.trace",'a') as outFile:
        outFile.write(f"{timestamp:0.4f},{quality}\n")
    
    
    
    
    