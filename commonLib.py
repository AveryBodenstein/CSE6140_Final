#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 27 15:32:56 2022

@author: Avery Bodenstein
"""

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