#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 2022

@author: Avery Bodenstein

This main function is intended to implement the overall interface to all underlying vertex cover solvers
"""

import sys, argparse
from pathlib import Path

#   Import solvers
from BnBsolver_r2 import solveBnB
from BnBsolver_r4 import solveBnB as solveBnB4
from ApproxSolver import solveApprox
from LS1solver import solveLS1
from LS2solver import solveLS2


if __name__ == '__main__':
    inputFile = ''

    # Create input argument parser
    parser = argparse.ArgumentParser(description='Evaluate Minimum Vertex Cover on Metis IO graph.')
    parser.add_argument('-inst', help='Input Metis IO graph file')
    parser.add_argument('-alg', help='Algorithm [BnB|Approx|LS1|LS2]',choices=['BnB','Approx','LS1','LS2','BnB_r4'])
    parser.add_argument('-time', help='Maximum running time (sec)')
    parser.add_argument('-seed', help='Random seed initialization')
    parser.add_argument('-out', help='Output File Directory (Optional)')

    # Parse Arguments
    args = parser.parse_args()

    # Create output file
    if args.out is not None:
        outDir = args.out
    else:
        outDir = './'
        
    # remove filepath and extension from ouput file
    inst = args.inst.split('/')
    inst = inst[-1].split('.')
    inst = inst[0]
    # if algorithm is deterministic don't add seed to filename
    if args.alg in ['BnB','Approx']:
        outputFile = f"{outDir}/{inst}_{args.alg}_{args.time}"
    else:
        outputFile = f"{outDir}/{inst}_{args.alg}_{args.time}_{args.seed}"
        initSeed = int(args.seed)

    # Assign variables for clarity 
    algorithmStr = args.alg
    maxTime = float(args.time)
    inputFile = args.inst

    # Pass graph problem to specific solver
    if algorithmStr == 'BnB':
        solveBnB(inputFile,outputFile,maxTime)
    elif algorithmStr == 'Approx':
        solveApprox(inputFile,outputFile,maxTime)
    elif algorithmStr == 'LS1':
        solveLS1(inputFile,outputFile,maxTime,initSeed)
    elif algorithmStr == 'LS2':
        solveLS2(inputFile,outputFile,maxTime,initSeed)
    elif algorithmStr == 'BnB_r4':
        solveBnB4(inputFile,outputFile,maxTime)


