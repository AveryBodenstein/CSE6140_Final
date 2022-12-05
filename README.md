# CSE6140_Final
Georgia Tech CSE6140 Final Project Repository

This project was a collaboration between:

Avery Bodenstein, Adrian Thinnyun, Jai Jacob, Zheyi Zhang

To run any .graph dataset use the command:

"python template.py -inst <inputFile.graph> -alg <selectedAlgorithm> -time <timeLimit> - seed <randSeed> -out <outputDir>"

this assumes you are running python 3.10+ with deap installed (pip install deap)

where <selectedAlgorithm> can be one of [BnB|Approx|LS1|LS2|BnB_r4]
outputDir is optional, if not specified the current working directory is used.

template.py launches each algorithm with the parameters specified. The algorithms are broken into four main files:
- ApproxSolver.py : Implements Greedy Independent Cover heuristic algorithm
- BnBSolver_r2.py : Implements (single threaded) Branch and Bound algorithm
- BnBSolver_r4.py : Implements (multi threaded) Branch and Bound algorithm
- LS1Solver.py : Implements Simulated Annealing
- LS2Solver.py : Implements Genetic Algorithm

There are also several helper/library files
- commonLib.py : Implements helper functions for file import/export 
- heapdict.py : Implements priority queue dictionary data structure (NOTE! This is not the standard implementation of this library function and has been                   modified for this application!)
- stats.py : creates run performance graphs for Local Search algorithms

Note that there are also several shell script files
- runApprox.sh : runs all graph files using ApproxSolver.py
- runBnB.sh : runs all graph files using BnBSolver_r2.py
- runBnB_r4.sh : runs all graph files using BnBSolver_r4.py (multithreaded BnB solver)
(these may need to be given execute permission on your machine using chmod +x <*.sh>

