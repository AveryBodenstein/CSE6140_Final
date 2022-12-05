#!/bin/bash


graphFiles=`ls ../DATA-1/ | grep .graph`

for graph in ${graphFiles}
do
	echo ${graph}
	python template.py -inst ../DATA-1/${graph} -alg BnB_r4 -time 500 -seed 7 -out ../output

done
