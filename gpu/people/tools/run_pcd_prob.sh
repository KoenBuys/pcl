#!/bin/bash
#/data/svn/pcl/trunk/build/bin/pcl_people_pcd_prob -numTrees 3 -tree0 /data/results/forest1/tree_20.txt -tree1 /data/results/forest2/tree_20.txt -tree2 /data/results/forest3/tree_20.txt -tree3 /data/results/forest3/tree_20.txt -mask 0 -FG 0 -pcd $1

/data/git/pcl/build/bin/pcl_people_pcd_prob -numTrees 3 -tree0 /data/results/forest1/tree_20.txt -tree1 /data/results/forest2/tree_20.txt -tree2 /data/results/forest3/tree_20.txt -tree3 /data/results/forest3/tree_20.txt -pcd /data/pcd/koen.pcd -XML /data/git/pcl/gpu/people/data/generic.xml -prob 1

