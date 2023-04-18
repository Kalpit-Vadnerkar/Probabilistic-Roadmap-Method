# Probabilistic-Roadmap-Method
Construction Phase:
After some trail and errors i decided to sample 1000 configurations as that seemed to always result in a well connected graph all over the evironment. 
10 nearest neighbors was the best choice for fast building of the graph.
For the distance function, i weighted 0.8 for the euclidean distance and 0.2 weight for rotation distance as i think it should be easier to rotate compared to translate for the robot.
Query Phase:
The query phase was relatively easy to implement once the graph was ready.