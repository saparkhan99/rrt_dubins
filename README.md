# rrt_dubins
The files that consist of the code that create the shortest path from start to goal position using Dubins-Based RRT* algorithm with random angles of going into the nodes.



# path_finder.py 
Creates dubins-based paths and stops running after the first path is created.
![image](https://user-images.githubusercontent.com/47181212/231216252-dbfe68a0-dbc1-4999-8435-cb6c5439a7d0.png)

Explanation:

1) The first green circle is the start position.

2) The hollow green circle is the goal position.

3) The gray square blocks are the obstacles.

4) The blue lines are the randomly sampled paths.

5) The red path is the sub-optimal generaetd path from start to the goal.

# path_with_robot.py
Creates the path and show the movement of the animated car along the generated path.

![image](https://user-images.githubusercontent.com/47181212/231217619-ffbc47ec-913b-405d-af69-6584810b7448.png)

Explanation:

The same as above but with a car going along the generated red path.

# generate_path.py

Generate paths for the training of neural network.

# paths_many.py 
Creates multiple paths, compares their distance and chooses the smallest one. 
