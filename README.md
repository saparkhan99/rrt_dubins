# rrt_dubins
The files that consist of the code that create the shortest path from start to goal position using Dubins-Based RRT* algorithm with random angles of going into the nodes.



# path_finder.py 
Creates dubins-based paths and stops running after the first path is created.
![image](https://user-images.githubusercontent.com/47181212/231216252-dbfe68a0-dbc1-4999-8435-cb6c5439a7d0.png)

The first green circle is the start position.

The hollow green circle is the goal position.

The gray square blocks are the obstacles.

The red path is the generaetd path from start to the goal.

# path_with_robot.py
Creates the path and show the movement of the animated car along the generated path.

# paths_many.py 
Creates multiple paths, compares their distance and chooses the snallest one. 
