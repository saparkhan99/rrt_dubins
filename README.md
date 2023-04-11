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

The code that generates ground true dataset for the neural network to learn. This script generates a dataset only with usual RRT* paths without dubins motion.
![photo_2023-04-11_22-04-16](https://user-images.githubusercontent.com/47181212/231222454-6582472e-b9ab-441d-af33-35816acac147.jpg)

Explanation:
The image for learning differs from the image before. It is done for the model to better learn the fetures.
1) Blue circle is the start position.
2) Green circle is the goal position
3) White square are the obstacles
4) Red circle are the nodes for of the generarted paths


# paths_many.py 
Creates multiple paths, compares their distance and chooses the smallest one. 
