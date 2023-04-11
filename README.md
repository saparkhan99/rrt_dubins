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
the same as generate_paths but creates multiple paths, compares their distance and chooses the smallest one. 

# model.py

The code that takes the dataset, trains the model, generates paths using the trained model, and compares it with the test data.

The model losses:
![photo_2023-04-11_22-12-45](https://user-images.githubusercontent.com/47181212/231225424-af6738bf-bfdc-4364-b24d-968dc7285ac9.jpg)

The comparisons: 
![photo_2023-04-11_22-12-49](https://user-images.githubusercontent.com/47181212/231224887-4383dc7e-5612-4cf8-8ab7-6f4f8e03775b.jpg)

![photo_2023-04-11_22-12-40](https://user-images.githubusercontent.com/47181212/231224977-fddd57e2-6932-4927-9533-2422df50f676.jpg)

Yellow circles represent the nodes generated from the trained network.

As we can see, although the losses are good (with a slight overfitting), the model do not generate an obstacle free path when connecting the dots.

# requirement 

You can create a conda environment by running the following code:
conda create --name <env> --file requirements.txt
  
You can also do it by just having the latest version of python and installing the following libraries with the latest versions:
  1. pip install torchvision
  2. pip install opencv-python
  3. pip install pygame
  4. pip install pandas
