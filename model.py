import matplotlib.pylab as plt
import pandas as pd
import numpy as np
import json
import cv2
import os
from PIL import Image


import torch
import torch.nn as nn

from torch.utils.data import random_split
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
from torchvision.transforms import ToTensor
from torch.optim import Adam


# Read CSV file from the URL and print out the first five samples
directory = "results/prediction_comparisons/points_model/second"
csv_train = 'mycsvs/train.csv'
csv_test = 'mycsvs/test.csv'


class Dataset(Dataset):
    # Constructor
    def __init__(self, csv_file, data_dir):
        # path to the data
        self.data_dir = data_dir

        # Load the CSV file contians image info
        self.data_csv = pd.read_csv(os.path.join(self.data_dir,csv_file))

        # Number of images in dataset
        self.len=self.data_csv.shape[0]

    # Get the length
    def __len__(self):
        return self.len

    # Getter
    def __getitem__(self, idx):
        # Image file path
        image_name=os.path.join(self.data_dir, self.data_csv.iloc[idx, 6])

        # Load image
        image = cv2.imread(image_name)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image.astype('float32') / 255
        image = image.reshape((3, 512, 512))
        image = torch.from_numpy(image)

        # The target for the image
        x = self.data_csv.iloc[idx, 2] / 512
        y = self.data_csv.iloc[idx, 3] / 512
        target = [x, y]
        target = np.array(target)
        target = torch.from_numpy(target)



        #start/goal
        isstart = self.data_csv.iloc[idx, 7]
        isgoal = self.data_csv.iloc[idx, 8]

        return image, target, isstart, isgoal



dataset = Dataset(csv_file=csv_train, data_dir="")


with torch.cuda.device('cuda:11'):
    torch.cuda.empty_cache()

# set the device we will be using to train the model
device = torch.device("cuda:11" if torch.cuda.is_available() else "cpu")



# define training hyperparameters
INIT_LR = 1e-4
BATCH_SIZE = 64
EPOCHS = 60


# define the train and val splits
TRAIN_SPLIT = 0.6
VAL_SPLIT = 0.2


# calculate the train/validation split
print("[INFO] generating the train/validation split...")
numTrainSamples = int(len(dataset) * TRAIN_SPLIT)
numValSamples = int((len(dataset) - numTrainSamples)/2)
other = int(len(dataset) - numTrainSamples - numValSamples)


trainData, valData, other_data = torch.utils.data.random_split(dataset, [numTrainSamples, numValSamples, other])
testData = Dataset(csv_file=csv_train, data_dir="")


# initialize the train, validation, and test data loaders
trainDataLoader = DataLoader(trainData, shuffle=True, batch_size=BATCH_SIZE, num_workers = 2)
trainDataLoader1 = DataLoader(trainData, shuffle=True, batch_size=1, num_workers = 2)
valDataLoader = DataLoader(valData, shuffle = True, batch_size=BATCH_SIZE*2, num_workers = 2)
valDataLoader1 = DataLoader(valData, shuffle = True, batch_size=1, num_workers = 2)



#testDataLoader1 = DataLoader(testData, batch_size = BATCH_SIZE*2, num_workers = 2)
testDataLoader2 = DataLoader(testData, batch_size = 1)


# calculate steps per epoch for training and validation set
trainSteps = len(trainDataLoader.dataset) // BATCH_SIZE
valSteps = len(valDataLoader.dataset) // (BATCH_SIZE*2)
testSteps = len(testDataLoader2.dataset)



from torch.nn import Module
from torch.nn import Conv2d
from torch.nn import Linear
from torch.nn import MaxPool2d
from torch.nn import ReLU
from torch.nn import LogSoftmax
from torch import flatten

class RRTNet(Module):
    def __init__(self, numChannels, output_size):
        # call the parent constructor
        super(RRTNet, self).__init__()

        # initialize first set of CONV => RELU => POOL layers
        self.conv1 = Conv2d(in_channels=numChannels, out_channels=16,
                            kernel_size=(5, 5))
        self.relu1 = ReLU()
        self.maxpool1 = MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
        self.conv1_bn = nn.BatchNorm2d(16)

        # initialize second set of CONV => RELU => POOL layers
        self.conv2 = Conv2d(in_channels=16, out_channels=16,
                            kernel_size=(5, 5))
        self.relu2 = ReLU()
        self.maxpool2 = MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
        self.conv2_bn = nn.BatchNorm2d(16)

        # initialize third set of CONV => RELU => POOL layers
        self.conv3 = Conv2d(in_channels=16, out_channels=16,
                            kernel_size=(5, 5))
        self.relu3 = ReLU()
        self.maxpool3 = MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
        self.conv3_bn = nn.BatchNorm2d(16)

        # initialize fourth set of CONV => RELU => POOL layers
        self.conv4 = Conv2d(in_channels=16, out_channels=16,
                            kernel_size=(5, 5))
        self.relu4 = ReLU()
        self.maxpool4 = MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
        self.conv4_bn = nn.BatchNorm2d(16)

        # initialize first (and only) set of FC => RELU layers
        self.fc1 = Linear(in_features=12544, out_features=500)
        self.relu5 = ReLU()

        # initialize our softmax classifier
        self.fc2 = Linear(in_features=500, out_features=output_size)


        self.droput = nn.Dropout(p=0.5, inplace=True)

    def forward(self, x):
        # pass the input through our first set of CONV => RELU =>
        # POOL layers
        x = self.conv1(x)
        x = self.relu1(x)
        x = self.conv1_bn(x)
        x = self.maxpool1(x)


        # pass the output from the previous layer through the second
        # set of CONV => RELU => POOL layers
        x = self.conv2(x)
        x = self.relu2(x)
        x = self.conv2_bn(x)
        x = self.maxpool2(x)


        # pass the output from the previous layer through the third
        # set of CONV => RELU => POOL layers
        x = self.conv3(x)
        x = self.relu3(x)
        x = self.conv3_bn(x)
        x = self.maxpool3(x)


        # pass the output from the previous layer through the fourth
        # set of CONV => RELU => Batch Norm => POOL layers
        x = self.conv4(x)
        x = self.relu4(x)
        x = self.conv4_bn(x)
        x = self.maxpool4(x)

        x = self.droput(x)

        # flatten the output from the previous layer and pass it
        # through our only set of FC => RELU layers
        x = flatten(x, 1)
        x = self.fc1(x)
        x = self.relu5(x)

        # pass the output to our softmax classifier to get our output
        # predictions
        output = self.fc2(x)

        # return the output predictions
        return output




print("[INFO] initializing the RRTNet model...")
model = RRTNet(numChannels=3, output_size=2).to(device)
print(device)


# initialize our optimizer and loss function
opt = Adam(model.parameters(), lr=INIT_LR)
lossFn = nn.MSELoss()


# initialize a dictionary to store training history
H = {"train_loss": [], "val_loss": []}

for e in range(0, EPOCHS):

    model.train()

    totalTrainLoss = 0
    totalValLoss = 0


    for (x, y, isstart, isgoal) in trainDataLoader:

        (x, y) = (x.to(device), y.to(device))
        #x = x.float()
        y = y.float()
        #print(y)
        pred = model(x)
        loss = lossFn(pred, y)

        opt.zero_grad()
        loss.backward()
        opt.step()

        totalTrainLoss += loss

    with torch.no_grad():
        model.eval()

        for (x, y, isstart, isgoal) in valDataLoader:

            (x, y) = (x.to(device), y.to(device))
            #x = x.float()
            y = y.float()

            pred = model(x)
            totalValLoss += lossFn(pred, y)

    # calculate the average training and validation loss
    avgTrainLoss = totalTrainLoss / trainSteps
    avgValLoss = totalValLoss / valSteps

    # update our training history
    H["train_loss"].append(avgTrainLoss.cpu().detach().numpy())
    H["val_loss"].append(avgValLoss.cpu().detach().numpy())

    # print the model training and validation information
    print("[INFO] EPOCH: {}/{}".format(e + 1, EPOCHS))
    print("Train loss: {:.6f}, Val loss: {:.6f}".format(avgTrainLoss, avgValLoss))


plt.style.use("ggplot")
plt.figure()
plt.plot(H["train_loss"], label="train_loss")
plt.plot(H["val_loss"], label="val_loss")
plt.title("Training and validation lossess")
plt.xlabel("Epoch #")
plt.ylabel("Loss")
plt.legend(loc="upper right")

plt.savefig(os.path.join(directory,"train_val_loss.png"))


torch.save(model.state_dict(), os.path.join(directory,'rrt_point1') )



def euclidean(predicted, true):
    true = true.tolist()[0]
    predicted = predicted.tolist()[0]

    error = ((true[0] - predicted[0])**2 + (true[1]- predicted[1])**2)**(0.5)
    #print(error)
    return error




i1 = 0
error_train = 0.0
with torch.no_grad():
    # set the model in evaluation mode
    model.eval()

    for (x, y, isstart, isgoal) in trainDataLoader1:

        (x, y) = (x.to(device), y.to(device))
        #x = x.float()
        y = y.float()
        i1+=1
        z = model(x)



        error_train += euclidean(z,y)

        if i1%125 == 0:
            print("Error -----"+str(i1)+"---", error_train)

ave_error_train = error_train/numTrainSamples
print("Final train error ---------", ave_error_train)


i2 = 0
error_val = 0.0
with torch.no_grad():
    # set the model in evaluation mode
    model.eval()

    for (x, y, isstart, isgoal) in valDataLoader1:

        (x, y) = (x.to(device), y.to(device))
        #x = x.float()
        i2+=1
        y = y.float()
        z = model(x)




        if i2%50 == 0:
            print("Error -----"+str(i2)+"---", error_train)

        error_val += euclidean(z,y)

ave_error_val = error_val/numValSamples
print("Final validation error ---------", ave_error_val)



i3 = 0
error_test = 0.0
with torch.no_grad():
    # set the model in evaluation mode
    model.eval()

    for (x, y, isstart, isgoal) in testDataLoader2:

        (x, y) = (x.to(device), y.to(device))
        #x = x.float()
        y = y.float()
        z = model(x)
        i3 += 1

        error_test += euclidean(z,y)
        if i3%50 == 0:
            print("Error ----"+str(i3)+"----", error_test)

ave_error_test = error_test/testSteps
print("Final test error ---------", ave_error_test)



parametric_data = {'train_error': ave_error_train, 'val_error': ave_error_val, 'test_error': ave_error_test,}


with open(os.path.join(directory,'errors.json'), 'w') as f:
                        json.dump(parametric_data, f)


def drawpredictions(canvas, predictions):
    for i in range(len(predictions)):
        point = (round(predictions[i][0]*512), round(predictions[i][1]*512))
        cv2.circle(canvas, point, 15, (255, 255, 0), -1, 8, 0)


    return canvas



def predict(model, device, testDataloader2):
    predictions = []
    currents = []
    images = []
    image_counter = 0
    with torch.no_grad():
        # set the model in evaluation mode
        model.eval()

        for (x, y, isstart, isgoal) in testDataloader2:
            # send the input to the device
            (x, y) = (x.to(device), y.to(device))
            x = x.float()
            y = y.float()
            #print(current)
            z = model(x)
            if isstart.tolist()[0] == 1:
                predictions = []
                currents = []
            #print(z.tolist()[0])
            pred = z.tolist()[0]

            predictions.append(pred)

            if isgoal.tolist()[0] == 1:
                image = x.cpu().numpy().reshape((512,512,3))
                image = image * 255
                image = image.astype('uint8')
                image = drawpredictions(image, predictions)
                plt.imshow(image,cmap='gray', vmin=0, vmax=255)
                plt.show()
                image = Image.fromarray(image, 'RGB')

                image.save(os.path.join(directory,'testing_images/img'+str(image_counter+1000000)+'.png') )
                #print(image_counter)
                image_counter += 1



predict(model, device, testDataLoader2)
