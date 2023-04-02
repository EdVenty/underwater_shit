from json import load
from typing import Iterable
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch
import torchvision
import numpy as np
import cv2
# import plotly.express as px
import torchvision.transforms as transforms
from torch.autograd import Variable
from torch import Tensor
# import pandas as pd
import os

DST_FOLDER = "D:/dst"

train_progress = []
n_epochs = 15
batch_size_train = 64
batch_size_test = 1000
learning_rate = 0.0001
momentum = 0.5
log_interval = 2
loss_func = F.mse_loss

random_seed = 1
torch.backends.cudnn.enabled = False

# gr = px.line()

def max_indx(array: Iterable):
    max_ = None
    i_ = None
    for (i, el) in enumerate(array):
        if max_ is None: max_ = el; i_ = i; continue
        if el > max_: max_ = el; i_ = i; continue
    return (i_, max_)



# train_loader = torch.utils.data.DataLoader(
#   torchvision.datasets.MNIST('./files/', train=True, download=True,
#                              transform=torchvision.transforms.Compose([
#                                torchvision.transforms.ToTensor(),
#                                torchvision.transforms.Normalize(
#                                  (0.1307,), (0.3081,))
#                              ])),
#   batch_size=batch_size_train, shuffle=True)

# test_loader = torch.utils.data.DataLoader(
#   torchvision.datasets.MNIST('./files/', train=False, download=True,
#                              transform=torchvision.transforms.Compose([
#                                torchvision.transforms.ToTensor(),
#                                torchvision.transforms.Normalize(
#                                  (0.1307,), (0.3081,))
#                              ])),
#   batch_size=batch_size_test, shuffle=True)
# train = pd.read_csv('ARIAL.csv')
# train_loader = torch.tensor(train.to_numpy())

# class CustomDataset(torch.utils.data.Dataset):
#     def __init__(self, csv_path, images_folder, transform = None):
#         self.df = pd.read_csv(csv_path)
#         self.images_folder = images_folder
#         self.transform = transform
#         self.class2index = {"cat":0, "dog":1}

#     def __len__(self):
#         return len(self.df)
#     def __getitem__(self, index):
#         filename = self.df[index, "FILENAME"]
#         label = self.class2index[self.df[index, "LABEL"]]
#         image = PIL.Image.open(os.path.join(self.images_folder, filename))
#         if self.transform is not None:
#             image = self.transform(image)
#         return image, label

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 16, 5)
        self.conv2 = nn.Conv2d(16, 32, 5)
        self.conv3 = nn.Conv2d(32, 64, 5)
        self.conv2_drop = nn.Dropout2d(p=0.2)
        self.conv3_drop = nn.Dropout2d(p=0.2)

        self.fc1 = nn.Linear(2304, 1280)
        self.fc2 = nn.Linear(1280, 512)
        self.fc3 = nn.Linear(512, 256)
        self.fc4 = nn.Linear(256, 9) #9 umbers

    def forward(self, x):
        x = F.relu(F.max_pool2d(self.conv1(x), 2))
        x = F.relu(F.max_pool2d(self.conv2_drop(self.conv2(x)), 4))
        x = F.relu(F.max_pool2d(self.conv3_drop(self.conv3(x)), 2))
        
        # print(x.shape)
        # x = x.flatten().unsqueeze(0)
        x = x.view(-1, 2304)
        # print(x.shape)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        x = self.fc3(x)
        x = F.dropout(x, training=self.training)
        # x = self.fc3(x)
        x = self.fc4(x)
        # return torch.log_softmax(x)
        return x
def train(epoch):
    net.train()
    for batch_idx, (data, target) in enumerate(train_loader):
        frame = data[0].transpose(2, 0).detach().numpy()
        draw = frame.copy() * 255
        # draw = cv2.cvtColor(draw[0], cv2.COLOR_GRAY2BGR)
        # draw = cv2.rotate(draw, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # draw = cv2.flip(draw, 0)

        output = net(data)

        need = max_indx(list(target[0].detach().numpy()))[0] + 1
        guess = max_indx(list(output[0].detach().numpy()))[0] + 1
        cv2.putText(draw, str(need), (10, 40), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (255, 0, 0))
        cv2.putText(draw, str(guess), (10, 80), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (0, 255, 0))
        # i = 10
        if need == guess:
            cv2.rectangle(draw, (10, 80), (20, 90), (0, 0, 255), -1)
        # guessed = 0
        # for (t, o) in zip(target, output):
        #     if max_indx(t.detach().numpy())[0] == max_indx(o.detach().numpy())[0]:
        #         guessed += 1
        # cv2_imshow(draw)
        # cv2.waitKey(1)
        # cv2_imshow(draw)
        # train_progress.append({
        #     "epoch": epoch, 
        #     "guessed": guessed, 
        #     "batch index": batch_idx
        #     })

        optimizer.zero_grad()
        loss = loss_func(output, target)
        loss.backward()
        optimizer.step()
        if batch_idx  % log_interval == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                epoch, batch_idx * len(data), len(train_loader.dataset),
                100. * batch_idx / len(train_loader), loss.item())
                )
            train_losses.append(loss.item())
            train_counter.append(
                (batch_idx*64) + ((epoch-1)*len(train_loader.dataset)))
            torch.save(net.state_dict(), './drive/MyDrive/Models/model.pth')
            torch.save(optimizer.state_dict(), './drive/MyDrive/Models/optimizer.pth')

    # test_loss /= len(test_loader.dataset)
    # test_losses.append(test_loss)
    # print('\nTest set: Avg. loss: {:.4f}, Accuracy: {}/{} ({:.0f}%)\n'.format(
    #     test_loss, correct, len(test_loader.dataset),
    #     100. * correct / len(test_loader.dataset)))
    # return 100. * correct / len(test_loader.dataset)

def detect(frame):
    global net
    net.eval()
    # img_transforms=transforms.Compose([transforms.Resize((imh,imw)),
    #      transforms.Pad((max(int((imh-imw)/2),0), 
    #           max(int((imw-imh)/2),0), max(int((imh-imw)/2),0),
    #           max(int((imw-imh)/2),0)), (128,128,128)),
    #      transforms.ToTensor(),
    #      ])
    # image_tensor = img_transforms(frame).float()
    # image_tensor = image_tensor.unsqueeze_(0)
    # input_img = Variable(image_tensor.type(Tensor))
    # im_np = np.asarray(frame)
    with torch.no_grad():
        # .transpose(2,0)
        frame = cv2.resize(frame, (192, 108))
        output = net(torch.FloatTensor(frame).transpose(2,0).unsqueeze(0))
        return output



# progress = []



# l = px.line(progress, "Epoch", "Accurancy", title="Neuro Network Deep Learning Progress")
# l.show()
    

net = Net()
net.load_state_dict(torch.load("./results/model.pth"))

if __name__ == "__main__":
    
    # net.load_state_dict(torch.load("./results/model.pth"))
    # optimizer = optim.SGD(net.parameters(), learning_rate, momentum)
    optimizer = optim.Adam(net.parameters(), learning_rate)

    # test()
    progress = []
    for file in os.listdir(DST_FOLDER):
        train_loader = torch.utils.data.DataLoader(torch.load(f"{DST_FOLDER}/{file}"), batch_size=batch_size_train, shuffle=True)
        # # training 
        train_losses = []
        train_counter = []
        test_losses = []
        test_counter = [i*len(train_loader.dataset) for i in range(n_epochs + 1)]
        # test()
        for epoch in range(1, n_epochs + 1):
            train(epoch)
            # percentage = test()
            # progress.append({"Epoch": epoch, "Accurancy": percentage})
    # px.line_3d(train_progress, "epoch", "guessed", "batch index").show()
    # px.bar(progress, )

