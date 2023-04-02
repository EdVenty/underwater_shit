import sys

sys.path = ['', '/usr/lib/python37.zip', '/usr/lib/python3.7', '/usr/lib/python3.7/lib-dynload', '/home/pi/.local/lib/python3.7/site-packages', '/usr/local/lib/python3.7/dist-packages', '/usr/local/lib/python3.7/dist-packages/pyzmq-22.0.3-py3.7-linux-armv7l.egg', '/usr/lib/python3/dist-packages']

from typing import Iterable
import cv2
from torch import nn
import pymurapi as mur
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.functional as F

HSV_MIN = (0, 0, 0)
HSV_MAX = (180, 30, 255)

VIDEO_ID = 1
auv = mur.mur_init()
mur_view = auv.get_videoserver()

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 16, 5, 2)
        self.conv2 = nn.Conv2d(16, 32, 3, 1)
        self.conv3 = nn.Conv2d(32, 64, 3, 2)
        self.conv2_drop = nn.Dropout2d(p=0.2)
        self.conv3_drop = nn.Dropout2d(p=0.2)

        self.fc1 = nn.Linear(64, 1280)
        self.fc2 = nn.Linear(1280, 512)
        self.fc3 = nn.Linear(512, 256)
        self.fc4 = nn.Linear(256, 9)

    def forward(self, x):
        x = torch.relu(torch.max_pool2d(self.conv1(x), 2))
        x = torch.relu(torch.max_pool2d(self.conv2_drop(self.conv2(x)), 4))
        x = torch.relu(torch.max_pool2d(self.conv3_drop(self.conv3(x)), 2))
        
        # print(x.shape)
        # x = x.flatten().unsqueeze(0)
        x = x.view([-1, 64])
        # print(x.shape)
        x = self.fc1(x)
        x = self.fc2(x)
        x = self.fc3(x)
        x = torch.dropout(x, train=self.training, p=0.2)
        # x = self.fc3(x)
        x = self.fc4(x)
        # return torch.log_softmax(x)
        return x

net = Net()
# net.load_state_dict(torch.load('/home/pi/model.pth'))
# cum = cv2.VideoCapture(1)

def max_indx(array: Iterable):
    max_ = None
    i_ = None
    for (i, el) in enumerate(array):
        if max_ is None: max_ = el; i_ = i; continue
        if el > max_: max_ = el; i_ = i; continue
    return (i_, max_)

cam = cv2.VideoCapture(VIDEO_ID)
while True:
    r, img = cam.read()
    # img = cv2.imread("D:/Downloads/drive-download-20210428T124617Z-001/4_original_2_1619610787390_false_Mi 9T.png")
    # assert r, "Your frame wasn't captured"
    if not r:
        exit(0)

    # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWsISE)
    w, h, _ = img.shape
    # img = img[:192, :108, :] 
    img = cv2.resize(img, (128, 96))
    # print(img.shape)
    # img = cv2.resize(img, (166, 125))
    # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    # img = cv2.resize(img, (108, 192))
    t = torch.tensor(np.transpose(np.expand_dims(img, axis=0), [2, 0, 1])).unsqueeze(0) # не нада ансквиз (или нада)
    # print(t.detach().numpy().shape)
    # cv2.imshow("ggvp", t.squeeze(0).transpose(2, 0).detach().numpy())
    # print(t.shape)
    output = net(t / 255.)
    mmx = max_indx(output.detach().numpy()[0])[0] + 1
    print(mmx)
    mur_view.show(img, 0)
    # print(t[0].transpose(2, 0).detach().numpy().shape)
    # cv2.imshow("output", t[0].transpose(2, 0).detach().numpy())
    # cv2.waitKey(1)