import json
import pickle
from test import Net
from typing import Iterable, Tuple

import cv2
from torch import nn
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.functional as F
from torch.utils import data


net = Net()
net.load_state_dict(torch.load('./results/model.pth'))
# cum = cv2.VideoCapture(1)

def max_indx(array: Iterable):
    max_ = None
    i_ = None
    for (i, el) in enumerate(array):
        if max_ is None: max_ = el; i_ = i; continue
        if el > max_: max_ = el; i_ = i; continue
    return (i_, max_)
cum = cv2.VideoCapture("http://192.168.43.169:8080/video")
while True:
    
    r, img = cum.read()
    r = True
    # img = cv2.imread("D:/Downloads/drive-download-20210428T124617Z-001/4_original_2_1619610787390_false_Mi 9T.png")
    assert r, "Your ass wasn't captured"

    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    w, h, _ = img.shape
    # img = img[:192, :108, :] 
    img = cv2.resize(img, (108, 192))
    # print(img.shape)
    # img = cv2.resize(img, (166, 125))
    # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    # img = cv2.resize(img, (108, 192))
    t = torch.tensor(np.transpose(img, [2, 0, 1])).unsqueeze(0) # не нада ансквиз
    # print(t.detach().numpy().shape)
    # cv2.imshow("ggvp", t.squeeze(0).transpose(2, 0).detach().numpy())
    # print(t.shape)
    anus = net(t / 255.)
    print(max_indx(anus.detach().numpy()[0])[0] + 1)
    # print(t[0].transpose(2, 0).detach().numpy().shape)
    cv2.imshow("asus", t[0].transpose(2, 0).detach().numpy())
    cv2.waitKey(1)