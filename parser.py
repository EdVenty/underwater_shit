import json
import pickle

from torch import nn
import cv2
import numpy as np

with open("via_project_28Apr2021_23h13m34s.json", 'r') as file:
    jsoned = json.loads(file.read())
    ids = jsoned["project"]["vid_list"]
    files = jsoned["file"]
    md: dict = jsoned["metadata"]
    sorted_md_keys = sorted(list(md.keys()), key=lambda x: int(x.split("_")[0]))
    out1 = [] # пути файлов
    out2 = []
    images = []
    # out = {}
    # for (id_, md_el) in zip(ids, sorted_md_keys):
    #     out.update({id_: [files[id_]["fname"], md[md_el]['av']['1']]})
    for (id_, md_el) in zip(ids, sorted_md_keys):
        out1.append(files[id_]["fname"])
        m = [0 for i in range(5)]
        m[int(md[md_el]['av']['1'])] = 1
        out2.append(m)
        img = cv2.imread(f"D:/Downloads/files/{files[id_]['fname']}")
        
        w, h, _ = img.shape
        img = cv2.resize(img, (w // 24, h // 24))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 27, 10)
        cv2.imshow("img", img)
        cv2.waitKey(1)
        # print(img.shape)
        # hui = np.transpose([img], [2, 0, 1])
        hui = [img]
        # print(hui.shape)
        images.append(hui)

    with open("keys.torch", 'wb') as file2:
        file2.write(pickle.dumps([images, out2]))
import torch
import torch.nn as nn
import torch.functional as F
from torch.utils import data

t1 = torch.Tensor(images) / 255.
t2 = torch.Tensor(out2)

dataset = data.TensorDataset(t1, t2)
torch.save(dataset, "nums.egor")
