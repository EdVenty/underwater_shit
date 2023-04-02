import cv2
import os
import torch
import numpy as np
import json
from torch.utils import data
from random import shuffle

LABELS_FILE = "D:/videos/labels.json"
VIDEOS_FOLDER = "D:/videos"
DATASET_FOLDER = "D:/dst"
CHUCK_SIZE = 500
SHUFFLE_IMAGES = True

HSV_MIN = ()
HSV_MAX = ()

随机的 = []
chunk_labels = []
chunk_number = 0
size_printed = False
dataset_raw = []

with open(LABELS_FILE) as file:
    labels = json.loads(file.read())

objects = (os.listdir(VIDEOS_FOLDER))
if SHUFFLE_IMAGES:
    shuffle(objects)
    print("Videos shuffled")

for file in objects:
    if file == "labels.json": continue
    video = cv2.VideoCapture(f"{VIDEOS_FOLDER.strip('/')}/{file}")
    while(video.isOpened()):
        r, frame = video.read()
        if not r: break
        w, h, c = frame.shape
        hui = cv2.resize(frame, (h // 5, w // 5))
        # cv2.imshow("a", hui)
        # cv2.waitKey(1)
        if not size_printed:
            print(f"Size is w: {w // 5}, h: {h // 5}")
            print(f"real w: {w}, real h: {h}")
            size_printed = True
        随机的.append([cv2.inRange(hui, HSV_MIN, HSV_MAX), labels[file]])
        # 随机的.append([np.transpose(hui, [2, 0, 1]), labels[file]])
    print(f"Video {file} loaded")

i = 0
n_chunk = []
shuffle(随机的)
print("Chunks shuffled")
for c in 随机的:
    if i < len(随机的):
        n_chunk.append(c)
        if (i % CHUCK_SIZE == 0 and i != 0) or len(随机的) == i - 1:
            np_ar = np.array(n_chunk)
            t = torch.Tensor(list(np_ar[:, 0])) / 255.
            l = torch.Tensor(list(np_ar[:, 1]))
            # t = torch.Tensor(np_ar[:, 0]) / 255.
            # l = torch.Tensor(np_ar[:, 1])
            dataset = data.TensorDataset(t, l)
            torch.save(dataset, f"{DATASET_FOLDER.strip('/')}/{chunk_number}.dst")
            n_chunk = []
            print(f"Chunk {chunk_number} saved!")
            chunk_number += 1
    i += 1


        