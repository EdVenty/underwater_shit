import digit_recognition
import torch
from torch.utils.data import DataLoader

train_loader = DataLoader(torch.load(f"D:/dst/0.dst"), batch_size=64, shuffle=True)

with open()
nn = digit_recognition.NeuralNetwork([62208, 20, 5])
nn.
nn.train(train_loader.dataset.tensors[0], train_loader.dataset.tensors[1], 50)

nn.save("digimodel")
test_loader = DataLoader(torch.load(f"D:/dst/1.dst"), batch_size=64, shuffle=True)

nn.evaluate(test_loader.dataset.tensors[0], test_loader.dataset.tensors[1])