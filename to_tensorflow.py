import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.autograd import Variable

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 16, 5, 2)
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
        
trained_model = Net()
trained_model.load_state_dict(torch.load('D:/Downloads/model.pth'))
dummy_input = Variable(torch.randn(16, 3, 5, 5))
torch.onnx.export(trained_model, dummy_input, "D:/Downloads/model.onnx")