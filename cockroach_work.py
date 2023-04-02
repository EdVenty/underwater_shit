from array import array
import torch.nn as nn
import torch

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.flatten = nn.Flatten()
        self.stack = nn.Sequential(
            nn.Linear(4, 2),
            nn.ReLU(),
            # nn.Sigmoid(),
            nn.Linear(2, 1),
            nn.ReLU()
        )
    
    def forward(self, x):
        x = self.stack(x)
        return x

net = Net()
net.load_state_dict(torch.load("./cockroach.pth"))

def should_i_run(t: torch.Tensor):
    o = net(t)
    if 0.8 <= o <= 1.2:
        return True
    else:
        return False

arr = [0, 1, 0, 0]

print("БЕЖИМ!" if should_i_run(torch.tensor(arr).float()) else "чиллим")
