from typing import List
import random
import numpy as np
import time


class Neuro:
    def __init__(self, inputs: int, weights: List[float] = None) -> None:
        """Neuro object

        Args:
            inputs (int): Number of inputs
            weights (List[float], optional): Weights list. If not then auto-generate. Defaults to None.
        """
        self.inputs = inputs
        if weights is None:
            self.weights = [random.uniform(-2, 2) for _ in range(inputs)]
        else:
            assert inputs == len(
                weights), f"Weight length ({len(weights)}) not equals inputs length ({inputs})."
            self.weights = weights

    def work(self, values: List[float]) -> float:
        """Process values with neuro

        Args:
            values (List[int]): Values to work

        Returns:
            float: Result
        """
        assert len(values) == self.inputs, f"Values length ({len(values)}) not equals inputs length ({self.inputs})."
        # sum = 0
        sum([i * w for (i, w) in zip(values, self.weights)])
        return self._sigmoid(sum)

    def _sigmoid(self, value: float) -> float:
        """Calculates sigmoid of value

        Args:
            value (float): Value to calc

        Returns:
            float: Result
        """
        return 1 / (1 + np.exp(-value))

# i = 0
# while True: 
#     a1 = Neuro(2)
#     a2 = Neuro(2)
#     b1 = Neuro(2)
#     inp = [1, 0, 0, 1]
#     out = b1.work([a1.work(inp[:2]), a2.work(inp[2:])])
#     inp2 = [0, 1, 1, 0]
#     out2 = b1.work([a1.work(inp2[:2]), a2.work(inp2[2:])])
#     print("{}, {:.1f}, {:.1f}".format(i, out, out2))
#     if out > 0.6 and out2 < 0.4:
#         print(f"Таракан обучен! Мы потеряли {i} бойцов.")
#         break
#     # print("Out is", n.work([0.6, 0.9]))
#     i += 1

import torch.nn as nn
import torch
import time

LEARNING_RATE = 0.0001
EPOCHS = 30000
PRINT_DELAY = 2
TEST_IN_EPOCHS = 10

# loss_func = nn.BCEWithLogitsLoss()
loss_func = nn.MSELoss()

device = "cuda" if torch.cuda.is_available() else "cpu"
print("Using {} device".format(device))

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

    def train(self, data: torch.Tensor, target: torch.Tensor, epoch, last_test, print_progress: bool=False):
        super().train()
        size = len(target)
        for i, (d, t) in enumerate(zip(data, target)):
            d, t = d.to(device), t.to(device)
            output = self(d)
            loss = loss_func(output, t.float())

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            if print_progress:
                print("Epoch {} | Loss: {} [{}/{}] Last test: {}%".format(epoch + 1, loss.item(), i, size, last_test))


    def test(self, data: torch.Tensor, target: torch.Tensor):
        super().train(False)
        size = len(target)
        counter = 0
        with torch.no_grad():
            for i, (d, t) in enumerate(zip(data, target)):
                d, t = d.to(device), t.to(device)
                output = self(d)
                if -0.2 < t - output < 0.2: counter += 1
            return counter

input = torch.tensor([
    [1, 0, 0, 1],
    [1, 1, 1, 0],
    [0, 1, 0, 1], 
    [1, 1, 1, 1], 
    [0, 0, 0, 0], 
    [0, 0, 1, 1],
    [1, 1, 0, 0],
    [0,0,0,0]
]).float()

target = torch.tensor([
    1, 1, 0, 1, 0, 0, 1, 0
]).unsqueeze(1)

test = (
    torch.tensor([
        [1, 0, 0, 0], 
        [0, 0, 1, 0], 
        [0, 1, 1, 1], 
        [1, 0, 1, 1]
    ]).float(),
    torch.tensor([
        1, 0, 0, 1
    ]).unsqueeze(1)
)

net = Net().to(device)
optimizer = torch.optim.SGD(net.parameters(), lr=LEARNING_RATE)

last_test = 0
timer = 0

for epoch in range(EPOCHS):
    # print(f"Epoch {epoch + 1}.")
    # print("---------------------")
    if_print = False
    if time.time() > timer + PRINT_DELAY:
        if_print = True
        timer = time.time()
        # torch.save(net.state_dict(), "./drive/MyDrive/Models/cockroach.pth")

    net.train(input, target, epoch, last_test, if_print)
    if epoch % TEST_IN_EPOCHS == 0:
        correct = net.test(test[0], test[1])
        size = len(test[1])
        last_test = correct / size * 100