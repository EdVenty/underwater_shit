# n, r = [int(i) for i in input().split()]

# ns = [[int(i) for i in input().split()] for _ in range(n)]

# ns.sort(key=lambda x: x[1])

# сортируем точки по x
# находим расстояния между точками x и y
# для каждой точки идем вправо и влево и смотрим если по x попадает в радиус и если по y
# проверяем по теореме пифагора
# если коалиция не присвоена, то создаем новую для двух точек, иначе присваиваем обший номер коалиции

# import re

# with open("C:/Users/d3and/Desktop/10-0.txt", encoding='utf-8') as file:
#     text = file.read()

# i = re.findall(r'\W[нН]яня\W', ' ' + text + ' ')
# print(i)
# print(len(i))

# n, r = [int(i) for i in input().split()]
# dots = [[int(i) for i in input().split()] + [0] for _ in range(n)]
# r = 2

# dots = [
#     [1, 1, 0],
#     [2, 2, 0],
#     [10, 5, 0],
#     [11, 5, 0],
#     [12, 5, 0],
#     [1.5, 2, 0],
#     [-10, 0, 0]
# ]

# last_c = 1
# cs = {}

# dots.sort(key=lambda x: x[0])

# for i, d in enumerate(dots):
#     # go right
#     j = i + 1
#     while j <= len(dots) - 1 and dots[j][0] - d[0] <= r:
#         if abs(d[1] - dots[j][1]) <= r and ((d[0] - dots[j][0]) ** 2 + (d[1] - dots[j][1]) ** 2) ** 0.5 <= r:
#             print(d, "близко к", dots[j])
#             c = max(d[2], dots[j][2])
#             if c == 0:
#                 c = last_c
#                 d[2] = c
#                 dots[j][2] = c
#                 last_c += 1
#             elif d[2] == 0:
#                 d[2] = c
#             else:
#                 dots[j][2] = c
#         j += 1
#     if d[2] == 0:
#         d[2] = last_c
#         last_c += 1

# print(dots)
# print(last_c - 1)

# s = int(input())
# print(max(min(0.08 * s, 100), 0.05 * s))

n = int(input())
ps = [[i for i in input().split()] for _ in range(n)]

for p in ps:
    