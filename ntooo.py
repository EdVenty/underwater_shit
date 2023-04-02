n = int(input())
ss = [input() for _ in range(n)]

sk = {i: ss.count(i) for i in set(ss)}



print(' '.join(new))