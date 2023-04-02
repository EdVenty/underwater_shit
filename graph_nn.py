import plotly.express as px

with open("D:/Downloads/nn2.csv") as file:
    data = [line.strip('\n').split(",") for line in file.readlines()]
    df = [{"epoch": int(x) * 10 * 15, "accuracy": float(y)} for x, y in data]
# print(df)
fig = px.line(df, x='epoch', y="accuracy")
fig.show()