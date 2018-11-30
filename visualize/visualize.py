import plotly.plotly as py
import plotly.graph_objs as go
import plotly
plotly.tools.set_credentials_file(username='PaulANathan', api_key='JdiOKIVlHnNRIslmbZ4d')
import numpy as np

#def visual_some_stuff():
def visual_some_stuff(vertices, edges):
    print("Visualizing...")

    # N = 1000
    # random_x = np.random.randn(N)
    # random_y = np.random.randn(N)
    i = 0
    x = []
    y = []
    for v in vertices:
        x.append(v[0])
        y.append(v[1])

    trace = go.Scatter(
        x= x,
        y= y,
        mode='lines+markers'
    )

    data = [trace]
    print("data = " + str(data))
    py.iplot(data, filename='scatter-mode')