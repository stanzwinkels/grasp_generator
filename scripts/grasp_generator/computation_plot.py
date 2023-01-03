#!/usr/bin/env python

import pdb
import numpy as np
import pickle
import joblib

import pandas as pd
import plotly.express as px



data = np.array([[5, 10, 20, 50, 100, 200],
        [0.154, 0.137, 0.148, 0.166, 0.147, 0.145],
        [0.286, 0.309, 0.305, 0.299, 0.311, 0.280]])

df = pd.DataFrame(np.transpose(data), columns=['Tasks', 'Version-1', 'Version-2'])

# fig = px.line(group, x="Hour", y="value",color='Level', title='Graph',category_orders={'Level':['H','M','L']}
#              ,color_discrete_map={'H':'royalblue','M':'orange','L':'firebrick'}, line_dash='Level')


# plotly
fig = px.line(df, y=df.columns[1:3], x='Tasks').update_layout(
    xaxis_title="Tasks",
    yaxis_title="Time [s]",
    yaxis_range=[0,0.5],
    plot_bgcolor="white",
    paper_bgcolor="white",

)

fig.update_traces(patch={"line": {"color": "blue", "width": 4, "dash": 'longdash'}})
fig.update_traces(patch={"line": {"color": "orange", "width": 4, "dash": 'solid'}}, selector={"legendgroup": "Version-1"}) 

fig.update_xaxes(showline=True, linewidth=4, linecolor='black', gridcolor='black')
fig.update_yaxes(showline=True, linewidth=4, linecolor='black', gridcolor='black')

fig.update_layout(
    font=dict(
        # family="Courier New, monospace",
        size=40,  # Set the font size here
        # color="RebeccaPurple"
    )
)


fig.show()



# fig = px.line(df, x="Tasks", y=df.columns[1:3], category_orders={'Level':['H','M']}, color_discrete_map={'H':'royalblue','M':'orange'}).update_layout(
#     xaxis_title="Tasks",
#     yaxis_title="Time [s]",
#     yaxis_range=[0,0.5],
#     plot_bgcolor="white",
#     paper_bgcolor="white",
#     # xaxis=dict(showgrid=True), 
#     # yaxis=dict(showgrid=True)
#      )

# fig.update_traces(
#     selector={"name": "version-1"}, 
#     line = {'dash', 'dash'},

# )
# fig.update_xaxes(showline=True, linewidth=4, linecolor='black', gridcolor='black')
# fig.update_yaxes(showline=True, linewidth=4, linecolor='black', gridcolor='black')

# fig.update_layout(
#     font=dict(
#         # family="Courier New, monospace",
#         size=40,  # Set the font size here
#         # color="RebeccaPurple"
#     )
# )


# fig.show()
