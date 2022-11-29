import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d

import plotly.graph_objects as go
from plotly.subplots import make_subplots

from scipy.spatial.transform import Rotation as R
from itertools import cycle

palette = cycle(['grey']) 


class SuperQuadric():
    def __init__(self, alpha, beta, scale):
        self.alpha = alpha
        self.beta = beta
        self.x = scale[0]
        self.y = scale[1]
        self.z = scale[2]

    def fexp(self, x,p):
        """a different kind of exponentiation"""
        return (np.sign(x) * (np.abs(x)**p))

    def tens_fld(self, A,B,C,P,Q):
        """this module plots superquadratic surfaces with the given parameters"""
        phi, theta = np.mgrid[0:np.pi:80j, 0:2*np.pi:80j]
        x = A * (self.fexp(np.sin(phi),P)) * (self.fexp(np.cos(theta),Q))
        y = B * (self.fexp(np.sin(phi),P)) * (self.fexp(np.sin(theta),Q))
        z = C * (self.fexp(np.cos(phi),P))
        return x , y , z 

    def visualize(self):
        x,y,z = self.tens_fld(self.x, self.y, self.z, self.beta, self.alpha)
        col = np.arange(len(x)**2)
        fig = plt.figure()
        ax3D = fig.add_subplot(111, projection='3d')
        p3d = ax3D.scatter(x, y, z, s=30, c=col, marker='o')                                                                                
        # plt.show()
        return x,y,z

def comp_superquadric(points, scale, eps):
    values = []
    for i in range(len(points[0])):
        value = ((abs(points[0][i])/scale[0])**(2/eps[0]) + (abs(points[1][i])/scale[1])**(2/eps[0]))**(eps[0]/eps[1]) + (abs(points[2][i])/scale[2])**(2/eps[1])
        values.append(value)
    return np.array(values)


# scale = np.array([1,1,1])
# alpha = [1.9, 1.7, 1.5, 1.3, 1.1, 0.9, 0.7, 0.5, 0.3, 0.1]
# beta = [1.9, 1.7, 1.5, 1.3, 1.1, 0.9, 0.7, 0.5, 0.3, 0.1]


# alpha = [0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]
# beta = [1.9, 1.7, 1.5, 1.3, 1.1, 0.9, 0.7, 0.5, 0.3, 0.1]

# alpha = [1.9, 1.7, 1.5, 1.3, 1.1, 0.9, 0.7, 0.5, 0.3, 0.1]
# beta = [0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]

# alpha = [0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]
# beta = [0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]


# fig = make_subplots(
#     rows=len(alpha), cols=len(beta),
#     specs=[[{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}],
#            [{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}],
#            [{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}],
#            [{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}],
#            [{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}],
#            [{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}],
#            [{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}],
#            [{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}],
#            [{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}],
#            [{'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}, {'type': 'surface'}]]
#     )


# for i in range(len(alpha)):
#     for j in range(len(beta)):
#         print(i, j)
#         superquadric = SuperQuadric(alpha[i], beta[j], scale)
#         x,y,z = superquadric.visualize()
#         fig.add_trace(
#             go.Mesh3d(
#                 name = 'superquadric, alpha' + str(beta[i]) + str(alpha[j]),
#                 showlegend=True,
#                 x=x.flatten(), 
#                 y=y.flatten(), 
#                 z=z.flatten(), 
#                 alphahull= 0, 
#                 color=next(palette), 
#                 opacity=0.70,
#                 showscale= False),
#                 row = i+1,
#                 col = j+1)

# fig.update_yaxes(secondary_y=False, showgrid=False)
# # fig.update_yaxes(secondary_y=True, showgrid=False)
# fig.update_scenes(
#     xaxis_visible=False, 
#     yaxis_visible=False,
#     zaxis_visible=False,
# )

# fig.update_layout(
#     plot_bgcolor="white",
#     paper_bgcolor="white",
#     title_text='3D subplots with different colorscales',
#     height=1200,
#     width=1200)

# fig.show()



alpha = 1
beta = 0.1
scale = np.array([1,1,1])

fig = go.Figure()
superquadric = SuperQuadric(alpha, beta, scale)
x,y,z = superquadric.visualize()
fig.add_trace(
    go.Mesh3d(
        name = 'superquadric, alpha' + str(beta) + str(alpha),
        showlegend=True,
        x=x.flatten(), 
        y=y.flatten(), 
        z=z.flatten(), 
        alphahull= 0, 
        color=next(palette), 
        opacity=0.70,
        showscale= False))

# fig.update_yaxes(secondary_y=False, showgrid=False)
# fig.update_yaxes(secondary_y=True, showgrid=False)
# fig.update_scenes(
#     xaxis_visible=False, 
#     yaxis_visible=False,
#     zaxis_visible=False,
# )

# fig.update_layout(
#     plot_bgcolor="white",
#     paper_bgcolor="white",
#     title_text='3D subplots with different colorscales',
#     height=1200,
#     width=1200)

fig.show()