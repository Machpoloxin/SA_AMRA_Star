import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def read_map_file(filename):
    x_coords, y_coords, z_coords = [], [], []
    with open(filename, 'r') as file:
        for line in file:
            # (x, y, z, value)
            # seprate with ()
            parts = line.strip('()\n').split(', ')
            if len(parts) == 4:  
                x, y, z, value = parts
                if value != 'non':  
                    x_coords.append(int(x))
                    y_coords.append(int(y))
                    z_coords.append(int(z))
    return x_coords, y_coords, z_coords


def init_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')
    plt.title('AMRA*_Search')
    return fig, ax

def update_plot(ax, x_coords, y_coords, z_coords, color='r', marker='s',markersize=100):
    ax.scatter(x_coords, y_coords, z_coords, c=color, marker=marker,s=markersize)
    plt.draw()

fig, ax = init_plot()

x_coords, y_coords, z_coords = read_map_file('Testmap.map')
update_plot(ax, x_coords, y_coords, z_coords)
x_coords, y_coords, z_coords = read_map_file('MyExploredNodes.map')
update_plot(ax, x_coords, y_coords, z_coords, color='g',marker='s',markersize=10)
x_coords, y_coords, z_coords = read_map_file('MySolutionPath.map')
update_plot(ax, x_coords, y_coords, z_coords, color='b',marker='x',markersize=30)

plt.show()

