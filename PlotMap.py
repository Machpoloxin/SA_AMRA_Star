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
                if value == 'ob':  
                    x_coords.append(int(x))
                    y_coords.append(int(y))
                    z_coords.append(int(z))
    return x_coords, y_coords, z_coords


def plot_map(x_coords, y_coords, z_coords):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o')  #
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')
    plt.title('3D Map of Obstacles')
    plt.show()


x_coords, y_coords, z_coords = read_map_file('sparsemap.map')
plot_map(x_coords, y_coords, z_coords)
