import random
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.transforms import Bbox
from matplotlib.path import Path
import math

# GLOBAL VARIABLES AND FUNCTIONS
start_point = (1, 9)
end_point = (9, 1)
nodes = [start_point]
lines = []
dead_end_lines = []


def remove_common(a, b):
    for i in a[:]:
        if i in b:
            a.remove(i)
            b.remove(i)


def closest(x1, y1):
    dist = 10 * ((2) ** 0.5)
    j = 0
    # n=0
    for i in range(len(nodes) - 1):
        x = nodes[i][0]
        y = nodes[i][1]
        if (((x - x1) ** 2 + (y - y1) ** 2) ** (0.5)) <= dist:
            dist = ((x - x1) ** 2 + (y - y1) ** 2) ** (0.5)
            j = i

    return nodes[j]

def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

# DIMENSIONS OF GRID
width = 10
height = 10

# OBSTACLE AND COURSE INITIALIZATION
num_obstacles = int(input("How many obstacles do you want to make (recommended below 5): "))
bbox = []


def generate_obstacles(num_rectangles, width, height):
    rectangles = []
    for i in range(num_rectangles):

        while True:
            x = random.randint(3, width - 3)
            y = random.randint(3, height - 3)
            w = random.randint(1, 2)
            h = random.randint(1, 2)
            rect = (x, y, w, h)
            overlap = False
            for r in rectangles:
                if (x <= r[0] + r[2] and x + w >= r[0] and y <= r[1] + r[3] and y + h >= r[1]):
                    overlap = True
                    break
            if not overlap:
                rectangles.append(rect)
                bbox1 = Bbox.from_bounds(x, y, w, h)
                bbox.append(bbox1)

                break

    return rectangles


rectangles = generate_obstacles(num_obstacles, width, height)

fig, ax = plt.subplots()

plt.scatter(start_point[0], start_point[1], color='red')
plt.scatter(end_point[0], end_point[1], color='red')
end_rect = plt.Rectangle((end_point[0] - 0.5, end_point[1] - 0.5), 1, 1, color='blue', alpha=0.5)
ax.add_patch(end_rect)

for rectangle in rectangles:
    rect = plt.Rectangle((rectangle[0], rectangle[1]), rectangle[2], rectangle[3], color='gray', alpha=0.5)
    ax.add_patch(rect)


# OBSTACLE DETECTION
def obstacle_hit(p1, p2):
    hits = False
    path = Path([p1, p2])
    for i in range(num_obstacles):
        n = 0
        if path.intersects_bbox(bbox[i]):
            n = n + 1
            if n >= 1:
                hits = True
                break
    return hits


# DETECTION OF END POINT
def endpoint_hit(p):
    hits = False
    if ((end_point[0] - 0.5) <= p[0] <= (end_point[0] + 0.5)) and (
            (end_point[1] - 0.5) <= p[1] <= (end_point[1] + 0.5)):
        hits = True

    return hits


plt.xlim(0, 10)
plt.ylim(0, 10)


# RRT ALGORITHM
iterations = 0
while endpoint_hit(nodes[len(nodes) - 1]) == False:

    x2 = random.randint(0, 10)
    y2 = random.randint(0, 10)
    sample_point = (x2, y2)
    (x1, y1) = closest(x2, y2)
    if x2 - x1 == 0:
        if y2 - y1 > 0:
            dir = np.pi / 2

        elif y2 - y1 < 0:
            dir = -(np.pi / 2)

    else:
        dir = math.atan((y2 - y1) / (x2 - x1))

    x_new = x1 + 0.5 * np.cos(dir)
    y_new = y1 + 0.5 * np.sin(dir)
    if obstacle_hit((x1, y1), (x_new, y_new)) == False and 0 < x_new < 10 and 0 < y_new < 10:
        plt.plot([x1, x_new], [y1, y_new], color='black')
        nodes.append((x_new, y_new))
        lines.append(((x1, y1), (x_new, y_new)))
        plt.scatter(x_new, y_new, color='black', s=10)
        iterations = iterations + 1


# PLOTTING PATH
for j in range(iterations):
    for i in range(len(lines) - 1):
        if lines[i][1] not in [l[0] for j, l in enumerate(lines) if j != i] and endpoint_hit(lines[i][1]) == False:
            dead_end_lines.append(lines[i])

    remove_common(lines, dead_end_lines)

for line in lines:
    plt.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], 'r-')

path_length = (len(lines) - 1) * 0.5



# EFFICIENT PATH ALGORITHM
eff_point = (0, 0)
i = 0
path_length_eff = 0
# print(len(lines))

final_point = lines[len(lines)-1][1]
for j in range(iterations):
    if obstacle_hit(start_point,final_point) == False:
        plt.plot([start_point[0], final_point[0]], [start_point[1], final_point[1]], color='green', linewidth=4)
        path_length_eff = path_length_eff + distance(start_point,final_point)
        break
    else:
        while obstacle_hit(start_point, lines[i][1]) == False:
            eff_point = lines[i][1]
            i = i + 1

        plt.plot([start_point[0], eff_point[0]], [start_point[1], eff_point[1]], color='green',linewidth=4)
        path_length_eff = path_length_eff + distance(start_point, eff_point)
        start_point = lines[i][0]


ax.set_xlim([0, width])
ax.set_ylim([0, height])
print("The number of iternations: ",iterations)
print("Total path length: ", path_length, " units")
print("Total efficient path length: ", path_length_eff, " units")
# plt.legend()
plt.show()