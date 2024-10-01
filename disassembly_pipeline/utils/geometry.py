import numpy as np


def is_on_right_side(x, y, xy0, xy1):
    x0, y0 = xy0
    x1, y1 = xy1
    a = float(y1 - y0)
    b = float(x0 - x1)
    c = - a*x0 - b*y0
    return a*x + b*y + c >= 0


def is_inside_rectangle(rect_coords, center_coords):
    """Check if the center_coords lies within the rect_coords """
    vertices = [[rect_coords[0], rect_coords[1]],[rect_coords[2], rect_coords[3]],[rect_coords[4], rect_coords[5]], [rect_coords[6], rect_coords[7]]]
    num_vert = len(vertices)
    x = center_coords[0]
    y = center_coords[1]

    #is_right =[]
    #for i in range(num_vert):
    #    rospy.loginfo("{}".format(vertices[i]))
    #    k = is_on_right_side(x, y, vertices[i], vertices[(i + 1)%num_vert])
    #    is_right.append(k)
    is_right = [is_on_right_side(x, y, vertices[i], vertices[(i + 1) % num_vert]) for i in range(num_vert)]
    all_left = not any(is_right)
    all_right = all(is_right)
    return all_left or all_right
