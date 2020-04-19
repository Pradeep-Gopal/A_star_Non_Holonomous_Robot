# import cv2
import numpy as np
import time
import math
# import matplotlib.pyplot as plt


# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import queue
import argparse
from matplotlib.patches import Rectangle

plt.ion()

height = 10.2
width = 10.2


def obstaclecheck_circle(x, y, r, c):
    tot = r + c
    #     print(r+c)
    #     circle1 =  ((x - 5) ** 2) + ((y - 5) ** 2) <= ((1 + r + c) ** 2)
    #     circle2 =  ((x - 3) ** 2) + ((y - 2) ** 2) <= ((1 + r + c) ** 2)
    #     circle3 =  ((x - 7) ** 2) + ((y - 2) ** 2) <= ((1 + r + c) ** 2)
    #     circle4 =  ((x - 7) ** 7) + ((y - 8) ** 2) <= ((1 + r + c) ** 2)

    circle1 = ((np.square(x - 5)) + (np.square(y - 5)) <= np.square(1 + tot))
    circle2 = ((np.square(x - 3)) + (np.square(y - 2)) <= np.square(1 + tot))
    circle3 = ((np.square(x - 7)) + (np.square(y - 2)) <= np.square(1 + tot))
    circle4 = ((np.square(x - 7)) + (np.square(y - 8)) <= np.square(1 + tot))

    if circle1 or circle2 or circle3 or circle4:
        return True
    else:
        return False


def obstaclecheck_square(x, y, r, c):
    tot = r + c
    square1 = (x >= 0.25 - tot) and (x <= 1.75 + tot) and (y >= 4.25 - tot) and (y <= 5.75 + tot)
    square2 = (x >= 8.25 - tot) and (x <= 9.75 + tot) and (y >= 4.25 - tot) and (y <= 5.75 + tot)
    square3 = (x >= 2.25 - tot) and (x <= 3.75 + tot) and (y >= 7.25 - tot) and (y <= 8.25 + tot)
    if square1 or square2 or square3:
        return True
    else:
        return False


def obstacle_check(new_i, new_j, r, c):
    if obstaclecheck_circle(new_i, new_j, r, c):
        return True
    elif obstaclecheck_square(new_i, new_j, r, c):
        return True
    elif ((new_i - r - c <= 0) or (new_j - r - c <= 0) or (new_i + r + c >= width) or (new_j + r + c >= height)):
        return True
    else:
        return False


def goalcheck_circle(x, y, goal_x, goal_y):
    if (((x - goal_x) ** 2) + ((y - goal_y) ** 2) < ((0.5) ** 2)):
        return True
    else:
        return False


def obstacle_map(rad, clearance):
    plot_x = []
    plot_y = []
    rigid_x = []
    rigid_y = []
    for x in range(10):
        for y in range(10):
            if obstacle_check(x, y, rad, clearance):
                rigid_x.append(x)
                rigid_y.append(y)
            #                 blank_image[x, y] = (150, 150, 150)
            if obstacle_check(x, y, 0, 0):
                plot_x.append(x)
                plot_y.append(y)
    return plot_x, plot_y, rigid_x, rigid_y

def action_model(rpm1, rpm2):
    actions = [[rpm1, 0], [0, rpm1], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]
    return actions


def cost2go(pt1, pt2):
    dist = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
    return dist


def threshold(x, y, th, theta):
    x = (round(x * 10) / 10)
    y = (round(y * 10) / 10)
    th = (round(th / theta) * theta)
    return (x, y, th)


def plot_curve(Xi, Yi, Thetai, UL, UR, r, c, flag, N_list, S_list):
    t = 0
    r = 0.033
    L = 0.16
    dt = 0.6
    cost = 0
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180

    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes

    while t < 2:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += r*0.5 * (UL + UR) * math.cos(Thetan) * dt
        Yn += r*0.5 * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        if not (obstacle_check(Xn, Yn, r, c)):
            if flag == 0:
                # plt.plot([Xs, Xn], [Ys, Yn], color="blue")
                cost = cost + cost2go((Xs, Ys), (Xn, Yn))
                N_list.append((Xn, Yn))
                S_list.append((Xs, Ys))
            if flag == 1:
                plt.plot([Xs, Xn], [Ys, Yn], color="red")
        else:
            return None
    Thetan = 180 * (Thetan) / 3.14
    return [Xn, Yn, Thetan, cost, N_list, S_list]


def a_star(start_node, goal_node, step_size, theta, rad, clearance, L_rpm, R_rpm):
    N_list = []
    S_list = []

    start_node = threshold(start_node[0], start_node[1], start_node[2], theta)

    goal_node = threshold(goal_node[0], goal_node[1], goal_node[2], theta)

    #     print(goal_node)
    visited_nodes = np.zeros((140, 140, int(360 / theta)))
    start = (0, start_node, None, None)  # cost, node, parent node
    # print("start", start)
    goal = (0, goal_node, None, None)
    # print("goal", goal)

    nodes = queue.PriorityQueue()
    path_nodes = []
    path_dict = {}
    action_dict = {}
    nodes.put(start)

    while (1):
        current_node = list(nodes.get())

        if (current_node[2] != None):
            current_node[0] = current_node[0] - cost2go(current_node[1], goal[1])
            action_dict[current_node[2]] = current_node[3]


        path_dict[current_node[1]] = current_node[2]

        actions = action_model(L_rpm, R_rpm)

        for new_pos in actions:
            X1 = plot_curve(current_node[1][0], current_node[1][1], current_node[1][2], new_pos[0], new_pos[1], rad,
                            clearance, 0, N_list, S_list)

            if (X1 != None):
                angle = X1[2]

                th = (round(angle / theta) * theta)
                while(th>360):
                    th = th - 360

                while(th<-360):
                    th = th + 360

                if (th < 0):
                    th= th + 360

                if (th > 360):
                    th = th - 360

                if (th == 360):
                    th = 0

                node = (X1[0], X1[1], th)

                #             node_cost = current_node[0] + cost2go((current_node[1][0],current_node[1][1]) ,node) + cost2go(node, goal[1])
                node_cost = current_node[0] + X1[3] + cost2go(node, goal[1])
                node_parent = current_node[1]
                node = threshold(node[0], node[1], node[2], theta)
                act = new_pos
                new_node = (node_cost, node, node_parent, act)
                #                 print("newnode",new_node)

                if obstacle_check(node[0], node[1], rad, clearance) == False:
                    if (visited_nodes[int(node[0] * 10)][int(node[1] * 10)][int(node[2] / (theta))] == 0):
                        visited_nodes[int(node[0] * 10)][int(node[1] * 10)][int(node[2] / (theta))] = 1
                        #                     x_explored.append(node[0])
                        #                     y_explored.append(node[1])
                        #                     x_parent.append(node_parent[0])
                        #                     y_parent.append(node_parent[1])

                        #                     new_node = (node_cost, node, node_parent, act)
                        #                         print("newnodeappended",new_node)
                        #                         print("\n")
                        nodes.put(new_node)

        if goalcheck_circle(current_node[1][0], current_node[1][1], goal_node[0], goal_node[1]):
            print("Goal reached")

            # print("path_nodes = ", path_nodes)
            # t = time.localtime()
            # current_time = time.strftime("%H:%M:%S", t)
            # print('Goal reached')
            # print("Time taken to explore = ", current_time)
            path = []
            path.append((goal_node[0], goal_node[1]))

            #             str_p1 = str(current_node[2][0])
            #             str_p2 = str(current_node[2][1])
            #             str_p3 = str(current_node[2][2])
            #             parent = str_p1+','+str_p2+','+str_p3
            parent = current_node[2]

            while parent != None:
                temp = path_dict.get(parent)

                path.append(parent)
                #                 print("path",path)
                parent = temp
                if parent == start_node:
                    break
            # t = time.localtime()
            # current_time = time.strftime("%H:%M:%S", t)
            # print("time taken to backtrack = ", current_time)
            
            #             path.append((start_node[0], start_node[1]))
            print("Backtracking done - shortest path found")

            path = path[::-1]
            print(path)
            # print(N_list, S_list)

            return path, N_list, S_list


def main():
    Parser = argparse.ArgumentParser()
    Parser.add_argument('--user_input')
    Parser.add_argument('--animation')

    Args = Parser.parse_args()
    user_input = int(Args.user_input)
    animation = int(Args.animation)
    if user_input == 1:
        x_start = float(input('Enter the x-coordinate of start point: '))
        y_start = float(input('Enter the y-coordinate of start point: '))
        theta_start = float(input('Enter the orientation of start point: '))
        x_goal = float(input('Enter the x-coordinate of goal point: '))
        y_goal = float(input('Enter the y-coordinate of goal point: '))

        L_rpm = float(input('Enter the left wheel RPM: '))
        R_rpm = float(input('Enter the right wheel RPM: '))
        clearance1 = float(input('Enter the clearance for the robot: '))

        robot_radius = 0.08
        clearance = 0.16 + clearance1
        theta = 15
        step_size = 1
        theta_goal = 0.0

    else:

        x_start = -4.5
        y_start = -4.5
        theta_start = 0.0
        theta = 15
        x_goal = 4.5
        y_goal = 4.5
        theta_goal = 0.0
        step_size = 1

        robot_radius = 0.08
        clearance = 0.16

        L_rpm = 9.0
        R_rpm = 11.0
        # L_rpm = 13
        # R_rpm = 15


    print("start_coordinates = ", x_start, y_start)
    print("goal_coordinates = ", x_goal, y_goal)
    x_start += 5.0
    y_start += 5.0
    x_goal += 5.0
    y_goal += 5.0
    # t = time.localtime()
    # current_time = time.strftime("%H:%M:%S", t)
    # print("start_time = ", current_time)
    start_time =  time.time()
    start_node = (x_start, y_start, theta_start)
    goal_node = (x_goal, y_goal, theta_goal)
    if obstacle_check(start_node[0], 10 - start_node[1], robot_radius, clearance) == True:
        print("The start node is either in the obstacle space or out of map")
    elif obstacle_check(goal_node[0], 10 - goal_node[1], robot_radius, clearance) == True:
        print("The goal node is either in the obstacle space or out of map")
    else:

        fig, ax = plt.subplots()
        ax.set(xlim=(0, 10.2), ylim=(0, 10.2))

        #         plot_x, plot_y, rigid_x, rigid_y = obstacle_map(robot_radius, clearance)

        #         ax.plot(rigid_x, rigid_y, ".y")
        #         ax.plot(plot_x, plot_y, ".k")

        c1 = plt.Circle((5, 5), 1, fill=None)
        c2 = plt.Circle((3, 2), 1, fill=None)
        c3 = plt.Circle((7, 2), 1, fill=None)
        c4 = plt.Circle((7, 8), 1, fill=None)
        currentAxis = plt.gca()
        currentAxis.add_patch(Rectangle((0.25, 4.25), 1.5, 1.5, fill=None, alpha=1))
        currentAxis.add_patch(Rectangle((8.25, 4.25), 1.5, 1.5, fill=None, alpha=1))
        currentAxis.add_patch(Rectangle((2.25, 7.25), 1.5, 1.5, fill=None, alpha=1))
        currentAxis.add_patch(Rectangle((0, 0), 10, 10, fill=None, alpha=1))

        ax.add_artist(c1)
        ax.add_artist(c2)
        ax.add_artist(c3)
        ax.add_artist(c4)
        ax.set_aspect('equal')
        plt.grid()
        path, N_list, S_list = a_star(start_node, goal_node, step_size, theta, robot_radius, clearance, L_rpm, R_rpm)
        print("size", len(N_list))

        #         path,x_explored,y_explored,x_parent,y_parent = a_star(start_node, goal_node, step_size, theta, robot_radius, clearance)

        if path == None:
            print("Path could not be found. Check inputs")

        else:
            end_time = time.time()
            print("Time taken to find the shortest path (in seconds) is:", abs(end_time - start_time))
            plt.plot(goal_node[0], goal_node[1], color='green', marker='o', linestyle='dashed', linewidth=1,
                     markersize=4)
            plt.plot(start_node[0], start_node[1], color='green', marker='o', linestyle='dashed', linewidth=1,
                     markersize=4)

            if animation ==1:
	            l = 0
	            while l < len(N_list):
	                # plt.plot([Xs, Xn], [Ys, Yn], color="blue")
	                plt.plot([S_list[l][0], N_list[l][0]], [S_list[l][1], N_list[l][1]], color="blue")
	                # plt.plot([x_explored[l], x_parent[l]], [y_explored[l], y_parent[l]], "-c")
	                l = l + 1
	                plt.show()
	                plt.pause(0.000000000000000000000000000000000005)
            else:
	            l = 0
	            while l < len(N_list):
	                plt.plot([S_list[l][0], N_list[l][0]], [S_list[l][1], N_list[l][1]], color="blue")
	                l = l + 1

            path = path[::-1]
            x_path = [path[i][0] for i in range(len(path))]
            y_path = [path[i][1] for i in range(len(path))]
            plt.plot(x_path, y_path, "-r")
            # plt.figure(figsize=(10, 10))
            # plt.show()
            plt.savefig("output.png")
            plt.pause(5)
            plt.close()
            t = time.localtime()
            end_time2 = time.time()
            print("Time taken to find the shortest path and to plot (in seconds) is:", abs(end_time2 - start_time))


if __name__ == '__main__':
    main()
