import copy
import matplotlib.pyplot as plt
import seaborn as sns


def inner_city_check(nodes_subset, subset_distances_inside, subset_distances_outside) -> dict:
    dict_outside = {}

    for c_checked in range(len(nodes_subset)):
        position = 'inside'
        for c_other in range(len(nodes_subset)):
            if subset_distances_inside[c_checked, c_other] + subset_distances_outside[c_checked, c_other] > 0:

                if subset_distances_inside[c_checked, c_other] == 0:  # if there exists a trip from this customer for which inner city costs are 0, the customer must be outside
                    position = 'outside'

        dict_outside[c_checked] = position

    return dict_outside


def plotVRP(solution, points, outside_dict, show_depot=True, title='ArcPlot'):
    """
    routes: List of lists with the different orders in which the nodes are visited
    points: coordinates for the different nodes
    color: currently not used
    show_depot: deletes depot from all paths if false
    """
    plt.figure(1, figsize=(10, 9))
    plt.title(title)

    path = copy.deepcopy(solution)  # need to deepcopy so we dont destroy the real route by drawing them without depot

    if not show_depot:  # allows us to draw routes without depot, which gives a clearer picture
        for r in path:
            while 0 in r.customer_list:
                r.customer_list.remove(0)

    a_scale = 0.0025  # size of the arrowhead

    n = len(path)
    colorlist = sns.color_palette("husl", n)

    counter_route = 0
    for r in path:
        counter_route += 1
        x = []
        y = []
        color = colorlist[counter_route - 1]
        for c in r.customer_list:
            x.append(points[c][0])
            y.append(points[c][1])
        for i in range(0, len(x) - 1):  # draw the route
            if i == 0:
                plt.arrow(x[i], y[i], (x[i + 1] - x[i]), (y[i + 1] - y[i]), head_width=a_scale,
                          color=color, length_includes_head=True,
                          label="route " + str(counter_route) + ' - ' + str(r.vehicle.type))
            else:
                plt.arrow(x[i], y[i], (x[i + 1] - x[i]), (y[i + 1] - y[i]), head_width=a_scale,
                          color=color, length_includes_head=True)

            if outside_dict[r.customer_list[i]] == 'outside':
                plt.plot(x[i], y[i], "bo", markersize=6)
                plt.text(x[i], y[i], str(counter_route))  # adds route_counter to every dot
            else:
                plt.plot(x[i], y[i], "ro", markersize=6)
                plt.text(x[i], y[i], str(counter_route))

        plt.plot(points[0][0], points[0][1], "ks", markersize=8)

    plt.legend(loc="upper left")
    plt.show()


def plotGraph(points, title: str):
    x = []
    y = []

    for i in range(len(points)):
        x.append(points[i][0])
        y.append(points[i][1])

    plt.plot(x, y, "co", markersize=3)
    plt.title(title)

    plt.show()


def plotSubplots(points, points2, title='SubPlots'):
    plt.style.use('seaborn-whitegrid')

    x = []
    y = []

    for i in range(len(points)):
        x.append(points[i][0])
        y.append(points[i][1])

    x2 = []
    y2 = []

    for i in range(len(points2)):
        x2.append(points2[i][0])
        y2.append(points2[i][1])

    fig = plt.figure(1, figsize=(10, 9))
    ax1 = fig.add_subplot(211)
    plt.plot(x, y, "co", markersize=2)  # draw the nodes

    ax2 = fig.add_subplot(212, sharex=ax1)
    plt.plot(x2, y2, color='red', markersize=3)

    plt.title(title)
    fig.subplots_adjust(hspace=0)  # remove vertical space between subplots

    plt.show()


def plot3Subplots(points, points2, points3, title='SubPlots'):
    plt.style.use('seaborn-whitegrid')

    x = []
    y = []

    for i in range(len(points)):
        x.append(points[i][0])
        y.append(points[i][1])

    x2_green = []
    y2_green = []
    x2_red = []
    y2_red = []

    for i in range(len(points2)):
        if points2[i][2]:  # feasible solutions
            x2_green.append(points2[i][0])
            y2_green.append(points2[i][1])
        else:
            x2_red.append(points2[i][0])
            y2_red.append(points2[i][1])

    x3 = []
    y3 = []

    for i in range(len(points3)):
        x3.append(points3[i][0])
        y3.append(points3[i][1])

    fig = plt.figure(1, figsize=(10, 9))
    ax1 = fig.add_subplot(211)
    plt.plot(x, y, "o", color='c', markersize=2)  # draw the nodes
    plt.plot(x2_red, y2_red, "d", color='orange', markersize=4)  # draw best solutions in red/green depending on feasibility
    plt.plot(x2_green, y2_green, "d", color='blue', markersize=5)

    fig.add_subplot(212, sharex=ax1)
    plt.plot(x3, y3, color='red', markersize=3)

    plt.title(title)
    fig.subplots_adjust(hspace=0)  # remove vertical space between subplots

    plt.show()
