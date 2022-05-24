import pandas as pd
import cplex
from instances.Tools import create_graph, print_solution

df_nodes = pd.read_csv("data/nodes.txt")

# PERMUTATION (if needed)
# import random
# for row, content in df_nodes.iterrows():
#     if row == 0:
#         df_nodes.iloc[row, 0] = int(random.uniform(0, 500))
#         df_nodes.iloc[row, 1] = int(random.uniform(0, 500))
#     else:
#         df_nodes.iloc[row, 0] = int(random.uniform(0, 500))
#         df_nodes.iloc[row, 1] = int(random.uniform(0, 500))
#         df_nodes.iloc[row, 2] = int(random.uniform(300, 600))
# df_nodes.to_csv('data/nodes_permutated.txt', index=False)

# LOCATIONS (coordinates of nodes)
locations = df_nodes.drop(columns=['Demand[kg]']).values.tolist()

# DEMANDS (in kg)
demands = df_nodes.drop(columns=['Lon', 'Lat']).values.tolist()

# VEHICLES
payload = 900
vehicles = 6

location_list = create_graph(locations, demands)  # call the function to create the locations. 7 locations (2 of which with 0 demand at the end and start represent depos)

cpx = cplex.Cplex()
cpx.parameters.timelimit.set(225.0)  # you can set the time limit in seconds
cpx.objective.set_sense(cpx.objective.sense.minimize)

x = {i: {j: "x_" + str(i) + "_" + str(j) for j in range(len(location_list))} for i in range(len(location_list))}
# x - names of edges. x[0][1] for example will return us x_0_1

y = {i: "y_" + str(i) for i in range(len(location_list))}

# _____DECISION VARIABLES_____
# edge decision variable - do we keep the edge
for i in location_list:
    for j in location_list:
        if j.index != i.index:  # excluding the case when locations are the same (distance 0)
            cpx.variables.add(obj=[i.distance[j]], types=["B"], names=[x[i.index][j.index]])
            # [i.distance[j]] - distance between i and j
            # [x[i.index][j.index]] - x[0][1] for example will return us x_0_1

# print(cpx.variables.get_types())
# print(len(location_list))

# node decision variable - ???
for i in location_list:
    cpx.variables.add(lb=[i.demand[0]], ub=[payload], types=["C"], names=[y[i.index]])
    # note that y variables are continuous

# TODO: add more variables

# _____CONSTRAINTS_____
# all nodes are visited exactly once
for i in location_list:  # constraints 1 (2.2)
    if i.index != 0 and i.index != len(location_list) - 1:  # excluding the depos
        coef_1, var_1 = [], []  # what's that?
        for j in location_list:
            if j.index != i.index and j.index != 0:  # excluding the same locations + depos
                coef_1.append(1)  # appending just the number 1 for some reason
                var_1.append(x[i.index][j.index])  # appending the name of the edge
        cpx.linear_constraints.add(lin_expr=[[var_1, coef_1]], senses=["E"], rhs=[1])  # if rhs is 1 then can for example a float 0.5 be a result?

for h in location_list:  # constraints 2 (2.3)
    if h.index != 0 and h.index != len(location_list) - 1:  # excluding the depos
        coef_2, var_2 = [], []
        for i in location_list:
            if i.index != len(location_list) - 1 and i is not h:  # excluding the last node (depo) and the same nodes
                coef_2.append(1)
                var_2.append(x[i.index][h.index])
        for j in location_list:
            if j.index != 0 and j is not h:  # excluding the first node (depo) and the same nodes
                coef_2.append(-1)
                var_2.append(x[h.index][j.index])
        cpx.linear_constraints.add(lin_expr=[[var_2, coef_2]], senses=["E"], rhs=[0])

coef_3, var_3 = [], []  # constraints 3 (2.4)
for j in location_list:
    if j.index != 0 and j.index != len(location_list) - 1:  # excluding the first node (depo) and the same nodes
        coef_3.append(1)
        var_3.append(x[0][j.index])  # x_0_1, x_0_2 and so on
cpx.linear_constraints.add(lin_expr=[[var_3, coef_3]], senses=["L"], rhs=[vehicles])

for i in location_list:  # constraints 4 (2.5)
    for j in location_list:
        if j.index != i.index:
            coef_4 = [1, -1, -j.demand[0] - payload]
            var_4 = [y[j.index], y[i.index], x[i.index][j.index]]
            cpx.linear_constraints.add(lin_expr=[[var_4, coef_4]], senses=["G"], rhs=[-payload])

print_solution(cpx, location_list, x, y)  # solve the model and print the solution
