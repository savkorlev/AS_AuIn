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
vehicles = 2

location_list = create_graph(locations, demands)  # call the function to create the locations. 7 locations (2 of which with 0 demand at the end and start represent depos)

cpx = cplex.Cplex()
cpx.parameters.timelimit.set(225.0)  # you can set the time limit in seconds
cpx.objective.set_sense(cpx.objective.sense.minimize)

x = {i: {j: "x_" + str(i) + "_" + str(j) for j in range(len(location_list))} for i in range(len(location_list))}

y = {i: "y_" + str(i) for i in range(len(location_list))}

# _____DECISION VARIABLES_____
for i in location_list:
    for j in location_list:
        if j.index != i.index:
            cpx.variables.add(obj=[i.distance[j]], types=["B"], names=[x[i.index][j.index]])

for i in location_list:
    cpx.variables.add(lb=[i.demand[0]], ub=[payload], types=["C"], names=[y[i.index]])
    # note that y variables are continuous

# _____CONSTRAINTS_____
for i in location_list:  # constraints 1 (2.2)
    if i.index != 0 and i.index != len(location_list) - 1:
        coef_1, var_1 = [], []
        for j in location_list:
            if j.index != i.index and j.index != 0:
                coef_1.append(1)
                var_1.append(x[i.index][j.index])
        cpx.linear_constraints.add(lin_expr=[[var_1, coef_1]], senses=["E"], rhs=[1])

for h in location_list:  # constraints 2 (2.3)
    if h.index != 0 and h.index != len(location_list) - 1:
        coef_2, var_2 = [], []
        for i in location_list:
            if i.index != len(location_list) - 1 and i is not h:
                coef_2.append(1)
                var_2.append(x[i.index][h.index])
        for j in location_list:
            if j.index != 0 and j is not h:
                coef_2.append(-1)
                var_2.append(x[h.index][j.index])
        cpx.linear_constraints.add(lin_expr=[[var_2, coef_2]], senses=["E"], rhs=[0])

coef_3, var_3 = [], []  # constraints 3 (2.4)
for j in location_list:
    if j.index != 0 and j.index != len(location_list) - 1:
        coef_3.append(1)
        var_3.append(x[0][j.index])
cpx.linear_constraints.add(lin_expr=[[var_2, coef_2]], senses=["L"], rhs=[vehicles])

for i in location_list:  # constraints 4 (2.5)
    for j in location_list:
        if j.index != i.index:
            coef_4 = [1, -1, -j.demand[0] - payload]
            var_4 = [y[j.index], y[i.index], x[i.index][j.index]]
            cpx.linear_constraints.add(lin_expr=[[var_4, coef_4]], senses=["G"], rhs=[-payload])

print_solution(cpx, location_list, x, y)  # solve the model and print the solution
