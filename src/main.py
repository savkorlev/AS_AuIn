import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import random
import math
from docplex.mp.model import Model

df_nodes = pd.read_csv("data/nodes.txt")
df_routes = pd.read_csv("data/routes.txt")

# DATA HANDLING

# # ADD a new column to a dataframe
# unloading_times = []
# for i in range(len(df_nodes)):
#     unloading_times.append(int(round(random.uniform(600, 1200), 0)))
# df_nodes['Unloading Times[sec]'] = unloading_times
# df_nodes.to_csv('data/new_column_nodes.txt', index=False)

# # DELETE a column
# df_nodes.drop('Demand[boxes]', axis = 1)
# df_nodes.to_csv('data/new_column_nodes.txt', index=False)

# # CREATE a dataframe
# df_routes = pd.DataFrame()
# number_of_PLANEs = 4
# _from_ = []
# _to_ = []
# for i in range(1, len(df_nodes)):
#     _from_.extend([i] * number_of_PLANEs)
#     _to_.extend(range(1, 5))
# df_routes['From [supplier]'] = _from_
# df_routes['To [P-LANE]'] = _to_

# # PERMUTATION (change if needed)
# for row, content in df_nodes.iterrows():
#     if row == 0:
#         df_nodes.iloc[row, 0] = random.uniform(0, 500)
#         df_nodes.iloc[row, 1] = random.uniform(0, 500)
#         df_nodes.iloc[row, 2] = 0
#         df_nodes.iloc[row, 3] = 0
#     else:
#         df_nodes.iloc[row, 0] = random.uniform(0, 500)
#         df_nodes.iloc[row, 1] = random.uniform(0, 500)
#         df_nodes.iloc[row, 2] = int(random.uniform(1, 9))
#         df_nodes.iloc[row, 3] = int(random.uniform(600, 1200))
# df_nodes.to_csv('data/nodes_permutated.txt', index=False)



Q = 10  # vehicle capacity in boxes TODO: shouldn't be kilograms?
N = []  # set of nodes without the depot
for i in range(1, len(df_nodes)):
    N.append(i)
V = [0] + N  # set of nodes with the depot

loc_x = df_nodes["Lon"].to_list()
loc_y = df_nodes["Lat"].to_list()

# plt.scatter(loc_x[1:], loc_y[1:], c='b')
# for i in N:
#     plt.annotate('$q_%d=%d$'%(i,q[i]), (loc_x[i]+2, loc_y[i]))
# plt.plot(loc_x[0], loc_y[0], c='r', marker='s')
# plt.axis('equal')
# plt.show()

A = [(i, j) for i in V for j in V if i != j]  # (0,1), (0,2) and so on - set of arcs
c_jk = {(i, j): np.hypot(loc_x[i]-loc_x[j], loc_y[i]-loc_y[j]) for i, j in A}  # set of costs between arcs - distances

# ADDITIONS (differ from the youtube vid)

R = []  # set of routes
for i in range(1, len(df_nodes)):
    R.append(i)

P = []  # set of P-LANEs
for i in range(1, df_routes['To[P-LANE]'].nunique() + 1):  # number of unique values = number of P-LANEs
    P.append(i)

f = 5  # transportation cost per unit distance

b_jp = {}  # number of boxes needed from supplier j ∈ V\{0} on P-LANE p ∈ P (from supplier j ∈ N)
for row, content in df_routes.iterrows():
    b_jp[(content[0], content[1])] = content[2]

b_j = {}  # total number of boxes needed from supplier j in manufacturer’s one-day production
for j in N:
    sum_b_j = 0
    for p in P:
        sum_b_j += b_jp[(j, p)]
    b_j[j] = sum_b_j

# TODO: the average number of boxes collected from supplier j per each visit if supplier j is visited s times? Calculate that for every supplier j (just like b_j) - divide b_j by F

d_p = {p: random.uniform(8, 20) for p in P}  # the due date of P-LANE p. Time interval from 8 AM to 8 PM. TODO: make it actual times and part of dataset and not randomly generated. Issue - we don't have a dataset for P-LANEs

theta = 5  # earliness cost
psi = 5  # tardiness cost
omega = 5  # fixed cost per vehicle per visit

u_j = {}  # unloading time per box for the parts collected from supplier j (in seconds))
for row, content in df_nodes.iloc[1:].iterrows():
    u_j[row] = content[2]

F = {}  # set of visiting frequencies
for j in b_j:
    F[j] = list(range(1, math.ceil(b_j[j] / Q) + 1))  # maybe just use max of all suppliers instead
'''
Summary before the constraints and variables:
payload   payload of a vehicle
N         set of nodes without the depot (set of suppliers)
V         set of nodes with the depot (vertex set)
q         set of demands
A         set of arcs (arc set)
c         set of costs (distances) between arcs 
Missing:
set of routes
set of p-lane
'''

# 1) All P-PANEs lie in the depot
# 2) Make a routes file (demands from the supplier to the P-LANE)
# 3)

mdl = Model('CVRP')

# VARIABLES
x = mdl.binary_var_dict(A, name='x')  # do we select an arc
u = mdl.continuous_var_dict(N, ub=payload, name='u')  # Q is upperbound

# OBJECTIVE
mdl.minimize(mdl.sum(c[i, j]*x[i, j] for i, j in A))

# CONSTRAINTS
mdl.add_constraints(mdl.sum(x[i, j] for j in V if j != i) == 1 for i in N)
mdl.add_constraints(mdl.sum(x[i, j] for i in V if i != j) == 1 for j in N)
mdl.add_indicator_constraints(mdl.indicator_constraint(x[i, j], u[i]+q[j] == u[j]) for i, j in A if i != 0 and j != 0)
mdl.add_constraints(u[i] >= q[i] for i in N)

mdl.parameters.timelimit = 15

# SOLVE AND PRINT
solution = mdl.solve(log_output=True)
print(solution)
solution.solve_status
active_arcs = [a for a in A if x[a].solution_value > 0.9]
plt.scatter(loc_x[1:], loc_y[1:], c='b')
for i in N:
    plt.annotate('$q_%d=%d$' % (i, q[i]), (loc_x[i]+2, loc_y[i]))
for i, j in active_arcs:
    plt.plot([loc_x[i], loc_x[j]], [loc_y[i], loc_y[j]], c='g', alpha=0.3)
plt.plot(loc_x[0], loc_y[0], c='r', marker='s')
plt.axis('equal')
plt.show()





# # PREVIOUS VERSION
# import pandas as pd
# import cplex
# from instances.Tools import create_graph, print_solution
#
# df_nodes = pd.read_csv("data/nodes.txt")
#
# # PERMUTATION (if needed)
# # import random
# # for row, content in df_nodes.iterrows():
# #     if row == 0:
# #         df_nodes.iloc[row, 0] = int(random.uniform(0, 500))
# #         df_nodes.iloc[row, 1] = int(random.uniform(0, 500))
# #     else:
# #         df_nodes.iloc[row, 0] = int(random.uniform(0, 500))
# #         df_nodes.iloc[row, 1] = int(random.uniform(0, 500))
# #         df_nodes.iloc[row, 2] = int(random.uniform(300, 600))
# # df_nodes.to_csv('data/nodes_permutated.txt', index=False)
#
# # LOCATIONS (coordinates of nodes)
# locations = df_nodes.drop(columns=['Demand[kg]']).values.tolist()
#
# # DEMANDS (in kg)
# demands = df_nodes.drop(columns=['Lon', 'Lat']).values.tolist()
#
# # VEHICLES
# payload = 900
# vehicles = 6
#
# location_list = create_graph(locations, demands)  # call the function to create the locations. 7 locations (2 of which with 0 demand at the end and start represent depos)
#
# cpx = cplex.Cplex()
# cpx.parameters.timelimit.set(225.0)  # you can set the time limit in seconds
# cpx.objective.set_sense(cpx.objective.sense.minimize)
#
# x = {i: {j: "x_" + str(i) + "_" + str(j) for j in range(len(location_list))} for i in range(len(location_list))}
# # x - names of edges. x[0][1] for example will return us x_0_1
#
# y = {i: "y_" + str(i) for i in range(len(location_list))}
#
# # _____DECISION VARIABLES_____
# # edge decision variable - do we keep the edge
# for i in location_list:
#     for j in location_list:
#         if j.index != i.index:  # excluding the case when locations are the same (distance 0)
#             cpx.variables.add(obj=[i.distance[j]], types=["B"], names=[x[i.index][j.index]])
#             # [i.distance[j]] - distance between i and j
#             # [x[i.index][j.index]] - x[0][1] for example will return us x_0_1
#
# # print(cpx.variables.get_types())
# # print(len(location_list))
#
# # node decision variable - ???
# for i in location_list:
#     cpx.variables.add(lb=[i.demand[0]], ub=[payload], types=["C"], names=[y[i.index]])
#     # note that y variables are continuous
#
#
# # _____CONSTRAINTS_____
# # all nodes are visited exactly once
# for i in location_list:  # constraints 1 (2.2)
#     if i.index != 0 and i.index != len(location_list) - 1:  # excluding the depos
#         coef_1, var_1 = [], []  # what's that?
#         for j in location_list:
#             if j.index != i.index and j.index != 0:  # excluding the same locations + depos
#                 coef_1.append(1)  # appending just the number 1 for some reason
#                 var_1.append(x[i.index][j.index])  # appending the name of the edge
#         cpx.linear_constraints.add(lin_expr=[[var_1, coef_1]], senses=["E"], rhs=[1])  # if rhs is 1 then can for example a float 0.5 be a result?
#
# for h in location_list:  # constraints 2 (2.3)
#     if h.index != 0 and h.index != len(location_list) - 1:  # excluding the depos
#         coef_2, var_2 = [], []
#         for i in location_list:
#             if i.index != len(location_list) - 1 and i is not h:  # excluding the last node (depo) and the same nodes
#                 coef_2.append(1)
#                 var_2.append(x[i.index][h.index])
#         for j in location_list:
#             if j.index != 0 and j is not h:  # excluding the first node (depo) and the same nodes
#                 coef_2.append(-1)
#                 var_2.append(x[h.index][j.index])
#         cpx.linear_constraints.add(lin_expr=[[var_2, coef_2]], senses=["E"], rhs=[0])
#
# coef_3, var_3 = [], []  # constraints 3 (2.4)
# for j in location_list:
#     if j.index != 0 and j.index != len(location_list) - 1:  # excluding the first node (depo) and the same nodes
#         coef_3.append(1)
#         var_3.append(x[0][j.index])  # x_0_1, x_0_2 and so on
# cpx.linear_constraints.add(lin_expr=[[var_3, coef_3]], senses=["L"], rhs=[vehicles])
#
# for i in location_list:  # constraints 4 (2.5)
#     for j in location_list:
#         if j.index != i.index:
#             coef_4 = [1, -1, -j.demand[0] - payload]
#             var_4 = [y[j.index], y[i.index], x[i.index][j.index]]
#             cpx.linear_constraints.add(lin_expr=[[var_4, coef_4]], senses=["G"], rhs=[-payload])
#
# print_solution(cpx, location_list, x, y)  # solve the model and print the solution