import numpy as np
import matplotlib.pyplot as plt

rnd = np.random
rnd.seed(0)

n = 10
Q = 20
N = [i for i in range(1, n+1)]
V = [0] + N
q = {i: rnd.randint(1, 10) for i in N}

loc_x = rnd.rand(len(V)) * 200
loc_y = rnd.rand(len(V)) * 100

plt.scatter(loc_x[1:], loc_y[1:], c='b')
for i in N:
    plt.annotate('$q_%d=%d$'%(i,q[i]), (loc_x[i]+2, loc_y[i]))
plt.plot(loc_x[0], loc_y[0], c='r', marker='s')
plt.axis('equal')
plt.show()

A = [(i, j) for i in V for j in V if i != j]  # (0,1), (0,2) and so on - arcs
c = {(i, j): np.hypot(loc_x[i]-loc_x[j], loc_y[i]-loc_y[j]) for i, j in A}  # cost between arcs - distance

from docplex.mp.model import Model

mdl = Model('CVRP')

# VARIABLES
x = mdl.binary_var_dict(A, name='x')  # do we select an arc
u = mdl.continuous_var_dict(N, ub=Q, name='u')  #

# OBJECTIVE
mdl.minimize(mdl.sum(c[i, j]*x[i, j] for i, j in A))

# CONSTRAINTS
mdl.add_constraints(mdl.sum(x[i, j] for j in V if j != i) == 1 for i in N)
mdl.add_constraints(mdl.sum(x[i, j] for i in V if i != j) == 1 for j in N)
mdl.add_indicator_constraints(mdl.indicator_constraint(x[i, j], u[i]+q[j] == u[j]) for i, j in A if i != 0 and j != 0)
mdl.add_constraints(u[i] >= q[i] for i in N)
mdl.parameters.timelimit = 15

# SOLUTION
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

'''
Adapted
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from docplex.mp.model import Model

df_nodes = pd.read_csv("data/nodes.txt")

# # PERMUTATION (if needed)
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

# VEHICLES
payload = 900

N = []  # set of nodes without the depot
for i in range(1, len(df_nodes)):
    N.append(i)
V = [0] + N  # set of nodes with the depot
q = {i: df_nodes.iloc[i, 2] for i in N}  # set of demands

loc_x = df_nodes["Lon"].to_list()
loc_y = df_nodes["Lat"].to_list()

# plt.scatter(loc_x[1:], loc_y[1:], c='b')
# for i in N:
#     plt.annotate('$q_%d=%d$'%(i,q[i]), (loc_x[i]+2, loc_y[i]))
# plt.plot(loc_x[0], loc_y[0], c='r', marker='s')
# plt.axis('equal')
# plt.show()

A = [(i, j) for i in V for j in V if i != j]  # (0,1), (0,2) and so on - set of arcs
c = {(i, j): np.hypot(loc_x[i]-loc_x[j], loc_y[i]-loc_y[j]) for i, j in A}  # set of costs between arcs - distances



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
'''