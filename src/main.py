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
#     else:
#         df_nodes.iloc[row, 0] = random.uniform(0, 500)
#         df_nodes.iloc[row, 1] = random.uniform(0, 500)
#         df_nodes.iloc[row, 2] = round(random.uniform(0.3, 0.5), 1)
# df_nodes.to_csv('data/nodes_permutated.txt', index=False)



Q = 10  # vehicle capacity in boxes
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
c = {(i, j): np.hypot(loc_x[i]-loc_x[j], loc_y[i]-loc_y[j]) for i, j in A}  # set of costs between arcs - distances

# ADDITIONS (differ from the youtube vid)

R = []  # set of routes
for i in range(1, len(df_nodes)):
    R.append(i)

P = []  # set of P-LANEs
for i in range(1, df_routes['To[P-LANE]'].nunique() + 1):  # number of unique values = number of P-LANEs
    P.append(i)

b_jp = {}  # number of boxes needed from supplier j ∈ V\{0} on P-LANE p ∈ P (from supplier j ∈ N)
for row, content in df_routes.iterrows():
    b_jp[(content[0], content[1])] = content[2]

b_j = {}  # total number of boxes needed from supplier j in manufacturer’s one-day production
for j in N:
    sum_b_j = 0
    for p in P:
        sum_b_j += b_jp[(j, p)]
    b_j[j] = sum_b_j

random.seed(0)
d = {p: int(random.uniform(6, 24)) * 60 for p in P}  # the due date of P-LANE p (in minutes)
# TODO: make it actual times and part of dataset and not randomly generated. Issue - we don't have a dataset for P-LANEs

f = 5  # transportation cost per unit distance
theta = 5  # earliness cost
psi = 5  # tardiness cost
omega = 5  # fixed cost per vehicle per visit

U = {}  # unloading time per box for the parts collected from supplier j (in seconds)
for row, content in df_nodes.iloc[1:].iterrows():
    U[row] = content[2]

# F = {}  # set of visiting frequencies (old way)
# for j in b_j:
#     F[j] = math.ceil(b_j[j] / Q)  # !!! not exactly like in the base model (not using range)
#     # F[j] = list(range(1, math.ceil(b_j[j] / Q) + 1))  # all visiting frequencies (old version)
F = []
F.extend(list(range(1, math.ceil(b_j[j] / Q) + 1)))  # all visiting frequencies (old version)

nu = {}  # the average number of boxes collected from supplier j per each visit if supplier j is visited s times
for j in range(1, len(b_j) + 1):
    for s in F:
        nu[(j, s)] = b_j[j]/s

'''
Summary before the constraints and variables:
Q         vehicle capacity in boxes
N         set of nodes without the depot (set of suppliers)
V         set of nodes with the depot (vertex set)
A         set of arcs (arc set)
c         set of costs (distances) between arcs
R         set of routes
P         set of P-LANEs
b_jp      number of boxes needed from supplier j ∈ V\{0} on P-LANE p ∈ P (from supplier j ∈ N)
b_j       total number of boxes needed from supplier j in manufacturer’s one-day production 
d         the due date of P-LANE p. Time interval from 0 to 12 (check the related todo!)
f         transportation cost per unit distance
theta     earliness cost
psi       tardiness cost
omega     fixed cost per vehicle per visit
U         unloading time per box for the parts collected from supplier j (in seconds))
F         set of visiting frequencies
nu        the average number of boxes collected from supplier j per each visit if supplier j is visited s times
'''
# Note that we can't use just b because there are b_jp and b_j (same later on for A, F
# TODO: possible issue - p, j, s, etc. are already formally defined above in for loops

mdl = Model('CVRP')

# VARIABLES
x = mdl.binary_var_dict(R, name='x')  # do we select a route
y = mdl.binary_var_dict(((j, r) for j in V for r in R), name='y')  # is the supplier assigned !!! to route?
z = mdl.binary_var_dict(((j, k, r) for j, k in A for r in R), name='z')  # is arc (j, k) visited !!! by route r?
u = mdl.binary_var_dict(((r, s) for r in R for s in F), name='u')  # is frequency of route r s? Lukas: because of that I think its easier when F is just a list going from 1 to the largest possible visting frequency for any supplier
D = mdl.continuous_var_dict(((r, s) for r in R for s in F), name='D')  # the departure time of the sth visit from the manufacturer for route r Lukas: Frequency again
A_rs = mdl.continuous_var_dict(((r, s) for r in R for s in F), name='A_rs')  # the arrival time of the sth visit from the manufacturer for route r. Note that we can't use just A because A is reserved for arc set
sigma = mdl.binary_var_dict(((j, r, s) for j in V for r in R for s in F), name='sigma')  # is equal to 1 if y_jr = 1, u_rs = 1 and 0 otherwise
delta = mdl.binary_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name='delta')  # is equal to 1 if z_jkr = 1, u_rs = 1 and 0 otherwise
epsilon = mdl.binary_var_dict(((j, r, s, t, p) for j in V for r in R for s in F for t in F for p in P), name='epsilon_jrstp')  # is equal to 1 if delta_jrs = 1 and t-th visit of route r meet the demand needed from supplier j on P-LANE p and 0 otherwise
F_jp = mdl.continuous_var_dict(((j, p) for j in V for p in P), name='F_jp')  # is the finish time when the parts collected from supplier j meet the demand on P-LANE p. Note that we can't use just F because F is reserved for set of frequencies
E = mdl.continuous_var_dict(((j, p) for j in V for p in P), name='E')  # is the earliness of supplier j on P-LANE p
T = mdl.continuous_var_dict(((j, p) for j in V for p in P), name='T')  # is the tardiness time of supplier j on P-LANE p
M = 10e10  # sufficiently big number Lukas: might be a little bit too big, reducing it would be benefical for solving time, issue of parameter tuning

# OBJECTIVE
mdl.minimize(mdl.sum(f * s * c[j, k] * delta[j, k, r, s] for s in F for r in R for k in V for j in V if j != k) +
             mdl.sum(omega * s * u[r, s] for s in F for r in R) +
             mdl.sum(theta * E[j, p] for p in P for j in N) +
             mdl.sum(psi * T[j, p] for p in P for j in N))

# CONSTRAINTS
mdl.add_constraints(x[r + 1] <= x[r] for r in range(1, len(R)-1))  # 1b
mdl.add_constraints(mdl.sum(y[j, r] for j in N) == 1 for r in R)  # 1c !!! the sequence should be correct
mdl.add_constraints(mdl.sum(y[j, r] for j in N) >= x[r] for r in R)  # 1d
mdl.add_constraints(mdl.sum(z[j, k, r] for k in V if k != j) == y[j, r] for j in N for r in R)  # 1e
mdl.add_constraints(mdl.sum(z[j, k, r] for j in V if j != k) == y[k, r] for k in N for r in R)  # 1f
# TODO: 1g
mdl.add_constraints(mdl.sum(u[r, s] for s in F) == x[r] for r in R)  # 1h
mdl.add_constraints(mdl.sum(b_j[j] * y[j, r] for j in N) <= mdl.sum(s * Q * u[r, s] for s in F) for r in R)  # 1i
mdl.add_constraints(mdl.sum(b_j[j] * y[j, r] for j in N) >= mdl.sum(((s - 1) * Q + 1) * u[r, s] for s in F) for r in R)  # 1j
mdl.add_constraints(2 * delta[j, k, r, s] <= z[j, k, r] + u[r, s] for j in V for k in V if k != j for r in R for s in F)  # 1k
# about the constraint 1k TypeError: cannot unpack non-iterable int object. Reason: for j, k in V -> for j in V for k in V
#  also KeyError: (0, 0, 1, 1) -> means the pair 0, 0 should not exist (distance between the same node) -> if j != k
mdl.add_constraints(1 + delta[j, k, r, s] >= z[j, k, r] + u[r, s] for j in V for k in V if k != j for r in R for s in F)  # 1l
mdl.add_constraints(2 * sigma[j, r, s] <= y[j, r] + u[r, s] for j in N for r in R for s in F)  # 1m TODO: is k in sigma indexes a mistake in a paper? -> yes, the same for sigma in 1n
mdl.add_constraints(1 + sigma[j, r, s] >= y[j, r] + u[r, s] for j in N for r in R for s in F)  # 1n
mdl.add_constraints(mdl.sum(t * nu[j, s] * epsilon[j, r, s, t, p] for t in range(1, s + 1)) >= mdl.sum(b_jp[j, k] * sigma[j, r, s] for k in range(1, p + 1)) for j in N for r in R for s in F for p in P)  # 1o TODO: not sure if that's a correct implementation of sum from 1 to s. Plus is b_jk correct and not a mistake in a paper? -> looks good to me, b_jk is correct as stated in the paper (and the implementation)
mdl.add_constraints(mdl.sum(((t - 1) * nu[j, s] + 1) * epsilon[j, r, s, t, p] for t in range(1, s + 1)) <= mdl.sum(b_jp[j, k] * sigma[j, r, s] for k in range(1, p + 1)) for j in N for r in R for s in F for p in P)  # 1p
mdl.add_constraints(mdl.sum(epsilon[j, r, s, t, p] for t in range(1, s + 1)) == sigma[j, r, s] for j in N for r in R for s in F for p in P)  # 1q TODO: what is the set L? Should be F? -> yes should be F
mdl.add_constraints(A_rs[r, s] >= D[r, s] + mdl.sum(c[j, k] * z[j, k, r] for k in V for j in V if j != k) + mdl.sum(U[j] * nu[j, t] * sigma[j, r, t] for t in F for j in N) - M * mdl.sum(u[r, t] for t in range(1, s + 1)) for r in R for s in F)  # 1r TODO: KeyError: (1, 0). Reason: why u[r, t] can be [1, 0] according to the model? Should sum be from t = 1 to s? range(s) -> range(1, s + 1) -> I think the t=0 is a mistake, but going only until s-1 is correct, otherwise the Big-M formulation would also not work properly -> range(1, s) should be correct
# TODO: 1s can be implemented by changing C_rt to A_rs
# TODO: 1t can be implemented by changing C_rt to A_rs
mdl.add_constraints(E[j, p] >= d[p] - F_jp[j, p] for j in N for p in P)  # 1u
mdl.add_constraints(T[j, p] >= F_jp[j, p] - d[p] for j in N for p in P)  # 1u

# mdl.parameters.timelimit = 15

# SOLVE AND PRINT
solution = mdl.solve(log_output=True)
print(solution)
solution.solve_status

# EVERYTHING NEXT HASN'T BEEN DONE YET

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