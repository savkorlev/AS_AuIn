import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import random
import math
from docplex.mp.model import Model
from itertools import combinations

peri = 20000    # perimeter, in which the suppliers are arranged around the manufacturer (in meters!)

random.seed(0)
num_supp = 5
supp_list = [[peri/2, peri/2, 0]]   # manufacturer is in the middle
for i in range(1,num_supp+1):
    x = int(random.uniform(0, peri))
    y = int(random.uniform(0, peri))
    t = round(random.uniform(0.3, 0.5), 2)
    supp_list.append([x, y, t])
df_nodes = pd.DataFrame(supp_list, columns = ['x', 'y', 'Unloading Times[min]'])

#df_nodes = pd.read_csv("data/nodes.txt")
df_routes = pd.read_csv("data/routes.txt")
#df_nodes = df_nodes.iloc[:8]
df_routes = df_routes.iloc[:(num_supp*4 + 1)]


Q = 20  # vehicle capacity in boxes/paletts
N = []  # set of nodes without the depot
for i in range(1, len(df_nodes)):
    N.append(i)
V = [0] + N  # set of nodes with the depot

S = []   # set of all possible subtours without the depot (needed for constraint 1g)
for L in range(2, len(N)+1):
    for subset in combinations(N, L):
        S.append(subset)

loc_x = df_nodes["x"].to_list()
loc_y = df_nodes["y"].to_list()

# PLOT DIAGRAM OF NODES
plt.scatter(loc_x[1:], loc_y[1:], c='b')
for i in range(len(loc_x[1:])):
    plt.text(loc_x[i+1], loc_y[i+1]+1000, str(i+1))
plt.plot(loc_x[0], loc_y[0], c='r', marker='s')
plt.text(loc_x[0], loc_y[0]+1000,'0')

plt.axis('equal')
plt.show()

A = [(i, j) for i in V for j in V if i != j]  # (0,1), (0,2) and so on - set of arcs
c = {(i, j): np.hypot(loc_x[i]-loc_x[j], loc_y[i]-loc_y[j]) for i, j in A}  # set of costs between arcs - distances


R = []  # set of routes
for i in range(1, len(df_nodes)):
    R.append(i)


P = []  # set of P-LANEs
for i in range(1, df_routes['To[P-LANE]'].nunique() + 1):  # number of unique values = number of P-LANEs
    P.append(i)

b_jp = {}  # number of boxes needed from supplier j ∈ V\{0} on P-LANE p ∈ P (from supplier j ∈ N)
for row, content in df_routes.iterrows():
    b_jp[(content[0], content[1])] = content[2]

b_j = {}  # total number of paletts needed from supplier j in manufacturer’s one-day production
for j in N:
    sum_b_j = 0
    for p in P:
        sum_b_j += b_jp[(j, p)]
    b_j[j] = sum_b_j


d = {p: int(random.uniform(6, 24)) * 60 for p in P}  # the due date of P-LANE p (in minutes)
# TODO: make it actual times and part of dataset and not randomly generated. Issue - we don't have a dataset for P-LANEs

phi = 5  # transportation cost per unit distance
omega = 40  # fixed cost per vehicle per visit

U = {}  # unloading time per palett for the parts collected from supplier j (in seconds)
for row, content in df_nodes.iloc[1:].iterrows():
    U[row] = content[2]

F = []
F.extend(list(range(1, math.ceil(sum(b_j.values()) / Q) + 1)))

nu = {}  # the average number of boxes collected from supplier j per each visit if supplier j is visited s times
for j in range(1, len(b_j) + 1):
    for s in F:
        nu[(j, s)] = b_j[j]/s

############################ Extensions #############################################

L = [1, 2, 3]  # set of average speed levels (indices)

v = {1: 670, 2: 830, 3:1000}  # set of actual speed levels (40, 50 and 60 km/h in [m/minute])

c_e = 0.06453 * 10 ** (-6)    # electricity costs [€/J]
c_c = 0.0359 * 10 ** (-6) # fuel costs [€/J]
o_e = 325.20    # costs per electric truck [€/day]
o_c = 271.20    # costs per ICE truck [€/day]
alpha = 0.0981  # arc specific energy consumption constant [m/s2]
beta = 3.6123   # vehicle specific energy consumption constant [kg/m]
gamma_c = 12    # correction factor for energy consumption for ICE truck (efficiency, simplifications etc.)
gamma_e = 3.75     # correction factor for energy consumption for electric truck (efficiency, simplifications etc.)
C_e = 1857.6 * 10 ** 6  # electric energy for electric vehicle [J] (=range of 400km)
C_c = 17820 * 10 ** 6    # fuel energy for ICE truck [J] (=range of 1500km)
e = 0.02193 * 10 ** (-7)    # CO2 costs [€/J]
m_e = 4300  # empty weight of electric truck
m_c = 2200  # empty weight of ICE truck
pi = 30.96 * 10 ** 6    # recharging constant for electric truck [J/minute]


l_js = {}  # the average load [kg] collected from supplier j per each visit if supplier j is visited s times
for j in range(1, len(b_j) + 1):
    load_per_palette = int(random.uniform(400, 600))
    for s in F:
        l_js[(j, s)] = b_j[j]/s * load_per_palette


######################################################################################


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
phi       transportation cost per unit distance
theta     earliness cost
psi       tardiness cost
omega     fixed cost per vehicle per visit
U         unloading time per box for the parts collected from supplier j (in seconds))
F         set of visiting frequencies
nu        the average number of boxes collected from supplier j per each visit if supplier j is visited s times
'''
# Note that we can't use just b because there are b_jp and b_j (same later on for A, F)
# TODO: possible issue - p, j, s, etc. are already formally defined above in for loops

mdl = Model('CVRP')

# VARIABLES
x = mdl.binary_var_dict(R, name='x')  # do we select a route
y = mdl.binary_var_dict(((j, r) for j in V for r in R), name='y')  # is the supplier assigned to route? is "for j in N" correct or should it be V? Same applies for sigma, epsilon, F_jp, E and T
z = mdl.binary_var_dict(((j, k, r) for j, k in A for r in R), name='z')  # is arc (j, k) visited by route r?
u = mdl.binary_var_dict(((r, s) for r in R for s in F), name='u')  # is frequency of route r s? Lukas: because of that I think its easier when F is just a list going from 1 to the largest possible visting frequency for any supplier
D = mdl.continuous_var_dict(((r, s) for r in R for s in F), name='D')  # the departure time of the sth visit from the manufacturer for route r Lukas: Frequency again
sigma = mdl.binary_var_dict(((j, r, s) for j in V for r in R for s in F), name='sigma')  # is equal to 1 if y_jr = 1, u_rs = 1 and 0 otherwise
delta = mdl.binary_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name='delta')  # is equal to 1 if z_jkr = 1, u_rs = 1 and 0 otherwise
M = 10e10  # sufficiently big number Lukas: might be a little bit too big, reducing it would be benefical for solving time, issue of parameter tuning


####################### Extensions ######################################################

f_e = mdl.continuous_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name='f_e') # weight of boxes transported on arc (j, k)
f_c = mdl.continuous_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name='f_c') # weight of boxes transported on arc (j, k) on route r with frequency s and an ICE truck
t_e = mdl.binary_var_dict(R, name='t_e')    # = 1 if an electric truck is used in route r
t_c = mdl.binary_var_dict(R, name='t_c')    # = 1 if an ICE truck is used in route r
g_e = mdl.binary_var_dict(((j, k, r, l, s) for j, k in A for r in R for s in F for l in L), name='g_e') # = 1, if average speed on route r with frequency s on arc (j,k) is speed level l with an electric truck
g_c = mdl.binary_var_dict(((j, k, r, l, s) for j, k in A for r in R for s in F for l in L), name='g_c') # = 1, if average speed on route r with frequency s on arc (j,k) is speed level l with an ICE truck
sigma_e = mdl.binary_var_dict(((j, r, s) for j in V for r in R for s in F), name='sigma_e')  # is equal to 1 if y_jr = 1, u_rs = 1, t_e = 1 and 0 otherwise
sigma_c = mdl.binary_var_dict(((j, r, s) for j in V for r in R for s in F), name='sigma_c')  # is equal to 1 if y_jr = 1, u_rs = 1, t_c = 1 and 0 otherwise
E_e = mdl.continuous_var_dict(((j, k) for j, k in A), name = 'E_e')
E_c = mdl.continuous_var_dict(((j, k) for j, k in A), name = 'E_c')
delta_e = mdl.binary_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name = 'delta_e')
delta_c = mdl.binary_var_dict(((j, k, r, s) for j, k in A for r in R for s in F), name = 'delta_c')

####################################################################################################################

# CONSTRAINTS
# 1b
mdl.add_constraints((x[r + 1] <= x[r] for r in range(1, len(R))), names = '1b')

# 1c
mdl.add_constraints((mdl.sum(y[j, r] for r in R) == 1 for j in N), names = '1c')

# 1d
mdl.add_constraints((mdl.sum(y[j, r] for j in N) >= x[r] for r in R), names = '1d')

# 1e
mdl.add_constraints((mdl.sum(z[j, k, r] for k in V if k != j) == y[j, r] for j in N for r in R), names = '1e')
# if k != j because the distance between the same arcs doesn't exist (KeyError)

# 1f
mdl.add_constraints((mdl.sum(z[j, k, r] for j in V if j != k) == y[k, r] for k in N for r in R), names = '1f')
# if j != k because the distance between the same arcs doesn't exist (KeyError)

# 1g
mdl.add_constraints((mdl.sum(z[j, k, r] for j in subtour for k in subtour if j != k) <= len(subtour)-1 for subtour in S for r in R), names = '1g')

# 1h
mdl.add_constraints((mdl.sum(u[r, s] for s in F) == x[r] for r in R), names = '1h')

# 1i
mdl.add_constraints((mdl.sum(b_j[j] * y[j, r] for j in N) <= mdl.sum(s * Q * u[r, s] for s in F) for r in R), names = '1i')

# 1j
mdl.add_constraints((mdl.sum(b_j[j] * y[j, r] for j in N) >= mdl.sum(((s - 1) * Q + 1) * u[r, s] for s in F) for r in R), names = '1j')

# 1k
mdl.add_constraints((2 * delta[j, k, r, s] <= z[j, k, r] + u[r, s] for j in V for k in V if k != j for r in R for s in F), names = '1k')
# TypeError: cannot unpack non-iterable int object. Reason: for j, k in V -> for j in V for k in V
# if k != j because the distance between the same arcs doesn't exist (KeyError)

# 1l
mdl.add_constraints((1 + delta[j, k, r, s] >= z[j, k, r] + u[r, s] for j in V for k in V if k != j for r in R for s in F), names = '1l')

# 1m
mdl.add_constraints((2 * sigma[j, r, s] <= y[j, r] + u[r, s] for j in N for r in R for s in F), names = '1m')
# Q: is k in sigma indexes a mistake in a paper? A: yes, the same for sigma in 1n

# 1n
mdl.add_constraints((1 + sigma[j, r, s] >= y[j, r] + u[r, s] for j in N for r in R for s in F), names = '1n')


############################### Extensions #################################################################

# load constraint 1
mdl.add_constraints((mdl.sum(f_e[j, k, r, s] for k in V if k != j) - mdl.sum(f_e[k , j, r, s] for k in V if k != j) == l_js[j, s] * sigma_e[j, r, s] for r in R for s in F for j in N), names = 'load-constraint e1')

# load constraint 2
mdl.add_constraints((mdl.sum(f_c[j, k, r , s] for k in V if k != j) - mdl.sum(f_c[k, j, r, s] for k in V if k != j) == l_js[j, s] * sigma_c[j, r, s] for r in R for s in F for j in N), names = 'load-constraint c1')

# load constraint 3
mdl.add_constraints(f_e[j, k, r, s] <= max(l_js.values()) * delta_e[j, k, r, s] for r in R for j, k in A for s in F)

# load constraint 4
mdl.add_constraints(f_c[j, k, r, s] <= max(l_js.values()) * delta_c[j, k, r, s] for r in R for j, k in A for s in F)

# truck type constraint
mdl.add_constraints(t_e[r] + t_c[r] == x[r] for r in R)

# sigma_e constraint 1
mdl.add_constraints(2 * sigma_e[j, r, s] <= sigma[j, r, s] + t_e[r] for r in R for j in V for s in F)

# sigma_c constraint 1
mdl.add_constraints(2 * sigma_c[j, r, s] <= sigma[j, r, s] + t_c[r] for r in R for j in V for s in F)

# sigma_e constraint 2
mdl.add_constraints(1 + sigma_e[j, r, s] >= sigma[j, r, s] + t_e[r] for r in R for j in V for s in F)

# sigma_c constraint 2
mdl.add_constraints(1 + sigma_c[j, r, s] >= sigma[j, r, s] + t_c[r] for r in R for j in V for s in F)

# g_e constraint 1
mdl.add_constraints(2 * mdl.sum(g_e[j, k, r, l, s] for l in L) <= delta[j, k, r, s] + t_e[r] for j, k in A for r in R for s in F)

# g_c constraint 1
mdl.add_constraints(2 * mdl.sum(g_c[j, k, r, l, s] for l in L) <= delta[j, k, r, s] + t_c[r] for j, k in A for r in R for s in F)

# g_e constraint 2
mdl.add_constraints(1 + mdl.sum(g_e[j, k, r, l, s] for l in L) >= delta[j, k, r, s] + t_e[r] for j, k in A for r in R for s in F)

# g_c constraint 2
mdl.add_constraints(1 + mdl.sum(g_c[j, k, r, l, s] for l in L) >= delta[j, k, r, s] + t_c[r] for j, k in A for r in R for s in F)

# ICE range constraint
mdl.add_constraints(mdl.sum(gamma_c * s *(alpha * c[j, k] * (f_c[j, k, r, s] + m_c * delta_c[j, k, r, s]) + beta * c[j, k] * mdl.sum(((v[l]/60) ** 2) * g_c[j, k, r, l, s] for l in L)) for j, k in A for r in R for s in F) <= C_c for r in R for s in F)

# electric range constraint
mdl.add_constraints(mdl.sum(gamma_e * s *(alpha * c[j, k] * (f_e[j, k, r, s] + m_e * delta_e[j, k, r, s]) + beta * c[j, k] * mdl.sum(((v[l]/60) ** 2) * g_e[j, k, r, l, s] for l in L)) for j, k in A for r in R for s in F) <= C_e + (240 - mdl.sum((c[j, k] / v[l]) * g_e[j, k, r, l, s] * s for j, k in A for l in L) - mdl.sum(U[j]*nu[j, s]*s*sigma_e[j, r, s] for j in N)) * pi for r in R for s in F)

# time constraint for electric trucks
mdl.add_constraints(mdl.sum((c[j, k]/v[l]) * g_e[j, k, r, l, s]*s for j, k in A for l in L) + mdl.sum(U[j]*nu[j, s]*s*sigma_e[j, r, s] for j in N) <= 240 for r in R for s in F)


# time constraint for ICE trucks
mdl.add_constraints(mdl.sum((c[j, k]/v[l]) * g_c[j, k, r, l, s]*s for j, k in A for l in L) + mdl.sum(U[j]*nu[j, s]*s*sigma_c[j, r, s] for j in N) <= 240 for r in R for s in F)


# delta_e constraint 1
mdl.add_constraints(2 * delta_e[j, k, r, s] <= delta[j, k, r, s] + t_e[r] for j, k in A for r in R for s in F)

# delta_e constraint 2
mdl.add_constraints(1 + delta_e[j, k, r, s] >= delta[j, k, r, s] + t_e[r] for j, k in A for r in R for s in F)

# delta_c constraint 1
mdl.add_constraints(2 * delta_c[j, k, r, s] <= delta[j, k, r, s] + t_c[r] for j, k in A for r in R for s in F)

# delta_c constraint 2
mdl.add_constraints(1 + delta_c[j, k, r, s] >= delta[j, k, r, s] + t_c[r] for j, k in A for r in R for s in F)

# additional flow constraint 1
mdl.add_constraints(mdl.sum(z[0, k, r] for k in N) == x[r] for r in R)

# limit number of trucks of specific type
# mdl.add_constraints(mdl.sum(t_e[r] for r in R) <= 0 for r in R)   # no electric truck can be used

# force multiple routes
# mdl.add_constraint(mdl.sum(x[r] for r in R) == 2)

# force an arc to be used
# mdl.add_constraint(mdl.sum(z[3, 2, r] for r in R) == 1)

########################################################################################################

# SOLVE AND PRINT (+ EPSILON CONSTRAINT)
mdl.parameters.mip.tolerances.integrality = 0   # forces the binary variables to be excatly 1 or 0 not like 0.999999 or 0.00000003, might lead to a false infeasibility -> increase value slightly then (default = 1e-05); necesarry for Big M formulation of load constraints
# solution = mdl.solve(log_output=True)

################################### Extension objective function ##################################################

# mdl.minimize(
#              mdl.sum(omega * s * u[r, s] for r in R for s in F) +
#              mdl.sum(((f_e[j, k, r, s] + m_e * delta_e[j, k, r, s]) * c[j, k]) * alpha * c_e * gamma_e * s for j, k in A for r in R for s in F) +
#              mdl.sum(((f_c[j, k, r, s] + m_c * delta_c[j, k, r, s]) * c[j, k]) * alpha * (c_c + e) * gamma_c * s for j, k in A for r in R for s in F) +
#              mdl.sum((mdl.sum((v[l]/60) ** 2 * g_e[j, k, r, l, s] for l in L) * c[j, k] * beta) * c_e * gamma_e * s for j, k in A for r in R for s in F) +
#              mdl.sum((mdl.sum((v[l]/60) ** 2 * g_c[j, k, r, l, s] for l in L) * c[j, k] * beta) * (c_c + e) * gamma_c * s for j, k in A for r in R for s in F) +
#              mdl.sum(t_e[r] for r in R) * o_e +
#              mdl.sum(t_c[r] for r in R) * o_c)
# # Total costs: 325
# # Total [kWh]: 368
# # solution.get_objective_value() = 610.7

# truck costs + visits
obj_tr_vis = mdl.sum(omega * s * u[r, s] for r in R for s in F) + \
           mdl.sum(t_e[r] for r in R) * o_e + \
           mdl.sum(t_c[r] for r in R) * o_c
# # Total costs: 616
# # Total [kWh]: 1057
# # solution.get_objective_value() = 471.2

# pollution/fuel costs
obj_pol_fuel = mdl.sum(((f_e[j, k, r, s] + m_e * delta_e[j, k, r, s]) * c[j, k]) * alpha * c_e * gamma_e * s for j, k in A for r in R for s in F) + \
           mdl.sum(((f_c[j, k, r, s] + m_c * delta_c[j, k, r, s]) * c[j, k]) * alpha * (c_c + e) * gamma_c * s for j, k in A for r in R for s in F) + \
           mdl.sum((mdl.sum((v[l]/60) ** 2 * g_e[j, k, r, l, s] for l in L) * c[j, k] * beta) * c_e * gamma_e * s for j, k in A for r in R for s in F) + \
           mdl.sum((mdl.sum((v[l]/60) ** 2 * g_c[j, k, r, l, s] for l in L) * c[j, k] * beta) * (c_c + e) * gamma_c * s for j, k in A for r in R for s in F)
# # Total costs: 1938
# # Total [kWh]: 130
# # solution.get_objective_value() = 29.6

payoff = {}

mdl.minimize_static_lex([obj_tr_vis, obj_pol_fuel])
solution = mdl.solve(log_output=True)
payoff["transport"] = solution.get_objective_value()

mdl.minimize_static_lex([obj_pol_fuel, obj_tr_vis])
solution = mdl.solve(log_output=True)
payoff["pollution"] = solution.get_objective_value()

print('Each iteration will keep values lower than some values between min and max')

l = 10  # important for step
r = payoff["pollution"][1] - payoff["transport"][0]
step = r / l
steps = list(np.arange(payoff["transport"][0], payoff["pollution"][1], step)) + [payoff["pollution"][1]]

# obj_values = []
# mdl.e = 0
# mdl.delta = 0.0001
# mdl.s = mdl.continuous_var(name="mdl_s")
# mdl.minimize_static_lex([obj_pol_fuel - mdl.delta * mdl.s, obj_tr_vis])
# mdl.add_constraint(obj_tr_vis - mdl.s == mdl.e)
# for s in steps:
#     mdl.e = s
#     solution = mdl.solve(log_output=True)
#     obj_values.append(solution.get_objective_value())

obj_values = []
e_c_delta = 0.00001
e_c_s = mdl.continuous_var(name="e_c_s")
for s in steps:
    mdl.minimize_static_lex([obj_pol_fuel - e_c_delta * e_c_s, obj_tr_vis])
    mdl.add_constraint(obj_tr_vis - e_c_s == s, ctname='e_c_constr')
    solution = mdl.solve(log_output=True)
    obj_values.append(solution.get_objective_value())  # append also solution but for now just objective values to compare
    mdl.remove_constraint('e_c_constr')

# VISUALIZE THE RESULTS

plt.scatter(loc_x[1:], loc_y[1:], c='b')
for i in range(len(loc_x[1:])):
    plt.text(loc_x[i+1], loc_y[i+1]+1000, str(i+1))
plt.plot(loc_x[0], loc_y[0], c='r', marker='s')
plt.text(loc_x[0], loc_y[0]+1000,'0')
plt.axis('equal')


for j in V:
    for k in V:
        for r in R:
            if k != j:
                if solution.get_value(z[j, k, r]) > 0.5:
                    plt.plot([loc_x[j], loc_x[k]], [loc_y[j], loc_y[k]], linestyle='solid',color='black')

plt.show()

# PRINT OUT A SUMMARY OF THE RESULTS

for r in R:
    if solution.get_value(x[r]) == 1:
        print('\nRoute ' + str(r))
        sup = []
        for j in N:
            if solution.get_value(y[j, r]) == 1:
                sup.append(j)
        print(' Suppliers:' + str(sup))
        for s in F:
            if solution.get_value(u[r, s]) == 1:
                freq = s
        print(' Frequency: ' + str(freq))
        if solution.get_value(t_e[r]) == 1:
            print(' Vehicle type: electric')
        if solution.get_value(t_c[r]) == 1:
            print(' Vehicle type: conventional')
        distance = 0
        for j in V:
            for k in V:
                for s in F:
                    if k != j:
                        if (solution.get_value(delta[j, k, r, s]) == 1):
                            distance += (c[j, k] * s) / 1000
        print(' Driven distance in total [km]: ' + str(round(distance, 2)))
        time_driving = 0
        time_unloading = 0
        for s in F:
            for j, k in A:
                for l in L:
                    if j != k:
                        if (solution.get_value(g_e[j, k, r, l, s]) == 1):
                            time_driving += (c[j, k]/v[l]) * s
                        if (solution.get_value(g_c[j, k, r, l, s]) == 1):
                            time_driving += (c[j, k]/v[l]) * s
        for s in F:
            for j in N:
                if (solution.get_value(sigma_e[j, r, s]) == 1):
                    time_unloading += U[j]*nu[j, s] * s
                if (solution.get_value(sigma_c[j, r, s]) == 1):
                    time_unloading += U[j]*nu[j, s] * s
        print(' Needed time:')
        print('     In total (all trips) [min]: ' + str(round(time_driving + time_unloading)))
        print('     Driving (all trips) [min]: ' + str(round(time_driving)))
        print('     Loading (all trips) [min]: ' + str(round(time_unloading)))
        print(' Costs:')
        fuel_costs = 0
        weight_energy = 0
        for j in V:
            for k in V:
                if j != k:
                    for s in F:
                            if (solution.get_value(f_e[j, k, r, s]) > 0):
                                fuel_costs += solution.get_value(f_e[j, k, r, s]) * s * (c_e) * c[j, k] * alpha * gamma_e
                                weight_energy += solution.get_value(f_e[j, k, r, s]) * s * c[j, k] * alpha * gamma_e
                            if (solution.get_value(delta_e[j, k, r, s]) == 1):
                                fuel_costs += m_e * s * (c_e) * c[j, k] * alpha * gamma_e
                                weight_energy += m_e * s * c[j, k] * alpha * gamma_e
                            if (solution.get_value(f_c[j, k, r, s]) > 0):
                                fuel_costs += solution.get_value(f_c[j, k, r, s]) * s * (c_c + e) * c[j, k] * alpha * gamma_c
                                weight_energy += solution.get_value(f_c[j, k, r, s]) * s * c[j, k] * alpha * gamma_c
                            if (solution.get_value(delta_c[j, k, r, s]) == 1):
                                fuel_costs += m_c * s * (c_c + e) * c[j, k] * alpha * gamma_c
                                weight_energy += m_c * s * c[j, k] * alpha * gamma_c
        print('     Fuel/Electricity costs (weight) [€]: ' + str(round(fuel_costs, 2)))
        fuel_costs_2 = 0
        speed_energy = 0
        count = 0
        for j in V:
            for k in V:
                if j != k:
                    for s in F:
                        for l in L:
                            if (solution.get_value(g_e[j, k, r, l, s]) == 1):
                                fuel_costs_2 += (v[l]/60) ** 2 * c[j, k] * beta * (c_e) * gamma_e * s
                                speed_energy += (v[l]/60) ** 2 * c[j, k] * beta * gamma_e * s
                            if (solution.get_value(g_c[j, k, r, l, s]) == 1):
                                fuel_costs_2 += (v[l]/60) ** 2 * c[j, k] * beta * (c_c + e) * gamma_c * s
                                speed_energy += (v[l]/60) ** 2 * c[j, k] * beta * gamma_c * s
        truck_costs = 0
        if solution.get_value(t_e[r]) == 1:
            truck_costs += o_e
        if solution.get_value(t_c[r]) == 1:
            truck_costs += o_c
        costs_visit = 0
        for s in F:
            if solution.get_value(u[r, s]) == 1:
                costs_visit += omega * s
        print('     Fuel/Electricity costs (speed) [€]: ' + str(round(fuel_costs_2, 2)))
        print('     Fuel/Electricity costs (total) [€]: ' + str(round(fuel_costs + fuel_costs_2, 2)))
        print('     Fuel/Electricity costs (total per km) [€/km]: ' + str(round((fuel_costs + fuel_costs_2) / distance, 2)))
        print('     Truck costs [€]: ' + str(round(truck_costs, 2)))
        print('     Visits costs [€]: ' + str(round(costs_visit, 2)))
        print('     Total costs [€]: ' + str(round(fuel_costs + fuel_costs_2 + costs_visit + truck_costs, 2)))
        print(' Energy consumption: ')
        print('     Weight related [kWh]: ' + str(round((weight_energy * 10 **(-6) * 0.278),2)))
        print('     Speed related [kWh]: ' + str(round((speed_energy * 10 ** (-6) * 0.278), 2)))
        print('     Total [kWh]: ' + str(round(((weight_energy + speed_energy) * 10 ** (-6) * 0.278), 2)))
        print('     Energy consumption [kWh/km]: ' + str(round((((weight_energy + speed_energy) * 10 ** (-6) * 0.278) / distance),2)))

