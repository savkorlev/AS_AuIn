import pandas as pd
import numpy as np
from gurobipy import *
from gurobipy import Model
np.random.seed(0)

num_sup = 7     # number of suppliers (+1 for manufacturer)
num_freq = 3      # number of visiting frequencies
num_lanes = 2       # number of P-Lanes
num_level = 2       # number of speed levels
num_routes = 3      # number of possible routes

# Sets of the model
V = [*range(num_sup)]
Fr = [*range(1,num_freq+1)]
R = [*range(num_routes)]
L = [*range(num_level)]
P = [*range(num_lanes)]

# Parameters of the model
phi = 6                 # transportation costs per unit distance
theta_large = 100       # earliness penalty costs
psi = 100               # tardiness penalty costs
omega = 5               # fixed costs per vehicle per visit
Q = 100                 # vehicle capacity in number of boxes
c_e = 5                 # electricity costs [€/J]
c_c = 7                 # fuel costs [€/J]
e = 3                   # cost of emitted GHG
c_jk = pd.DataFrame(np.random.randint(0,30,size=(len(V), len(V))))      # non-negative costs between suppliers / distance
b_jp = pd.DataFrame(np.random.randint(0,100,size=(len(V), len(P))))      # demand in number of boxes of supplier j on lane p
b_j = b_jp.sum(axis=1)                                                   # total demand from one supplier
n_js = pd.DataFrame([b_j/Fr[i] for i in range(len(Fr))])                   # average number of boxes collected per s visit from supplier j
d_p = pd.DataFrame(np.random.randint(12,20,size=(len(P), 1)))            # due date of lane P
U_j = pd.DataFrame(np.random.uniform(0.01,0.05,size=(len(V), 1)))        # unloading time per box of supplier j
M = 100000                                                               # Big-M


# Setting up the model
#m: Model = Model()
m = Model()


# Decision variables
x = m.addVars(R, vtype = GRB.BINARY)
y = m.addMVar((len(V),len(R)), vtype = GRB.BINARY)
z = m.addMVar((len(V),len(V),len(R)), vtype = GRB.BINARY)
u = m.addMVar((len(R),len(Fr)), vtype = GRB.BINARY)
D = m.addMVar((len(R),len(Fr)), vtype = GRB.CONTINUOUS)
A = m.addMVar((len(R),len(Fr)), vtype = GRB.CONTINUOUS)
sigma = m.addMVar((len(V), len(R),len(Fr)), vtype = GRB.BINARY)
delta = m.addMVar((len(V), len(V),len(R),len(Fr)), vtype = GRB.BINARY)
epsilon = m.addMVar((len(V),len(R), len(Fr),len(Fr), len(P)),vtype = GRB.BINARY)
F = m.addMVar((len(V),len(P)), vtype = GRB.CONTINUOUS)
E = m.addMVar((len(V),len(P)), vtype = GRB.CONTINUOUS)
T = m.addMVar((len(V),len(P)), vtype = GRB.CONTINUOUS)
C = m.addMVar((len(R),len(Fr)), vtype = GRB.CONTINUOUS)


# Objective function
m.setObjective(sum(delta[j][k][r][s-1]*c_jk[j][k]*s*phi for j in range(len(V)) for k in range(len(V)) for r in range(len(R)) for s in Fr) + sum(u[r][s-1]*s*omega for r in range(len(R)) for s in Fr) + sum(E[j][p]*theta_large for j in range(len(V)) for p in range(len(P))) + sum(T[j][p]*psi for j in range(len(V)) for p in range(len(P))), GRB.MINIMIZE)


# Constraints
m.addConstrs(x[r+1] <= x[r] for r in range(len(R)-1))
m.addConstrs(sum(y[j][r] for r in range(len(R))) == 1 for j in range(1,len(V)))
m.addConstrs(sum(y[j][r] for j in range(1,len(V))) >= x[r] for r in range(len(R)))
m.addConstrs(sum(z[j][k][r] for k in range(len(V))) == y[j][r] for j in range(1,len(V)) for r in range(len(R)))
m.addConstrs(sum(z[j][k][r] for j in range(len(V))) == y[k][r] for k in range(1,len(V)) for r in range(len(R)))

# Constraint 1g) is missing
m.addConstrs(z[j][j][r] == 0 for j in range(len(V)) for r in range(len(R)))           # possible workaround for constraint 1g)
m.addConstrs(sum(z[0][j][r] for j in range(1,len(V))) == x[r] for r in range(len(R)))   # possible workaround for constraint 1g)
m.addConstrs(sum(z[j][0][r] for j in range(1,len(V))) == x[r] for r in range(len(R)))   # possible workaround for constraint 1g)

m.addConstrs(sum(u[r][s-1] for s in Fr) == x[r] for r in range(len(R)))
m.addConstrs(sum(y[j][r]*b_j[j] for j in range(1,len(V))) <= sum(u[r][s-1]*Q*s for s in Fr) for r in range(len(R)))
m.addConstrs(sum(y[j][r]*b_j[j] for j in range(1,len(V))) >= sum(((s-2)*Q+1)*u[r][s-1] for s in Fr) for r in range(len(R)))
m.addConstrs(2*delta[j][k][r][s-1] <= z[j][k][r] + u[r][s-1] for j in range(len(V)) for k in range(len(V)) for r in range(len(R)) for s in Fr)
m.addConstrs(1+delta[j][k][r][s-1] >= z[j][k][r] + u[r][s-1] for j in range(len(V)) for k in range(len(V)) for r in range(len(R)) for s in Fr)
m.addConstrs(2*sigma[j][r][s-1] <= y[j][r] + u[r][s-1] for j in range(1,len(V)) for r in range(len(R)) for s in Fr)
m.addConstrs(1+sigma[j][r][s-1] >= y[j][r] + u[r][s-1] for j in range(1,len(V)) for r in range(len(R)) for s in Fr)
m.addConstrs(sum(epsilon[j][r][s-1][t][p]*t*n_js[j][s-1] for t in range(s)) >= sum(sigma[j][r][s-1]*b_jp[j][k] for k in range(1,p)) for j in range(1,len(V)) for r in range(len(R)) for s in Fr for p in range(len(P)))
m.addConstrs(sum(((t-1)*n_js[j][s-1]+1)*epsilon[j][r][s-1][t][p] for t in range(s)) <= sum(sigma[j][r][s-1]*b_jp[j][k] for k in range(1,p)) for j in range(1,len(V)) for r in range(len(R)) for s in Fr for p in range(len(P)))
m.addConstrs(sum(epsilon[j][r][s-1][t][p] for t in range(s)) == sigma[j][r][s-1] for j in range(1,len(V)) for r in range(len(R)) for s in Fr for p in range(len(P)))
m.addConstrs(A[r][s-1] >= D[r][s-1] + sum(z[j][k][r]*c_jk[j][k] for j in range(len(V)) for k in range(len(V))) + sum(sigma[j][r][t-1]*n_js[j][t-1]*U_j[0][j] for j in range(1,len(V)) for t in Fr) - M*sum(u[r][t] for t in range(s-1)) for r in range(len(R)) for s in Fr)
m.addConstrs(F[j][p] >= C[r][t] + M*(epsilon[j][r][t][s-1][p] - 1) for j in range(1,len(V)) for r in range(len(R)) for s in Fr for p in range(len(P)) for t in range(len(Fr)))
m.addConstrs(F[j][p] <= C[r][t] + M*(1 - epsilon[j][r][t][s-1][p]) for j in range(1,len(V)) for r in range(len(R)) for s in Fr for p in range(len(P)) for t in range(len(Fr)))
m.addConstrs(E[j][p] >= d_p[0][p] -F[j][p] for j in range(1,len(V)) for p in range(len(P)))
m.addConstrs(T[j][p] >= F[j][p] - d_p[0][p] for j in range(1,len(V)) for p in range(len(P)))

# Solve model
m.params.OutputFlag = 0
m.optimize()

# print result
print('\nResults of x:')
for i in range(len(R)):
    print(x[i].X)

print('\nResults of y:')
for i in range(len(V)):
    for j in range(len(R)):
        if y[i][j].X == 1:
            print('y['+str(i)+']['+str(j)+']:'+str(y[i][j].X))


print('\nResults of z:')
for i in range(len(V)):
    for j in range(len(V)):
        for t in range(len(R)):
            if z[i][j][t].X == 1:
                print('z[' + str(i) + '][' + str(j) + '][' + str(t) + ']:' + str(z[i][j][t].X))

print('\nResults of u:')
for i in range(len(R)):
    for j in range(len(Fr)):
        if u[i][j].X == 1:
            print('u['+str(i)+']['+str(j)+']:'+str(u[i][j].X))

print('\nResults of F:')
for i in range(1,len(V)):
    for j in range(len(P)):
        print('F['+str(i)+']['+str(j)+']:'+str(F[i][j].X))

print('\nResults of D:')
for i in range(len(R)):
    for j in range(len(Fr)):
        print('D['+str(i)+']['+str(j)+']:'+str(D[i][j].X))

print('\nResults of A:')
for i in range(len(R)):
    for j in range(len(Fr)):
        print('A['+str(i)+']['+str(j)+']:'+str(A[i][j].X))


