from gpkit import Variable, VectorVariable, Model
from gpkit.nomials import Monomial, Posynomial, PosynomialInequality

num_nodes = 7

# If we use wcet for 1core with prio_nodes_tput:
# 0.09 or 0.2 or 0.8, 1, 1.2,1.5: soln unknown
# 0.8, 1.5, 1.5, 1.5: also infeasible
# 0.8, 1.5, 1.5, 2.0: also not working
# 0.8, 2.0, 2.0, 2.5: also not working
# 0.8, 2.0, 2.5, 3.0: also not working
# 0.8, 2.0, 3.0, 4.0: also not working
# 0.8, 2.0, 3.0, 4.5: also not working
path_ddl = [0.4, 2.0, 3.2, 4.5] # this works for 1core,prio_nodes_tput,wcet.
#0.8,2.0,3.2,4.5 works.

# If we use ci_75p for 1core with prio_nodes_tput:
# 0.09 or 0.2, 1, 1.2,1.5 didnt work
# 0.2, 1, 1.6, 2.3
#path_ddl = [0.2, 1, 1.6,2.3] #Works for 1core,prio_nodes_tput,ci_75p 

print("pATH LATENCY CONSTRAINTS: ", path_ddl)

# order: s,lc,lp,mc,mu,np,nc
prio_nodes_tput = [4, 6, 7, 1, 2, 3, 5]
#prio_nodes_int = [5,6,7,4,1,2,3]

wcet = [0.001, 0.0026, 0.0023, 0.3416, 0.1684, 0.2293, 0.0066]
ci_75p = [0.001, 0.0016, 0.0014, 0.0264, 0.1265, 0.181, 0.004]

print("Using the following ci: ", wcet)

util = 0.95

# 1core:
cores = [0 for i in range(num_nodes)]
num_cores = 1

# 3core:

a_si = [Variable("s"+ str(i) ) for i in range(num_nodes)]
a_ti = [Variable("t"+ str(i) ) for i in range(num_nodes)]

a_li = [Variable("l" + str(i) ) for i in range(4) ]

a_zij = [ [Variable("z" + str(i) + str(j), integer=True ) for j in range(num_nodes)  ] for i in range(num_nodes) ]

# Eqn 3 : Objective is sum of li.
objective = sum(a_li)
constr = []

# Const 4
for i in range(4):
	constr.append( a_li[i] <= path_ddl[i] )

print("ADDED Constr4: ", constr)

int_vars = []

# Const 5
for i in range(num_nodes):
	c5 = [ wcet[i]/a_si[i] ]
	for j in range(num_nodes):
		if ( (prio_nodes_tput[j] > prio_nodes_tput[i]) and (cores[i] == cores[j]) ):
			c5.append( a_zij[i][j]*wcet[j]/a_si[i] )
			# Const 10
			constr.append( a_zij[i][j] >= 0.99 )
			constr.append( a_si[i]/( a_ti[j] * a_zij[i][j] ) <= 1.0 )
			int_vars.append(str(i)+"_"+str(j))
	constr.append(sum(c5) <= 1.0)
print("added Constr5,10: ", constr)

# Const 7
for i in range(num_nodes):
	constr.append( a_si[i]/a_ti[i] <= 1.0 )
print("ADDED constr7: ", constr)

# Const 8
for nc in range(0,num_cores):
	c8_nc = []
	for i in range(num_nodes):
		if cores[i] == nc:
			c8_nc.append( wcet[i]/a_ti[i] )
	constr.append( sum(c8_nc) <= util )
print("ADDED constr8: ", constr)

# Const 9
# Scan <= 50Hz:
constr.append( a_ti[0] >= 1/50.0 )
#constr.append( a_ti[3] >= 1/5.0 )
constr.append( a_ti[4] >= a_ti[3] )
print(constr)

# Const 14 : lp = sum tk + sk, for k in path p.
chains = [ [0,1,2], [0,3,6,2], [0,3,5,6,2], [0,3,4,5,6,2] ]

for ch in range(4):
	c14_ch = []
	for i in chains[ch]:
		c14_ch.append( a_ti[i]/a_li[ch] )
		c14_ch.append( a_si[i]/a_li[ch] )
	constr.append( sum(c14_ch) <= 1.0 )
print(constr)

m = Model(objective, constr)

m.debug()
sol = m.solve(verbosity=1)

print("Main integer variables: ", int_vars)
print(sol.table())
