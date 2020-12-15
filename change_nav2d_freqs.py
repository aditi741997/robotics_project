import sys

ccf = float(sys.argv[1])
mcf = float(sys.argv[2])
muf = float(sys.argv[3])
ncf = float(sys.argv[4])
npf = float(sys.argv[5])

lcf = float(sys.argv[6])
lpf = float(sys.argv[7])

print("Got inputs: ccf %f [lcf %f, lpf %f], mcf %f, muf %f, ncf %f, npf %f"%(ccf, lcf, lpf, mcf, muf, ncf, npf))

# CC will be enforced by giving the right input to the shim_freq_node.
# Here, we set freqs for all the other NC subchains in nav2d.

pre_fname = "src/navigation_2d/nav2d_tutorials/param/"

mapper_fname = pre_fname + "mapper.yaml"
nav_fname = pre_fname + "navigator.yaml"
ope_fname = pre_fname + "operator.yaml"
cmp_fname = pre_fname + "costmap.yaml"

with open(mapper_fname, 'r') as mf:
	mapper_data = mf.readlines()

with open(nav_fname, 'r') as nf:
	nav_data = nf.readlines()

print "For mapper: Old muF: ", mapper_data[4], ", Old mcF : ", mapper_data[26]
mapper_data[4] = "map_update_rate: " + str(1.0/muf) + " \n" # note that this is the default param name, but this is actually period.
mapper_data[26] = "map_scan_period: " + str(1.0/mcf) + " \n"

with open(mapper_fname, 'w') as mf:
	mf.writelines(mapper_data)

print "For navigator: Old navcF: ", nav_data[1], ", Old navpF: ", nav_data[22]
nav_data[1] = "frequency: " + str(ncf) + " \n"
nav_data[22] = "replanning_period: " + str(1.0/npf) + " \n"

with open(nav_fname, 'w') as nf:
	nf.writelines(nav_data)

# Local Planner freq:
with open(ope_fname, 'r') as f:
	ope_data = f.readlines()

print("For operator, current freq: %s, new freq (LP): %f"%( ope_data[1], lpf) )
ope_data[1] = "frequency: " + str(lpf) + "\n"

with open(ope_fname, 'w') as f:
	f.writelines(ope_data)

# Local Costmap Freq:
with open(cmp_fname, 'r') as f:
	cmp_data = f.readlines()

print("For local cmp, current freq: %s, new freq (LC): %f"%( cmp_data[2], lcf ) )
cmp_data[2] = "update_frequency: " + str(lcf) + "\n"

with open(cmp_fname, 'w') as f:
	f.writelines(cmp_data)
