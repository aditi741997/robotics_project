import sys

ccf = float(sys.argv[1])
mcf = float(sys.argv[2])
muf = float(sys.argv[3])
ncf = float(sys.argv[4])
npf = float(sys.argv[5])

print("Got inputs: ccf %f, mcf %f, muf %f, ncf %f, npf %f"%(ccf, mcf, muf, ncf, npf))

# CC will be enforced by giving the right input to the shim_freq_node.
# Here, we set freqs for all the other NC subchains in nav2d.

pre_fname = "src/navigation_2d/nav2d_tutorials/param/"

mapper_fname = pre_fname + "mapper.yaml"
nav_fname = pre_fname + "navigator.yaml"

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
