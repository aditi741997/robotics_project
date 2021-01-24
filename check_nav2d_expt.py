import sys

ename = sys.argv[1]

runs = range( int(sys.argv[2]), int(sys.argv[3])+1 )

bad_runs = { -1}
efails = {-1}

# check NEG lat at shim
if "Default" not in ename:
	for r in runs:
		neg_ct = 0
		neg_ratio = 0.0
		with open("nav2d_shim_logs_"  + ename + "_run" + str(r) + ".out", 'r') as f:
			for l in f.readlines():
				if "NEGAT" in l:	
					neg_ratio = float( l.split(' ')[-1][:5] )
		with open("nav2d_shim_logs_"  + ename + "_run" + str(r) + ".err", 'r') as f:
			for l in f.readlines():
				if "-VE LAT" in l:
					neg_ct += 1
		if (neg_ratio > 0.001):
			print("Ename: %s, run: %i, has NEG LAT ratio : %f"%(ename, r, neg_ratio) )		
		if (neg_ct > 4):
			print("Ename: %s, run: %i, has NEG LAT count : %f"%(ename, r, neg_ct) )
			bad_runs.add(r)

# check mMaxVel = 0.3
for r in runs:
	with open("nav2d_robot_logs_OpeMap_" + ename + "_run" + str(r) + ".err", 'r') as f:
		for l in f.readlines():
			if "MaxVel" in l:
				try:
					if "Default" not in ename:
						maxVel = float( l.split(' ')[-3][:-1] )
					else:
						maxVel = float( l.split(' ')[-1][:5] )
					if (maxVel != 0.3):
						print("Ename: %s, run: %i, has WRONG maxVel: %f"% (ename, r, maxVel) )
						bad_runs.add(r)
				except:
					print("Ename: %s, run: %i, ERROR in getting maxVel: "% (ename, r) )

# check if smt exited before the end? Done.
for r in runs:
	fnames = ["nav2d_robot_logs_OpeMap_", "nav2d_robot_logs_"]
	for fname in fnames:
		with open(fname + ename + "_run" + str(r) + ".err", 'r') as f:
			for l in f.readlines():
				if "process has died" in l:
					pname = l.split(' ')[0]
					exitcode = int( l.split(' ')[8][:-1] )
					if exitcode != -15:
						print("Ename: %s, run: %i, process: %s died with exitcode: %i \n"%( ename, r, pname, exitcode ) )
						bad_runs.add(r)

# check #weird prints in opeMap, nav logs
for r in runs:
	fnames = ["nav2d_robot_logs_OpeMap_", "nav2d_robot_logs_"]
	with open(fname + ename + "_run" + str(r) + ".err", 'r') as f:
		weird_arr = []
		for l in f.readlines():
			if "WEIRD" in l:
				weird_arr.append(l)
		if len(weird_arr) > 2:
			print("Ename: %s, run: %i, WEIRD prints: %s"%(ename, r, str(weird_arr)) )

# check nan transform
for r in runs:
	with open("nav2d_robot_logs_" + ename + "_run" + str(r) + ".err", 'r') as f:
		nane = False
		for l in f.readlines():
			if "nan" in l:
				nane = True
		if (nane):
			print("Ename: %s, run: %i, HAS nan transforms!"% (ename, r) )	
			bad_runs.add(r)

# get #tf errors.
for r in runs:
        with open("nav2d_robot_logs_" + ename + "_run" + str(r) + ".err", 'r') as f:
		tf_err = False
		map_fail_ct = 0
		efail = False
		for l in f.readlines():
			if "Could not get robot position" in l:
				tf_err = True
			if "ation fail" in l:
				efail = True	
			if "MAPPING F" in l:
				map_fail_ct += 1
		if (map_fail_ct > 1) or (tf_err) or (efail):
			bad_runs.add(r)
			print("Ename: %s, run: %i, HAS TF error %i EFail: %i / Mapping failed ct: %i"%(ename, r, tf_err, efail, map_fail_ct) )
		if (efail):
			efails.add(r)

print("FINAL Set of bad runs: ", bad_runs, "#BadRuns: ", len(bad_runs) )
print("#E Fails: ", len(efails), efails)
