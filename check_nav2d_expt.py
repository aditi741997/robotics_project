import sys

ename = sys.argv[1]

runs = range( int(sys.argv[2]), int(sys.argv[3])+1 )

bad_runs = { -1}
efails = {-1}

smallest_map = (sys.argv[4] == "smallest")

run_txt = "_r" if smallest_map else "_run"
run_txt = "_run"
# check NEG lat at shim
if "Default" not in ename:
	for r in runs:
		neg_ct = 0
		neg_ratio = 0.0
		with open("nav2d_shim_logs_"  + ename + run_txt + str(r) + ".out", 'r') as f:
			for l in f.readlines():
				if "NEGAT" in l:	
					neg_ratio = float( l.split(' ')[-1][:5] )
		with open("nav2d_shim_logs_"  + ename + run_txt + str(r) + ".err", 'r') as f:
			for l in f.readlines():
				if "-VE LAT" in l:
					neg_ct += 1
		if (neg_ratio > 0.001):
			print("Ename: %s, run: %i, has NEG LAT ratio : %f"%(ename, r, neg_ratio) )		
		if (neg_ct > 4):
			print("Ename: %s, run: %i, has NEG LAT count : %f"%(ename, r, neg_ct) )
			bad_runs.add(r)

# check scan lat if its negative.
for r in runs:
        with open("../robot_nav2d_mapper_scanCB_stats_" + ename + "_run" + str(r) + ".txt", 'r') as f:
                scan_lat = []
                for fl in f.readlines():
                        if "lat:" in fl:
                                scan_lat += [ float(x) for x in fl.split(" ")[2:-1] ]
                neg_scan_lat = filter(lambda x: x < 0.0, scan_lat)
                if len(neg_scan_lat) > 0:
                        print("ENAME %s, run %i has NEG LAT AT MAPCB!!"%(ename, r) )
                        bad_runs.add(r)

# check mMaxVel = 0.3
for r in runs:
	with open("nav2d_robot_logs_OpeMap_" + ename + run_txt + str(r) + ".err", 'r') as f:
		for l in f.readlines():
			if "MaxVel" in l:
				try:
					if "Default" not in ename:
						maxVel = float( l.split(' ')[-3][:-1] )
					else:
						maxVel = float( l.split(' ')[-1][:5] )
					if (maxVel != 0.25):
						print("Ename: %s, run: %i, has WRONG maxVel: %f"% (ename, r, maxVel) )
						bad_runs.add(r)
				except:
					print("Ename: %s, run: %i, ERROR in getting maxVel: "% (ename, r) )

# check if smt exited before the end? Done.
for r in runs:
	fnames = ["nav2d_robot_logs_OpeMap_", "nav2d_robot_logs_"]
	for fname in fnames:
		with open(fname + ename + run_txt + str(r) + ".err", 'r') as f:
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
	with open(fname + ename + run_txt + str(r) + ".err", 'r') as f:
		weird_arr = []
		for l in f.readlines():
			if "WEIRD" in l:
				weird_arr.append(l)
		if len(weird_arr) > 2:
			print("Ename: %s, run: %i, WEIRD prints: %s"%(ename, r, str(weird_arr)) )

def are_goals_equal(g1, g2):
        return ( abs(g1[1] - g2[1]) + abs(g1[0] - g2[0]) ) < 4

# check nan transform & THRASHing in goals at navPlan
start_rts = {}
thrash_set = {}
for r in runs:
        start_rt = 0.0
        start_mt = 0.0
        start_st = 0.0
        end_rt = 0.0
        end_st = 0.0
        thrashing = 0
        goali = (0,0)
        goali1 = (0,0)
        count_goali1 = 0
        with open("nav2d_robot_logs_" + ename + run_txt + str(r) + ".err", 'r') as f:
		nane = False
		for l in f.readlines():
			if "nan" in l:
				nane = True
		        if "StartExpl" in l:
                                start_rt = float( l.split(' ')[1][1:-1] )
                                start_st = float( l.split(' ')[2][:-2] )
                                start_mt = float( l.split(' ')[11][:-1] )
                        if ("Exploration failed" in l) or ("Exploration has fail" in l) or ("Exploration has fin" in l):
                                end_rt = float( l.split(' ')[1][1:-1] )
                                end_st = float( l.split(' ')[2][:-2] )
                        if ("| STartPOint :" in l):
                                goal = (int(l.split(' ')[-8]), int(l.split(' ')[-7]))
                                if (are_goals_equal(goal, goali1)):
                                        #if count_goali1%80 == 5:
                                                #print "RUN", r, "same goal", goal, goali1, "ct: ", count_goali1
                                        count_goali1 += 1
                                elif ( not are_goals_equal(goal, goali1) and are_goals_equal(goal, goali) ):
                                    print("################### Ename: %s, run: %i, HAS thrash!!"%(ename, r) , goal, goali1, goali, count_goali1)
                                    goali = goali1 # prev goal
                                    goali1 = goal # current goal
                                    thrashing += 1 
                                    count_goali1 = 0
                                else:
                                    #print("Ename: %s, run: %i, HAS NEW goal!!"%(ename, r) , goal, goali1, goali, count_goali1)
                                    goali = goali1 # prev goal
                                    goali1 = goal # current goal
                                    count_goali1 = 1
                if (nane):
			print("Ename: %s, run: %i, HAS nan transforms!"% (ename, r) )	
			bad_runs.add(r)
                if (thrashing > 1):
                    thrash_set[r] = thrashing
                    print("Ename: %s, run: %i, HAS THRASHing ct: %i !"% (ename, r, thrashing) )
        start_rts[r] = (start_mt)
        ratio=(end_st-start_st)/(end_rt-start_rt)
        if (end_rt == 0.0):
                print("Ename %s, run %i, did NOT finish!!!"% (ename, r) )
        #if (ratio < 4.5):i
        else:
                print("ST:RT ratio for ename %s, run %i is %f"%(ename, r, (end_st-start_st)/(end_rt-start_rt) ) )
        if (ratio < 4.1):
                bad_runs.add(r)

# get #tf errors.
for r in runs:
        with open("nav2d_robot_logs_" + ename + run_txt + str(r) + ".err", 'r') as f:
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

# check for WEIRD prints in PPMap logs.
for r in runs:
    with open("nav2d_PPMap_"  + ename + run_txt + str(r) + ".out", 'r') as f:
        for l in f.readlines():
            if "WEIRD" in l:
                print("Ename: %s, run: %i, HAS WEIRD PP error: %s"%(ename, r, l) )

# check for 'ERROR' tid=0 errors | yolo asan error:
good_runs_bad_yolo_stats = {}
for r in runs:
	yolo_asan_err = False
	try:
		with open("yolo_logs_"+  ename + run_txt + str(r) + ".err", 'r') as f:
			for l in f.readlines():
				if "SIGSEGV" in l:
					yolo_asan_err = yolo_asan_err or True
					print( ename + run_txt + str(r), "HAS ASAN error line: ", l)
		if yolo_asan_err:
			bad_runs.add(r)
	        bad_yolo_ct = 0
                with open("yolo_logs_"+  ename + run_txt + str(r) + ".out", 'r') as f:
                    bad_yolos = filter( lambda x: "BAD YOLO" in x, f.readlines())
                    bad_yolo_ct = len(bad_yolos)
                    bad_yolo_after_init = filter(lambda x: float(x.split(' ')[3]) > (start_rts[r]+10.0) , bad_yolos)
                    
                # if >3 reject, else if >1 in >10s : reject.
                if (bad_yolo_ct > 3) or (len(bad_yolo_after_init) > 1):
                    print("Ename: %s, run: %i, HAS too many bad yolos %i after init: %i [start: %f]"%(ename, r, bad_yolo_ct, len(bad_yolo_after_init), start_rts[r]))
                    bad_runs.add(r)
                elif r not in bad_runs:
                    good_runs_bad_yolo_stats[r] = (bad_yolo_ct, len(bad_yolo_after_init))
        except:
		print("Weird error in reading yolo logs.", sys.exc_info()[0])
        
print("FOR Good runs, bad yolo stats: ", good_runs_bad_yolo_stats)
print("FINAL Set of bad runs: ", bad_runs, "#BadRuns: ", len(bad_runs) )
print("#E Fails: ", len(efails), efails)
print("THRASHing: ", thrash_set)
