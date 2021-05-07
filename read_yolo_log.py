import sys

per_run_ts = []
per_run_tput = []
per_run_lat = []
per_run_rt = []

start_ind = int(sys.argv[1])
end_ind = int(sys.argv[2])

expt_id = "StaticNOFF1Y_1c"

runs = range(start_ind, end_ind+1)
for br in []:
	if br in runs:
		runs.remove(br)

for r in runs:
	print("doing RUN ", r, " for expt ", expt_id)
	ts_arr = []
	lat_arr = []
	tput_arr = []
	rt_arr = []
	with open("yolo_logs_" + expt_id + "_run" + str(r) + ".out", 'r') as yf:
		for l in yf.readlines():
			if " using cam ts:" in l and "" in l:
				add_ind = 4 if "WRITING ALL ARRS TO FILE" in l else 0
				la = l.split(' ')
				ts = float( la[10+add_ind][:-1] )
				lat = ts - float( la[7+add_ind][:-1] )
				tput = 0.0
				rt = 0.0
				if len(ts_arr) > 0:
					tput = ts - ts_arr[-1]
					rt = tput + lat_arr[-1]
					tput2 = ts - float( la[12+add_ind][:-4] )
					rt2 = ts - float( la[3+add_ind][:-1] )
					if abs(tput2-tput) > 0.01 or abs(rt2 - rt) > 0.01:
						print("WEIRD/ERROR in ", l, " tputs:", tput, tput2, " rts: ", rt, rt2)
					tput_arr.append(tput)
					rt_arr.append(rt)	
				ts_arr.append(ts)
				lat_arr.append(lat)
				print("GOT numbers: ", ts, lat, tput, rt)
	per_run_ts.append(ts_arr)
	per_run_lat.append(lat_arr)
	per_run_tput.append(tput_arr)
	per_run_rt.append(rt_arr)

print("PER RUN TS: ", per_run_ts)
print('-- ')
print("PER RUN LAT: ", per_run_lat)
print('-- ')
print("PER RUN TPUT: ", per_run_tput)
print('-- ')
print("PER RUN RT: ", per_run_rt)
print('-- ')

