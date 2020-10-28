import sys
import matplotlib.pyplot as plt

rt_fname = sys.argv[1]

def plot_smt(x,y,sty,n,yl):
	plt.plot(x,y,sty, label=n)
	plt.title('Nav2D %s'%(n) )
	if yl > 0.0:
		plt.ylim(0.0, yl)
	plt.legend()
	plt.show()

yls = {"Scan_MapCB_MapU_NavP_NavC_LP" : 3.0, "Scan_LC_LP" : 0.0, "Scan_MapCB_NavCmd_LP" : 3.0, "Scan_MapCB_NavPlan_NavCmd_LP" : 3.0}

for chain in ["Scan_MapCB_MapU_NavP_NavC_LP", "Scan_LC_LP", "Scan_MapCB_NavCmd_LP", "Scan_MapCB_NavPlan_NavCmd_LP"]:
	print "Starting chain", chain
	rts = []
	lats = []
	tputs = []
	ts = []
	with open(rt_fname, 'r') as f:
		fl = f.readlines()
		for l in fl:
			if chain in l:
				print "Chain in Line", l
				if "Latency" in l:
					lats += [ float(x) for x in l.split(' ')[1:-1] ]
				elif "Tput" in l:
					tputs += [ float(x) for x in l.split(' ')[1:-1] ]
				elif "RT_" in l:
					rts += [ float(x) for x in l.split(' ')[1:-1] ]
				elif "TS_" in l:
					ts += [ float(x) for x in l.split(' ')[1:-1] ]
			else:
				print "Chain not in Line", l
	# len(TS) should be = len(lat).
	# len(TS) should be len(tput)+1.
	if len(ts) > 1:
		plot_smt(ts, lats, 'bo:', chain + " Latency", yls[chain])
		plot_smt(ts[1:], tputs, 'g*:', chain + " Tput", yls[chain])
		plot_smt(ts[1:], rts, 'r^:', chain + " RT", yls[chain])
