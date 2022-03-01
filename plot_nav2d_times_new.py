# robot_nav2d_navigator_plan_stats_DynNOFF1_1c_run10.txt start: 73366.939882 , end: 73557.958750 

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import math
import sys, json

params = {'figure.figsize'  : [13, 9],
		  'pdf.fonttype' : 42,
		  'ps.fonttype' : 42,
		  # 'font.family': 'MankSans-Medium',
		  'grid.color': '#aaaaaa',
		  'grid.linestyle': '--',
		  'grid.linewidth': 1.5,
		  'figure.autolayout': True,
		  } 

dcolors = ['#006400','#990000', '#0066CC', '#FFA700', '#993399' , '#753b05', '#333333', '#00FFFF']
col_names = ["green", "red", "blue", "yellow", "pink", "brown", "darkgray", "aqua"]
fontsz = 37
numbins = 20
plt.rcParams.update(params)


def plot_ci_ts(fpath, fname, node_code, start_t, end_t):
	times = [] # y-axis
	ts = [] # x-axis
	with open(fname) as f:
		for fl in f.readlines():
			if "times:" in fl:
				times +=  [ round(float(x),4) for x in fl.split(" ")[2:-1] ]
			elif "ts:" in fl:
				ts +=  [ round(float(x),4) for x in fl.split(" ")[2:-1] ]
	# find first index > start_t
	ifirst = next(x[0] for x in enumerate(ts) if x[1] > start_t)
	# find last id < end_t
	ilast = next(x[0] for x in enumerate(ts) if x[1] > end_t) - 1
	# take subarr of times, ts
	ts = ts[ifirst:ilast]
	times = times[ifirst:ilast]
	ts = list(map(lambda x: x - start_t, ts))
	plt.plot(ts, times, 'o-', color="#0066CC")
	plt.title("Navigation: Compute time for Node " + node_code, fontsize=fontsz)
	plt.locator_params(axis='y', nbins=10)
	plt.locator_params(axis='x', nbins=8)
	plt.xlabel("Time since exploration start (s)", fontsize=fontsz)
	plt.ylabel("Computation time (s)", fontsize=fontsz)
	plt.xticks(fontsize=fontsz)
	plt.yticks(fontsize=fontsz)
	plt.grid()
	plt.savefig("%s/Plot_%s.pdf"%(fpath, node_code))
	plt.clf()


fpath = sys.argv[1]
fprefix = sys.argv[2]
fsuffix = sys.argv[3]

start_t = float(sys.argv[4])
end_t = float(sys.argv[5])

node_name_map = {"mapper_scanCB": "GL", "navigator_plan": "GP"}

for node in ["mapper_scanCB", "navigator_plan"]:
	plot_ci_ts( fpath, "%s/%s%s%s" % (fpath, fprefix, node, fsuffix), node_name_map[node], start_t, end_t )