import sys
import os
import matplotlib.pyplot as plt
import numpy as np

rtc_stats_f = sys.argv[1]
dyn_stats_f = sys.argv[2]
def_stats_f = sys.argv[3]
freq = int(sys.argv[4])
st = sys.argv[5]

for t in [4]:
	for metr in ['N1Latency T', 'N1Latency w', 'N2Latency T', 'N2Latency w', 'N3Latency T', 'N3Latency w', 'Tput', 'RxnTime T', 'RxnTime w', 'Rel. Metric ', 'Rel. Metric1', 'Abs Metric']:
		means = []
		meds = []
		tails = []
		# get rtc stats :
		with open(rtc_stats_f, 'r') as rf:
			for rl in rf.readlines():
				if metr in rl:
					ra = rl.split(' ')
					# 0 : Freq, 1 : t, -4 : tail, -3 : med,	-2 : mean
					if int(ra[1]) == t:
						means.append(float(ra[-2]))
						meds.append(float(ra[-3]))
						tails.append(float(ra[-4]))

		# get Dyn stats :
		with open(dyn_stats_f, 'r') as df:
			for dl in df.readlines():
				if metr in dl:
					da = dl.split(' ')
					if int(da[1]) == t:
						means.append(float(da[-2]))
						meds.append(float(da[-3]))
						tails.append(float(da[-4]))
		# get 30Hz Default stats :
		with open(def_stats_f, 'r') as df:
			for dl in df.readlines():
				if metr in dl:
					da = dl.split(' ')
					if int(da[1]) == t and int(da[0]) == freq:
						means.append(float(da[-2]))
						meds.append(float(da[-3]))
						tails.append(float(da[-4]))
		
		ngroups = 3
		fig, ax = plt.subplots()
		index = np.arange(ngroups)
		bar_width = 0.2
		opacity = 0.8
		rects1 = plt.bar(index, means, bar_width, alpha=opacity, color='b', label='Mean')
		rects2 = plt.bar(index+bar_width, meds, bar_width, alpha=opacity, color='g', label='Median')
		rects3 = plt.bar(index+2*bar_width, tails, bar_width, alpha=opacity, color='r', label='Tail')
		plt.xlabel('Algo')
		plt.ylabel('Metric')
		plt.title('RTC, DynAlgo, Default for %s %s'%(metr, st))
		plt.xticks(index+bar_width, ('RTC', 'DynAlgo', 'Default at ' + str(freq) + 'Hz'))
		plt.legend()
		plt.tight_layout()
		plt.show()
