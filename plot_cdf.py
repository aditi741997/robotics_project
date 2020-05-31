import re
import numpy as np
import matplotlib.pyplot as plt
import sys

folder = sys.argv[1]
pre1 = sys.argv[2]
pre2 = sys.argv[3]
t = int(sys.argv[4])
f=15

sty = ['g.:', 'b*-', 'r^--']
#need to merge all 5 runs
for metr in ['RxnTime', 'RelMetric1', 'RelMetric']:
	i = 0
	for alg in ['RTC', 'Dyn', 'Default']:
		arr = []
		if alg =='Default':
			f=30
		else:
			f=15	
		for r in [1,2,3,4,5]:
			#check validity
			hr = 0.0
			vf = '%s/%s/%s_%s_%s_vision_node_%i.%i.%i.out'%(folder, alg,pre1, alg, pre2, r, f, t)
			if i == 1:
				vf = '%s_%s_%s_New_vision_node_%i.%i.%i.out'%(pre1, alg, pre2, r,f,t)
			print "Reading", vf
			with open(vf, 'r') as fil:
                    		for l in fil.readlines():
                        		if "Hit rate :" in l:
                                		hr = float(l.split(' ')[7][:-1])
			print alg, r, t, hr
			if hr > 0.5:
				#run was valid. Now open log file.
				print "Reading for ", alg, r, t
				lf = '%s/%s/%s_%s_Logs_%s_%i.%i.%i.txt'%(folder, alg,pre1, alg, pre2, r,f,t)
				if i == 1:
					lf = '%s_%s_Logs_%s_New_%i.%i.%i.txt'%(pre1, alg, pre2, r,f,t)
				print lf
				with open(lf, 'r') as fil:
					for l in fil.readlines():
						if metr in l:
							arr += [float(x) for x in l.split(', ')[1:-1]]
		s_arr = np.sort(arr)
		pr = 1. * np.arange(len(arr))/(len(arr) - 1)
		print len(pr), len(s_arr)
		new_l = ((9*len(s_arr))/10)
		s_arr = s_arr[new_l:]
		pr = pr[new_l:]
		print len(pr), len(s_arr)
		plt.plot(s_arr, pr, sty[i], label=alg)
		i += 1
	plt.title(metr + ' for RTC, Dyn at O%i MCS3.2'%(t))
	plt.xlabel(metr)
	plt.ylabel('CDF')
	plt.legend()
	plt.show()
	
