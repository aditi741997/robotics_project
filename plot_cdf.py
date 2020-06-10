import re
import numpy as np
import matplotlib.pyplot as plt
import sys
import math

folder = sys.argv[1]
pre1 = sys.argv[2]
pre2 = sys.argv[3]
t = sys.argv[4]
f=15

def read_abs_deg_metric(fname):
	c=3
	a = []
	with open(fname, 'r') as fm:
        	fml = fm.readlines()
	        for i in range((len(fml)/c)):
        		hpp = c*i
       			try:
                		#human y coordinate
                		# hy = float(fml[hpp+2].split(' ')[-1][:-1])
				hy = float(fml[hpp].split(' ')[-2])
				tc = float(fml[hpp+2].split(' ')[-2])
				ts = float(fml[hpp+1].split(' ')[-2])					
				ty = ts*float((3.0*2*tc)/((2*tc*tc) - 1.0))
				h_theta = math.atan(hy/3.0)
				h_theta = math.atan2(hy,3.0)
                                t_theta1 = math.atan(ty/3.0)
                                t_theta1 = math.atan2(2*ts*tc, 2*tc*tc-1.0)
                                if (max(h_theta, t_theta1) -  min(h_theta, t_theta1)) > math.pi:
                                        a.append(abs( (max(h_theta, t_theta1) -  min(h_theta, t_theta1)) -2*math.pi))
                                else:
                                        a.append(abs(t_theta1-h_theta))
			except:
				print "error", i, hy, tc, ts, ty
	b = a[len(a)/50:]
	print "Length : ", len(a), "Ignoring " , len(a)/50, "Len b : ", len(b)
	return b

def read_new_abs_deg_metric(fname):
        c=4
        a=[]
        with open(fname, 'r') as fm:
                fml = fm.readlines()
                for i in range(len(fml)/c):
                        hpp = c*i
                        hy = float(fml[hpp].split(' ')[-2])
                        hx = float(fml[hpp+1].split(' ')[-2])
                        ts = float(fml[hpp+2].split(' ')[-2])
                        tc = float(fml[hpp+3].split(' ')[-2])
                        ha = math.atan2(hy,hx)
                        tss = 2*ts*tc
                        tcc = 2*tc*tc-1.0
                        ta = math.atan2(tss, tcc)
			if (max(ta,ha) - min(ta,ha)) > math.pi:
				a.append(abs(max(ta,ha) - min(ta,ha) - 2*math.pi))	
                        else:
				a.append(abs(ta-ha))
                        if a[-1] > math.pi*0.98:
                                print hpp, hy, hx, ha, ts, tc, ta
        b=a[len(a)/10:]
        print "Len a " , len(a), "Ignoring ", len(a)/10, "Len b : ", len(b)
        return b

sty = ['g.:', 'b*-', 'r^--']
#need to merge all 5 runs
for metr in ['RxnTime', 'RelMetric1', 'Abs Deg']: #'RelMetric', 
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
			vf = '%s/%s/%s_%s_vision_node_%i.%i.%s.out'%(folder, alg,pre1, alg,  r, f, t)
			#'''
			if i == 1:
				vf = '%s_%s_vision_node_%i.%i.%s.out'%(pre1, alg, r,f,t)
			#'''
			print "Reading", vf
			with open(vf, 'r') as fil:
                    		for l in fil.readlines():
                        		if "Hit rate :" in l:
                                		hr = float(l.split(' ')[7][:-1])
			print alg, r, t, hr
			if hr > 0.0:
				#run was valid. Now open log file.
				print "Reading for ", alg, r, t
				lf = '%s/%s/%s_%s_Logs_%i.%i.%s.txt'%(folder, alg,pre1, alg, r,f,t)
				#'''
				if i == 1:
					lf = '%s_%s_Logs_%i.%i.%s.txt'%(pre1, alg, r,f,t)
				#'''
				print lf
				with open(lf, 'r') as fil:
					for l in fil.readlines():
						if metr in l:
							a = [float(x) for x in l.split(', ')[1:-1]]
							arr += a[len(a)/50:]
				# NEED abs deg metric. Read perf file, ignore first 1%.
				if 'Abs' in metr:
					pf = '%s/%s/%s_%s_perf_%i.%i.%s.out'%(folder, alg, pre1, alg, r,f,t)
					#'''
					if i == 1:
						pf = '%s_%s_perf_%i.%i.%s.out'%(pre1, alg, r,f,t)
					#'''
					print pf
					arr += read_abs_deg_metric(pf)
		s_arr = np.sort(arr)
		print metr, alg, "median : ", s_arr[len(arr)/2], ", 95p : ", s_arr[(95*len(arr))/100], ", 99p : ", s_arr[(99*len(arr))/100]
		pr = 1. * np.arange(len(arr))/(len(arr) - 1)
		print len(pr), len(s_arr)
		'''
		new_l = ((9.5*len(s_arr))/10)
		s_arr = s_arr[new_l:]
		pr = pr[new_l:]
		print len(pr), len(s_arr)
		'''
		plt.plot(s_arr, pr, sty[i], markersize=0, linewidth=3,label=alg)
		i += 1
	plt.title(metr + ' for RTC, Dyn at O%s MCS3.2'%(t))
	plt.xlabel(metr)
	plt.ylabel('CDF')
	plt.legend()
	plt.show()
	
