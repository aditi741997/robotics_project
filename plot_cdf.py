import re
import numpy as np
import matplotlib.pyplot as plt
import sys
import math

folder = sys.argv[1]
pre1 = sys.argv[2]
pre2 = sys.argv[3]
t = (sys.argv[4])
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
	b = a[len(a)/10:]
	print "Length : ", len(a), "Ignoring " , len(a)/10, "Len b : ", len(b)
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
                        if a[-1] > math.pi*0.9999:
                                print hpp, hy, hx, ha, ts, tc, ta
	b=a[len(a)/5:]
	print "Len a " , len(a), "Ignoring ", len(a)/10, "Len b : ", len(b)
	return b

def plot_m_speed(t_arr, metr_dict, rt_dict, st):
	sp_arr = [8.0/x for x in t_arr]
	larr = ['ro-', 'b^-.', 'g*:']
	algs = metr_dict.keys()
	fs=30
	legloc='center left'
	legsz=24
	legcolsp=0.4
	legtextpad=0.2
	plt.figure(figsize=(8.,5.),dpi=120)
	for i in range(len(algs)):
		print i, algs
		plt.plot(sp_arr, metr_dict[algs[i]], larr[i],  markersize=9, linewidth=5, label=algs[i] + ' RT:' + str(rt_dict[algs[i]][0]) )
	plt.title('Comparing $\Delta$ Angle for different speeds %s'%(st))
	plt.xlabel('Speed', fontsize=fs)
	plt.ylabel('$\Delta$ Angle', fontsize=fs)
	plt.ylim(0, 3.2)
    	plt.xticks(fontsize=fs-2)
    	plt.yticks(fontsize=fs-1)
    	plt.legend(loc=legloc, prop={"size":legsz}, ncol=1, handlelength=1.5, columnspacing=legcolsp, handletextpad=legtextpad)
    	plt.grid()
    	plt.tight_layout()
    	fig=plt.gcf()
    	plt.show()
    	fig.savefig('Final_AllSpeeds_Algos_AbsDegMetric_%s.pdf'%(st))

sty = ['g:', 'b-', 'r--']
#need to merge all 5 runs
if __name__ == '__main__':
	tarr = [1.5, 2.3, 4, 8]
	tarr = [8, 4, 2.3, 1.5]
	m_95p = {}
	m_99p = {}
	m_med = {}
	for metr in ['RxnTime,', 'RelMetric,', 'RelMetric1', 'Abs Deg']:
		m_95p[metr] = {'RTC': [], 'Dyn' : [], 'Default' : [] }
		m_99p[metr] = {'RTC': [], 'Dyn' : [], 'Default' : [] }
		m_med[metr] = {'RTC': [], 'Dyn' : [], 'Default' : [] }
	for tt in tarr:
		t=str(tt)
		for metr in ['RxnTime,', 'RelMetric,', 'RelMetric1', 'Abs Deg']: #
			i = 0
			for alg in ['RTC', 'Dyn', 'Default']: #
				arr = []
				if alg =='Default':
					f=30
				else:
					f=15	
				for r in [1,2,3,4,5]: #
					#check validity
					hr = 0.0
					vf = '%s/O%s/%s/%s_%s_vision_node_%i.%i.%s.out'%(folder, t, alg,pre1, alg, r, f, t)
					#'''
					if i == 1:
						vf = 'FJ_Ser_Expt2_2c_%s_vision_node_%i.%i.%s.out'%(alg, r,f,t)
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
						lf = '%s/O%s/%s/%s_%s_Logs_%i.%i.%s.txt'%(folder, t, alg,pre1, alg, r,f,t)
						#'''
						if i == 1:
							lf = 'FJ_Ser_Expt2_2c_%s_Logs_%i.%i.%s.txt'%(alg, r,f,t)
							lf = '%s_%s_Logs_%i.%i.%s.txt'%(pre1, alg, r,f,t)
						#'''
						print lf
						with open(lf, 'r') as fil:
							for l in fil.readlines():
								if metr in l:
									a = [float(x) for x in l.split(', ')[1:-1]]
									arr += a[len(a)/10:]
						# NEED abs deg metric. Read perf file, ignore first 1%.
						if 'Abs' in metr:
							pf = '%s/O%s/%s/%s_%s_perf_%i.%i.%s.out'%(folder, t, alg, pre1, alg, r,f,t)
							#'''
							if i == 1:
								pf = 'FJ_Ser_Expt2_2c_%s_perf_%i.%i.%s.out'%(alg, r,f,t)
								pf = '%s_%s_perf_%i.%i.%s.out'%(pre1, alg, r,f,t)
							#'''
							print pf
							arr += read_new_abs_deg_metric(pf)
				s_arr = np.sort(arr)
				print metr, alg, "mean : ", sum(arr)/len(arr), "median : ", s_arr[len(arr)/2], ", 95p : ", s_arr[(95*len(arr))/100], ", 99p : ", s_arr[(99*len(arr))/100]
				m_95p[metr][alg].append(s_arr[(95*len(arr))/100])
				m_99p[metr][alg].append(s_arr[(99*len(arr))/100])
				m_med[metr][alg].append(s_arr[(len(arr))/2])
				pr = 1. * np.arange(len(arr))/(len(arr) - 1)
				print len(pr), metr, alg, len(s_arr)
				i += 1
				'''
				new_l = ((5*len(s_arr))/10)
				s_arr = s_arr[new_l:]
				pr = pr[new_l:]
				print len(pr), len(s_arr)
				'''
				'''
				plt.plot(s_arr, pr, sty[i], linewidth=3,label=alg)
			plt.title(metr + ' for RTC, Dyn at O%s MCS3.2'%(t))
			plt.xlabel(metr)
			plt.ylabel('CDF')
			plt.legend()
			plt.show()
			'''
	#we now have all the values.
	#plot speed on x axis, 3 lines for 95p of absdeg for RTC,Dyn,Def
	print m_95p['Abs Deg'], 'ABS DEG dict'
	print m_95p['RxnTime,'], 'RXNTIME dict'
	plot_m_speed(tarr, m_95p['Abs Deg'], m_95p['RxnTime,'], '95p')
	plot_m_speed(tarr, m_99p['Abs Deg'], m_95p['RxnTime,'], '99p')
	plot_m_speed(tarr, m_med['Abs Deg'], m_med['RxnTime,'], 'med')
