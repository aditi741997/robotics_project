import sys
import matplotlib.pyplot as plt

#farr = [5, 10, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100, 120, 150]
#farr = [5, 10, 20, 30, 40, 50, 60, 64, 66, 68, 70, 80, 100, 120, 150]
#farr = [5, 10, 20, 30, 37, 38, 40, 41, 42, 43, 45, 50, 60, 70]
#farr = [5, 15, 25, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 40, 45, 50]
farr = [5, 10, 12, 14, 16, 18, 19, 20, 21, 23, 27, 30, 40, 50, 67, 85, 100]
farr = [10, 15, 17, 20, 23, 26, 28, 30, 32, 35, 40, 60, 80, 100] #smallc1 2c

ind = {}
for i in range(len(farr)):
	ind[farr[i]] = i

perc_c1 = [0.0 for x in farr]
med_c1 = [0.0 for x in farr]
mean_c1 = [0.0 for x in farr]

perc_l1 = [0.0 for x in farr]
med_l1 = [0.0 for x in farr]
mean_l1 = [0.0 for x in farr]

#reading latency at N1:
with open(sys.argv[4], 'r') as f:
	la = f.readlines()
	for l in la:
		a = l.split(', ')
		i = ind[int(a[4])]
		perc_l1[i] = float(a[7])
		med_l1[i] = float(a[8])
		mean_l1[i] = float(a[9])		


p1 = plt.plot(farr, perc_l1, 'r^-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
plt.plot(farr, med_l1, 'g.:', label='Median')
plt.plot(farr, mean_l1, 'b*--', label='Mean')
plt.title('Latency at Node1 (inc. c1) for %s'%(sys.argv[3]))
plt.xlabel('Publisher Frequency')
plt.ylabel('Latency')
plt.ylim(0, 0.05)
plt.legend()
plt.show()

#reading latency at N2:
with open(sys.argv[1], 'r') as f:
	la = f.readlines()
	for l in la:
		a = l.split(', ')
		i = ind[int(a[4])]
		perc_l1[i] = float(a[7])
		med_l1[i] = float(a[8])
		mean_l1[i] = float(a[9])		


p1 = plt.plot(farr, perc_l1, 'r^-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
plt.plot(farr, med_l1, 'g.:', label='Median')
plt.plot(farr, mean_l1, 'b*--', label='Mean')
plt.title('Latency at Node2 (inc. c2) for %s'%(sys.argv[3]))
plt.xlabel('Publisher Frequency')
plt.ylabel('Latency')
plt.ylim(0, 0.08)
plt.legend()
plt.show()


perc_c2 = [0.0 for x in farr]
med_c2 = [0.0 for x in farr]
mean_c2 = [0.0 for x in farr]

perc_d = [0.0 for x in farr]
med_d = [0.0 for x in farr]
mean_d = [0.0 for x in farr]

perc_t = [0.0 for x in farr]
med_t = [0.0 for x in farr]
mean_t = [0.0 for x in farr]

perc_dt = [0.0 for x in farr]
med_dt = [0.0 for x in farr]
mean_dt = [0.0 for x in farr]

lost_msg = [0.0 for x in farr]

with open(sys.argv[2], 'r') as f:
	la = f.readlines()
	for l in la:
		a = l.split(', ')
		i = ind[int(a[4])]
		perc_d[i] = float(a[7])
		med_d[i] = float(a[8])
		mean_d[i] = float(a[9])
		
		perc_t[i] = float(a[10])
		med_t[i] = float(a[11])
		mean_t[i] = float(a[12])

		perc_dt[i] = float(a[14])
		med_dt[i] = float(a[15])
		mean_dt[i] = float(a[16])

		lost_msg[i] = (float(a[18])*100)/int(a[19])

		perc_c2[i] = float(a[25])
		med_c2[i] = float(a[26])
		mean_c2[i] = float(a[27])

#plt.ylim(0.01, 0.04)
p2 = plt.plot(farr, perc_d, 'r^-', label='99ile')
plt.plot(farr, med_d, 'g.:', label='Median')
plt.plot(farr, mean_d, 'b*--', label='Mean')
plt.title('d, %s'%(sys.argv[3]))
plt.xlabel('Publisher Frequency')
plt.ylabel('Message Delay (Latency)')
plt.legend()
plt.ylim(0, 0.1)
plt.show()

p3 = plt.plot(farr, perc_t, 'r^-', label='99ile')
plt.plot(farr, med_t, 'g.:', label='Median')
plt.plot(farr, mean_t, 'b*--', label='Mean')
plt.title('1/t, %s'%(sys.argv[3]))
plt.xlabel('Publisher Frequency')
plt.ylabel('Time b/w outputs')
plt.ylim(0, 0.12)
plt.legend()
plt.show()

p4 = plt.plot(farr, perc_dt, 'r^-', label='99ile')
plt.plot(farr, med_dt, 'g.:', label='Median')
plt.plot(farr, mean_dt, 'b*--', label='Mean')
plt.title('d+1/t, %s'%(sys.argv[3]))
plt.xlabel('Publisher Frequency')
plt.ylabel('Inreaction Time')
plt.legend()
plt.ylim(0, 0.25)
plt.show()

#reading c1 : 
with open(sys.argv[1], 'r') as f:
	la = f.readlines()
	for l in la:
		a = l.split(', ')
		i = ind[int(a[1])]
		perc_c1[i] = float(a[2])
		med_c1[i] = float(a[3])
		mean_c1[i] = float(a[4])
"""
		if "pub_c1" in l:
			a = l.split(' ')
			i = ind[int(a[17][:-1])]
			perc_c1[i] = float(a[11][:-1])
			med_c1[i] = float(a[8][:-1])
			mean_c1[i] = float(a[14][:-1])
"""


plt.ylim(0.00, 0.032)
p1 = plt.plot(farr, perc_c1, 'r-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
plt.plot(farr, med_c1, 'g:', label='Median')
plt.plot(farr, mean_c1, 'b--', label='Mean')
plt.title('c1 for %s'%(sys.argv[3]))
plt.xlabel('Publisher Frequency')
plt.ylabel('Actual Processing Time of Publisher')
plt.legend()
plt.show()

#plt.ylim(0.005, 0.02)
p5 = plt.plot(farr, perc_c2, 'r-', label='99ile')
plt.plot(farr, med_c2, 'g:', label='Median')
plt.plot(farr, mean_c2, 'b--', label='Mean')
plt.title('c2, %s'%(sys.argv[3]))
plt.xlabel('Publisher Frequency')
plt.ylabel('Actual Processing Time of Subscriber')
plt.legend()
plt.show()

p6 = plt.plot(farr, lost_msg, 'r-')
plt.title('Fraction of msgs dropped %s'%(sys.argv[3]))
plt.xlabel('Publisher Frequency')
plt.ylabel('Fraction of Msg Drops')
plt.show()
