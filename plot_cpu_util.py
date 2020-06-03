#CPU UTIL vs FREQ :
# AVG OF AVG ACROSS 5RUNS :
import sys
import matplotlib.pyplot as plt

fr = [7, 11, 16, 20, 30, 60]
fr = [9,13,16,20,30,60]
t = int(sys.argv[2])

mean_sub = [0.0 for f in fr]
mean_det = [0.0 for f in fr]
mean_track = [0.0 for f in fr]

ct = [0 for f in fr]

ind = {}
for i in range(len(fr)):
	ind[fr[i]] = i

with open(sys.argv[1], 'r') as f:
	for l in f.readlines():
		la = l.split(', ')
		freq = int(la[2])
		if int(la[1]) == t and freq in fr:
			ct[ind[freq]] += 1
			mean_sub[ind[freq]] += float(la[8])
			mean_det[ind[freq]] += float(la[9])
			mean_track[ind[freq]] += float(la[10])

print "Counts : ", ct
mean_total = [0.0 for f in fr]
for f in fr:
	count = ct[ind[f]]
	mean_sub[ind[f]] /= count
	mean_det[ind[f]] /= count
	mean_track[ind[f]] /= count
	mean_total[ind[f]] = mean_sub[ind[f]] + mean_det[ind[f]] + mean_track[ind[f]]

# PLOT!
plt.plot(fr, mean_sub, 'b*--', markersize=9, linewidth=2, label='Node1')
plt.plot(fr, mean_det, 'g^-.', markersize=9, linewidth=2, label='Node2')
plt.plot(fr, mean_track, 'yo--', markersize=9, linewidth=2, label='Node3')
plt.plot(fr, mean_total, 'r.-', markersize=9, linewidth=2, label='Total')
plt.title('CPU Util of Nodes w.r.t. Freq for %s'%(sys.argv[3]))
plt.xlabel('Freq')
plt.ylabel('CPU %age')
plt.legend()
plt.show()
