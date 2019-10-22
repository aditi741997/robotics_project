import sys
import matplotlib.pyplot as plt

pubf = [5, 10, 30, 50, 70]
#pubf = [10, 20, 30, 40, 50] #, 70] #, 90]
pf = {}
for i in range(len(pubf)):
    pf[pubf[i]] = i
#num_sam_orig = {360: 0, 720: 1, 1080: 2, 1440: 3, 1800: 4, 3600: 5}
num_sam = {1800: 0, 3600: 1}

x = ''
if len(sys.argv) > 1:
    x = sys.argv[1]

n = '2'
if len(sys.argv) > 2:
    n = sys.argv[2]

sub_freqs_file = 'overall_stats_%sc%s_NEW.txt'%(n, x)
cpu_file = 'tb3_sim_cpu_mem_%sc%s_NEW.txt'%(n, x)

move_base_cpu = {}
amcl_cpu = {}

move_base_subf = {}
costmap_subf = {}

move_base_cpu = [[-1.0 for i in pubf] for i in num_sam]
amcl_cpu = [[-1.0 for i in pubf] for i in num_sam]

move_base_subf = [[-1.0 for i in pubf] for i in num_sam]
lcostmap_subf = [[-1.0 for i in pubf] for i in num_sam]
gcostmap_subf = [[-1.0 for i in pubf] for i in num_sam]

mb_cmp_deltas = [[-1.0 for i in pubf] for i in num_sam]
mb_cmp_obs = [[-1.0 for i in pubf] for i in num_sam]
num_discards = [[-1 for i in pubf] for i in num_sam]
mb_counts = [[-1 for i in pubf] for i in num_sam]

plan_cmp_deltas = [[-1.0 for i in pubf] for i in num_sam]
plan_cmp_obs = [[-1.0 for i in pubf] for i in num_sam]
plan_counts = [[-1 for i in pubf] for i in num_sam]

mb_plan = [[-1.0 for i in pubf] for i in num_sam]
mb_plan_count = [[-1 for i in pubf] for i in num_sam]

# populate cpus
with open(cpu_file, 'r') as f:
    for l in f.readlines():
        if 'secondgoal' in l:
            l_a = l.split(', ')
            ns = num_sam[int(l_a[4])]
            pfr = pf[int(l_a[3])]
            move_base_cpu[ns][pfr] = float(l_a[12])
            amcl_cpu[ns][pfr] = float(l_a[13])

# populate avg times
with open(sub_freqs_file, 'r') as f:
    for l in f.readlines():
        l_a = l.split(', ')
        ns = num_sam[int(l_a[1])]
        pfr = pf[int(l_a[0])]
        mbc1 = int(l_a[3])
        mbc2 = int(l_a[5])
        s = float(l_a[6]) - float(l_a[4])
        move_base_subf[ns][pfr] = s/(mbc2 - mbc1)
        lcostmap_subf[ns][pfr] = (float(l_a[10]) - float(l_a[8]))/(int(l_a[9]) - int(l_a[7]))
        gcostmap_subf[ns][pfr] = (float(l_a[14]) - float(l_a[12]))/(int(l_a[13]) - int(l_a[11]))
        
        num_discards[ns][pfr] = int(l_a[15])
        mb_cmp_deltas[ns][pfr] = float(l_a[16])
        mb_cmp_obs[ns][pfr] = float(l_a[17])
        mb_counts[ns][pfr] = float(int(l_a[18]))

        plan_cmp_deltas[ns][pfr] = float(l_a[19])
        plan_cmp_obs[ns][pfr] = float(l_a[20])
        plan_counts[ns][pfr] = float(int(l_a[21]))

        mb_plan[ns][pfr] = float(l_a[22])
        mb_plan_count[ns][pfr] = float(int(l_a[23]))


# Avging for unknown data points (3)
"""
for ns in range(len(num_sam)):
    for pfr in range(len(pubf)):
        if move_base_subf[ns][pfr] == -1.0:
            print ns, pfr, "move base!"
            move_base_subf[ns][pfr] = (move_base_subf[ns][pfr-1] + move_base_subf[ns][pfr+1])/2.0
        if lcostmap_subf[ns][pfr] == -1.0:
            print ns, pfr, 'lcostmap!'
            lcostmap_subf[ns][pfr] = (lcostmap_subf[ns][pfr-1] + lcostmap_subf[ns][pfr+1])/2.0
        if gcostmap_subf[ns][pfr] == -1.0:
            print ns, pfr, 'gcostmap!'
            gcostmap_subf[ns][pfr] = (gcostmap_subf[ns][pfr-1] + gcostmap_subf[ns][pfr+1])/2.0
"""

for ns in range(len(num_sam)):
    for pfr in range(len(pubf)):
        num_discards[ns][pfr] /= mb_counts[ns][pfr]
        mb_cmp_deltas[ns][pfr] /= mb_counts[ns][pfr]
        mb_cmp_obs[ns][pfr] /= mb_counts[ns][pfr]

        mb_plan[ns][pfr] /= mb_plan_count[ns][pfr]

        plan_cmp_deltas[ns][pfr] /= plan_counts[ns][pfr]
        plan_cmp_obs[ns][pfr] /= plan_counts[ns][pfr]
        
        #mb_

"""
print move_base_cpu[0]
print lcostmap_subf[0]
print move_base_subf[0]
print gcostmap_subf[0]
"""
print move_base_subf[0], move_base_subf[1]
print mb_cmp_deltas[1], mb_cmp_obs[0]
print plan_cmp_deltas[0]
print num_discards[1]
print num_discards[0]

t = map(list, zip(*move_base_cpu))

# Plot 6 things : num disc, mb_cmp, mb_cmp_obs, mb_plan, plan_cmp, plan_cmp_obs
"""
# Plotting num discards
df = plt.plot(pubf, num_discards[0], 'b-', pubf, num_discards[1], 'g-')
plt.title('Num discards w.r.t. scan freq, b-1800, g-3600')
plt.show()

# Plotting mb_cmp
df1 = plt.plot(pubf, mb_cmp_deltas[0], 'b-', pubf, mb_cmp_deltas[1], 'g-')
plt.title('Avg delta(move_base_start - lcmp_last_update) w.r.t. scan freq, b-1800, g-3600')
plt.show()
"""

# mb_cmp_obs
df = plt.plot(pubf, mb_cmp_obs[0], 'b-', pubf, mb_cmp_obs[1], 'g-')
plt.title('Avg delta(mb - lcmp_obs) wrt scan freq, b-1800, g-3600)')
plt.show()

"""
# mb_plan
df = plt.plot(pubf, mb_plan[0], 'b-', pubf, mb_plan[1], 'g-')
plt.title('Avg delta(mb - plan) wrt scan freq, b-1800, g-3600)')
plt.show()

# Plotting plan_cmp
df = plt.plot(pubf, plan_cmp_deltas[0], 'b-', pubf, plan_cmp_deltas[1], 'g-')
plt.title('Avg delta(plan_cycle_start - gcmp_last_update) w.r.t. scan frequency, b-1800, g-3600')
plt.show()

# plan_cmp_obs
df = plt.plot(pubf, plan_cmp_obs[0], 'b-', pubf, plan_cmp_obs[1], 'g-')
plt.title('Avg delta(plan_cycle_start - gcmp_obsv) w.r.t. scan frequency, b-1800, g-3600')
plt.show()
"""


"""
# Plotting num discards
df = plt.plot(pubf, num_disc_scaled[0], 'b-', pubf, num_disc_scaled[1], 'g-')
plt.title('Num discards/Num loops w.r.t. scan frequency, b-1800, g-3600')
plt.show()
"""

#f = plt.plot(pubf, move_base_cpu[0], 'b-', pubf, move_base_cpu[1], 'g-', pubf, move_base_cpu[2], 'r-', pubf, move_base_cpu[3], 'c-', pubf, move_base_cpu[4], 'm-', pubf, move_base_cpu[5], 'y-')
f = plt.plot(pubf, move_base_cpu[0], 'b-', pubf, move_base_cpu[1], 'g-') #, pubf, move_base_cpu[2], 'r-') #, pubf, move_base_cpu[3], 'c-')
plt.title('Move base CPU, b-1800, g-3600')
plt.show()
"""
#f1 = plt.plot(pubf, amcl_cpu[0], 'b-', pubf, amcl_cpu[1], 'g-', pubf, amcl_cpu[2], 'r-', pubf, amcl_cpu[3], 'c-', pubf, amcl_cpu[4], 'm-', pubf, amcl_cpu[5], 'y-')
f1 = plt.plot(pubf, amcl_cpu[0], 'b-', pubf, amcl_cpu[1], 'g-', pubf, amcl_cpu[2], 'r-')#, pubf, amcl_cpu[3], 'c-')
plt.title('AMCL CPU')
plt.show()

#f2 = plt.plot(pubf, lcostmap_subf[0], 'b-', pubf, lcostmap_subf[1], 'g-', pubf, lcostmap_subf[2], 'r-', pubf, lcostmap_subf[3], 'c-', pubf, lcostmap_subf[4], 'm-', pubf, lcostmap_subf[5], 'y-')
f2 = plt.plot(pubf, lcostmap_subf[0], 'b-', pubf, lcostmap_subf[1], 'g-') #, pubf, lcostmap_subf[2], 'r-')#, pubf, lcostmap_subf[3], 'c-')
plt.title('Local Costmap Avg Time')
plt.show()

#f3 = plt.plot(pubf, gcostmap_subf[0], 'b-', pubf, gcostmap_subf[1], 'g-', pubf, gcostmap_subf[2], 'r-', pubf, gcostmap_subf[3], 'c-', pubf, gcostmap_subf[4], 'm-', pubf, gcostmap_subf[5], 'y-')
f3 = plt.plot(pubf, gcostmap_subf[0], 'b-', pubf, gcostmap_subf[1], 'g-') #, pubf, gcostmap_subf[2], 'r-')#, pubf, gcostmap_subf[3], 'c-')
plt.title('Global Costmap Avg Time')
plt.show()
"""

#f4 = plt.plot(pubf, move_base_subf[0], 'b-', pubf, move_base_subf[1], 'g-', pubf, move_base_subf[2], 'r-', pubf, move_base_subf[3], 'c-', pubf, move_base_subf[4], 'm-', pubf, move_base_subf[5], 'y-')
f4 = plt.plot(pubf, move_base_subf[0], 'b-', pubf, move_base_subf[1], 'g-') #, pubf, move_base_subf[2], 'r-')# pubf, move_base_subf[3], 'c-')
plt.title('Move Base Avg Time')
plt.show()


"""
fig = plt.figure()
ax = plt.subplot(111)
ax.plot(pubf, lcostmap_subf[0], 'g-')
plt.title('Local Costmap SubF vs Pub Freq')
plt.show()
fig.savefig('lcmp_360.png')
"""

# Plotting avg freq :
for ns in range(len(num_sam)):
    for pfr in range(len(pubf)):
        move_base_subf[ns][pfr] = 1.0/move_base_subf[ns][pfr]
        lcostmap_subf[ns][pfr] = 1.0/lcostmap_subf[ns][pfr]
        gcostmap_subf[ns][pfr] = 1.0/gcostmap_subf[ns][pfr]

print "Done with reciprocals"

#f5 = plt.plot(pubf, move_base_subf[0], 'b-', pubf, move_base_subf[1], 'g-', pubf, move_base_subf[2], 'r-', pubf, move_base_subf[3], 'c-', pubf, move_base_subf[4], 'm-', pubf, move_base_subf[5], 'y-')
f5 = plt.plot(pubf, move_base_subf[0], 'b-', pubf, move_base_subf[1], 'g-', pubf, move_base_subf[2], 'r-')#, pubf, move_base_subf[3], 'c-')
plt.title('Move base SubF')
plt.show()

#f6 = plt.plot(pubf, lcostmap_subf[0], 'b-', pubf, lcostmap_subf[1], 'g-', pubf, lcostmap_subf[2], 'r-', pubf, lcostmap_subf[3], 'c-', pubf, lcostmap_subf[4], 'm-', pubf, lcostmap_subf[5], 'y-')
f6 = plt.plot(pubf, lcostmap_subf[0], 'b-', pubf, lcostmap_subf[1], 'g-', pubf, lcostmap_subf[2], 'r-')#, pubf, lcostmap_subf[3], 'c-')
plt.title('Local costmap SubF')
plt.show()

#f7 = plt.plot(pubf, gcostmap_subf[0], 'b-', pubf, gcostmap_subf[1], 'g-', pubf, gcostmap_subf[2], 'r-', pubf, gcostmap_subf[3], 'c-', pubf, gcostmap_subf[4], 'm-', pubf, gcostmap_subf[5], 'y-')
f7 = plt.plot(pubf, gcostmap_subf[0], 'b-', pubf, gcostmap_subf[1], 'g-', pubf, gcostmap_subf[2], 'r-')#, pubf, gcostmap_subf[3], 'c-')
plt.title('Glocal Costmap SubF')
plt.show()
