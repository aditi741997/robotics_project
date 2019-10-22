import sys
import re
import sys

def update_costmap_vals(l, cm_vals, s):
    l_arr = l.split(',')
    cm_vals[s].append(int(l_arr[3].split(' ')[-1]))
    cm_vals[s].append(float(ansi_rem.sub('', l_arr[4].split(' ')[-1])))

def write_arr(f_w, arr):
    s = ''
    for i in arr:
        s += str(i) + ', '
    f_w.write(s)

x = ''
if len(sys.argv) > 1:
    x = sys.argv[1]

n = '2'
if len(sys.argv) > 2:
    n = sys.argv[2]

ansi_rem = re.compile(r'\x1B[@-_][0-?]*[ -/]*[@-~]')
for scan_freq in [5, 10, 30, 50,70]: #[10, 20, 30, 40, 50, 70]: #,10 ]:   
    for num_samples in [1800, 3600]: #[360, 1800, 7200]: 
        for run in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
            with open('tb3_navgn_%sc%s_logs_%d%d%d.out'% (n, x, scan_freq, num_samples, run), 'r') as f:
                print "STARTING for ", scan_freq, num_samples
                f_arr = f.readlines()
                i = 0
                # add 1 #loops, avg time, 2 #loops, avg time
                move_base_vals = []

                num_discards_mb = 0
                num_discards_scaled = 0.0
                mb_cmp_delta_avg = 0.0
                plan_cmp_delta_avg = 0.0
                curr_plan_cmp_deltas = []
                curr_plan_cmp_count = []
                plan_cmp_deltas = []
                plan_cmp_count = []
            
                # for each : add 1 #loops, avg time, 2 #loops, avg time
                curr_vals = {'local_costmap': [], 'global_costmap': []}
                costmap_vals = {'local_costmap': [], 'global_costmap': []}
                while i < len(f_arr):
                    if 'REACHED' in f_arr[i]:
                        print f_arr[i], i
                        l_arr = f_arr[i].split(',')
                        try:
                            move_base_vals.append(int(l_arr[2].split(' ')[-1]))
                        except:
                            print "Failed mb count for ", scan_freq, num_samples
                            move_base_vals.append(0)
                        try:
                            move_base_vals.append(float(ansi_rem.sub('', l_arr[3].split(' ')[-1])))
                        except:
                            print "Failed for ", scan_freq, num_samples
                            move_base_vals.append(-1.0)
                        #use curr costmap vals :
                        for k in costmap_vals:
                            for j in curr_vals[k][-2:]:
                                costmap_vals[k].append(j)

                        #use curr plan cmp deltas
                        #plan_cmp_deltas += curr_plan_cmp_deltas[-1]
                        #plan_cmp_count += curr_plan_cmp_count[-1]

                        i += 1
                        """
                        while (('_costmap' not in f_arr[i]) and ('Map update count' not in f_arr[i])):
                            i += 1
                        print f_arr[i], i
                        #got costmap val :
                        next_cm = 'global_costmap'
                        s = 'local_costmap'
                        if 'global_costmap' in f_arr[i]:
                            next_cm = 'local_costmap'
                            s = 'global_costmap'
                        update_costmap_vals(f_arr[i], costmap_vals, s)
                        print next_cm
                        while ((next_cm not in f_arr[i]) or ('Map update count' not in f_arr[i])):
                            i += 1
                        print f_arr[i], i
                        update_costmap_vals(f_arr[i], costmap_vals, next_cm)
                        """
                    else:
                        if 'Map update count' in f_arr[i]:
                            print i, "Map count"
                            try:
                                if 'local_costmap' in f_arr[i]:
                                    update_costmap_vals(f_arr[i], curr_vals, 'local_costmap')
                                elif 'global_costmap' in f_arr[i]:
                                    update_costmap_vals(f_arr[i], curr_vals, 'global_costmap')
                            except:
                                print "Costmap failed for ", f_arr[i]
                        if 'obuf_discard' in f_arr[i]:
                            new_arr = ansi_rem.sub('', f_arr[i]).split(' ')
                            
                            mb_delta_count = int(new_arr[18][:-1])
                            num_discards_mb = int(new_arr[8][:-1])
                           
                            #num_discards_scaled = float(num_discards_mb)/int(new_arr[18][:-1])
                            print new_arr
                            
                            mb_cmp_delta_sum = float(new_arr[13][:-1])
                            mb_cmp_obs_delta_sum = float(new_arr[22][:-1])

                            print i, "obuf discard", num_discards_mb, "avg mb cmp : ", mb_cmp_delta_sum, f_arr[i]
                        
                        if 'mb_plan_delta' in f_arr[i]:
                            new_arr = ansi_rem.sub('', f_arr[i]).split(' ')
                            mb_plan_delta = float(new_arr[5])
                            mb_plan_count = int(new_arr[7])
                            print "mb plan delta %f count %d"%(mb_plan_delta, mb_plan_count)

                        if 'sum_plan_cmp_delta' in f_arr[i]:
                            new_arr = ansi_rem.sub('', f_arr[i]).split(' ')
                            #curr_plan_cmp_deltas += float(new_arr[6][:-1])
                            #curr_plan_cmp_count += int(new_arr[-1][:-1])

                            plan_cmp_delta_sum = float(new_arr[6][:-1])
                            plan_delta_count = int(new_arr[11][:-1])
                            plan_cmp_obs_delta_sum = float(new_arr[14][:-1])
                            print "avg plan cmp delta", f_arr[i], plan_cmp_delta_sum, plan_cmp_obs_delta_sum
                        i += 1
                        if len(move_base_vals) == 4:
                            break
                # plan_cmp_delta_avg = 
                if len(move_base_vals) == 4:
                    with open('overall_stats_%sc%s.txt'%(n, x), 'a') as f_w:
                        f_w.write('%d, %d, %d, '% (scan_freq, num_samples, run))
                        write_arr(f_w, move_base_vals)
                        for k in costmap_vals:
                            write_arr(f_w, costmap_vals[k])
                        f_w.write(str(num_discards_mb) + ', ' + str(mb_cmp_delta_sum) + ', ' + str(mb_cmp_obs_delta_sum) + ', ' + str(mb_delta_count) + ', ' + str(plan_cmp_delta_sum) + ', ' + str(plan_cmp_obs_delta_sum) + ', ' + str(plan_delta_count) + ', ' + '%f, %d, '%(mb_plan_delta, mb_plan_count))
                        f_w.write('\n')

