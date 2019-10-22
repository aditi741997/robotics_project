import sys
import os.path

x = sys.argv[1]
n = sys.argv[2]
mb_sub_sums = {} #for each 
mb_sub_counts = {}

gcmp_sub_sums = {}
gcmp_sub_counts = {}

lcmp_sub_sums = {}
lcmp_sub_counts = {}

discards_sums = {}
mb_cmp_sums = {}
mb_cmp_obs_sums = {}
mb_deltas_counts = {}

mb_plan_sums = {}
mb_plan_counts = {}

plan_cmp_sums = {}
plan_cmp_obs_sums = {}
plan_deltas_counts = {}

def addElem(d, k, v):
    d[k] = (v + d[k]) if k in d else v

with open('overall_stats_%sc_%s.txt'%(n, x), 'r') as f:
    #for each line - check if err log file has 'Aborting' in it'.
    #if not, add the values
    #average over counts in the end.
    lines = f.readlines()
    for l in lines:
        larr = l.split(', ')
        p = 'tb3_navgn_%sc_%s_errlogs_%s%s%s.out'%(n, x, larr[0], larr[1], larr[2])
        if os.path.exists(p):
            with open(p, 'r') as f1:
                a = f1.read()
                if 'Aborting ' in a:
                    continue
        #ALSO CHECK IF REACHED written twice in logs
        with open('tb3_navgn_%sc_%s_logs_%s%s%s.out'%(n, x, larr[0], larr[1], larr[2]), 'r') as f2:
            a = f2.read()
            if a.count('REACHED') != 2:
                continue
        # Only add these values if both goals reached AND not aborted.
        k = larr[0] + ', ' + larr[1] + ', n, '

        mbs = (int(larr[5])*float(larr[6]) - int(larr[3])*float(larr[4]))
        mb_sub_sums[k] = (mbs + mb_sub_sums[k]) if k in mb_sub_sums else mbs

        mbsc = int(larr[5]) - int(larr[3])
        mb_sub_counts[k] = (mbsc + mb_sub_counts[k]) if k in mb_sub_counts else mbsc

        lcmps = float(larr[10]) - float(larr[8])
        addElem(lcmp_sub_sums, k, lcmps)

        lcmpc = int(larr[9]) - int(larr[7])
        lcmp_sub_counts[k] = (lcmpc + lcmp_sub_counts[k]) if k in lcmp_sub_counts else lcmpc

        gcmps = float(larr[14]) - float(larr[12])
        gcmp_sub_sums[k] = (gcmps + gcmp_sub_sums[k]) if k in gcmp_sub_sums else gcmps

        gcmpc = int(larr[13]) - int(larr[11])
        gcmp_sub_counts[k] = (gcmpc + gcmp_sub_counts[k]) if k in gcmp_sub_counts else gcmpc

        disc = int(larr[15])
        addElem(discards_sums, k, disc)

        mbcmp = float(larr[16])
        addElem(mb_cmp_sums, k, mbcmp)

        mbcmpobs = float(larr[17])
        addElem(mb_cmp_obs_sums, k, mbcmpobs)

        mbd = int(larr[18])
        addElem(mb_deltas_counts, k, mbd)

        plcmp = float(larr[19])
        addElem(plan_cmp_sums, k, plcmp)

        plcmpobs = float(larr[20])
        addElem(plan_cmp_obs_sums, k, plcmpobs)

        pld = int(larr[21])
        addElem(plan_deltas_counts, k, pld)
        
        mbps = float(larr[22])
        addElem(mb_plan_sums, k, mbps)
        
        mbpc = int(larr[23])
        addElem(mb_plan_counts, k, mbpc)

        print "Added stuff to dict", mb_sub_sums, lcmp_sub_sums, mb_sub_counts, mb_cmp_sums, plan_deltas_counts
        print "Other dicts : ", lcmp_sub_counts, gcmp_sub_sums, gcmp_sub_counts
        print "Disc : ", discards_sums, mb_cmp_obs_sums, plan_cmp_sums, plan_cmp_obs_sums
        print "Mb plan : ", mb_plan_sums, mb_plan_counts
    print "All lines done!"

    with open('overall_stats_%sc_%s_NEW.txt'%(n, x), 'w') as f2:
        for k in mb_sub_sums:
            f2.write(k)
            f2.write('0, 0.0, %d, %f, '%(mb_sub_counts[k], mb_sub_sums[k]))
            f2.write('0, 0.0, %d, %f, '%(lcmp_sub_counts[k], lcmp_sub_sums[k]))
            f2.write('0, 0.0, %d, %f, '%(gcmp_sub_counts[k], gcmp_sub_sums[k]))
            f2.write('%d, '%(discards_sums[k]))
            f2.write('%f, '%(mb_cmp_sums[k]))
            f2.write('%f, '%(mb_cmp_obs_sums[k]))
            f2.write('%d, '%(mb_deltas_counts[k]))
            f2.write('%f, '%(plan_cmp_sums[k]))
            f2.write('%f, '%(plan_cmp_obs_sums[k]))
            f2.write('%d, '%(plan_deltas_counts[k]))
            f2.write('%f, %d, '%(mb_plan_sums[k], mb_plan_counts[k]))
            f2.write('\n')
