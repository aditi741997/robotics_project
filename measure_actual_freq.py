import sys

fname = sys.argv[1]

print "Getting actual freq"
with open(fname, 'r') as f:
    fl = f.readlines()
    avg_s = 0.0
    avg_c = 0
    for l in fl[5:]:
        if "average" in l:
            la = l.split(' ')
            a = float(la[-1][:-1])
            avg_s += a
            avg_c += 1
    with open("%s_actual_freq.txt"%(sys.argv[4]), 'a') as fn:
        val = avg_s/avg_c
        fn.write("%s %s Actual_Freq: %f\n"%(sys.argv[2], sys.argv[3], val))