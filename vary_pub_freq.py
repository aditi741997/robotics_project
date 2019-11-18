import sys


# CHANGING gazerbo scan publish frequency, num samples
fpath = sys.argv[3]
fname = sys.argv[4]

f = fpath + '/' + fname

with open(f, 'r') as fi:
    data = fi.readlines()

print data[106]
# Changing base scan publish frequency
new_scan_freq = sys.argv[1]
data[106] = "      <update_rate>"+new_scan_freq+"</update_rate>\n"

print data[110]
# Changing base scan no. of samples
new_num_samples = sys.argv[2]
data[110] = "            <samples>"+new_num_samples+"</samples>\n"

with open(f, 'w') as fil:
    fil.writelines(data)

# CHANGING scan freq, num samples as input to move_base
if len(sys.argv) > 5:
    fname1 = sys.argv[5]
    with open(fname1, 'r') as f1:
        data2 = f1.readlines()
    
    _scan_freq = "scan_frequency: "+new_scan_freq+"\n"
    data2.append(_scan_freq)

    _num_sample = "num_samples: "+new_num_samples+"\n"
    data2.append(_num_sample)
    print data2

    with open(fname1, 'w') as f:
        f.writelines(data2)

# CHANGING expected_update_rate as inout to costmaps
if len(sys.argv) > 6:
    fname = sys.argv[6]
    with open(fname, 'r') as f:
        d = f.readlines()
    
    print d[-1]
    d1 = d[-1].split('expected_update_rate')
    dnew = d1[0] + 'expected_update_rate: ' + str(1.3/float(new_scan_freq)) + '}\n'
    print dnew

    d[-1] = dnew

# FOR R EXPTS : Not discarding anything to see if sub times increase with higher frequencies. (Less idle time, limited resource)
#    with open(fname, 'w') as f:
#        f.writelines(d)

# CHANGING costmap update freq for both local, global
if len(sys.argv) > 7:
    path = sys.argv[7]
    for x in ['local_', 'global_']:
        fname = path + x + 'costmap_params.yaml'
        with open(fname, 'r') as f:
            d = f.readlines()

        print d[4]
        d[4] = "  update_frequency: " + new_scan_freq + ".0\n"

        print d[4]

        with open(fname, 'w') as f:
            f.writelines(d)

# CHANGING plan update frequency, move base freq
if len(sys.argv) > 8:
    path = sys.argv[8]
    with open(path, 'r') as f:
        fl = f.readlines()

    print fl[1], fl[5]
    fl[1] = "controller_frequency: " + new_scan_freq + "\n"
    fl[5] = "planner_frequency: %s\n"%(new_scan_freq)

    print fl[1], fl[5]

    with open(path, 'w') as f:
        f.writelines(fl)
