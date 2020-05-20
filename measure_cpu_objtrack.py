import psutil
import time
import sys

# gzserver, move_base, amcl, robo state pub, rosout, mapsrv
cpu_util = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
mem_util = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

print sys.argv

t = int(sys.argv[1])
freq = int(sys.argv[2])

tim = int(sys.argv[3])
r = int(sys.argv[5])

sleep_time = 0.4
n_o = tim/sleep_time
print "Taking n_no obs : ", tim/sleep_time

count = 0
'''
def check_done():
    with open(sys.argv[5], 'r') as f:
        a = f.read()
        if a.count('REACHED') == int(sys.argv[6]) or count > int(sys.argv[7]):
            return True
    return False
'''

def get_cpu_mem():
    for proc in psutil.process_iter():
        pname = proc.name()
        if 'gzserver' in pname:
            cpu_util[0] += proc.cpu_percent()
            mem_util[0] += proc.memory_percent()
	if 'subscriber' in pname:
	    cpu_util[1] += proc.cpu_percent()
	if 'objdetector' in pname:
	    cpu_util[2] += proc.cpu_percent()
	if 'objtracker' in pname:
	    cpu_util[3] += proc.cpu_percent()
	if 'controller' in pname:
	    cpu_util[4] += proc.cpu_percent()
'''
For navigation:
        if 'move_base' in pname:
            a = proc.cpu_percent()
            cpu_util[1] += a
            mem_util[1] += proc.memory_percent()
            if sys.argv[8] == 'yes':
                move_base_cpu_arr.append(a)
        if 'amcl' in pname:
            a = proc.cpu_percent()
            cpu_util[2] += a
            mem_util[2] += proc.memory_percent()
            if sys.argv[8] == 'yes':
                amcl_cpu_arr.append(a)
        if 'robot_state_publisher' in pname:
            cpu_util[3] += proc.cpu_percent()
            mem_util[3] += proc.memory_percent()
        if 'map_server' in pname:
            cpu_util[4] += proc.cpu_percent()
            mem_util[4] += proc.memory_percent()
        if 'rosout' in pname:
            cpu_util[5] += proc.cpu_percent()
            mem_util[5] += proc.memory_percent()
'''
for i in range(int(n_o)):
    get_cpu_mem()
    count += 1
    time.sleep(sleep_time)

print "Added all observations", count, n_o
cpu_txt = ""
mem_txt = ""

for i in range(len(cpu_util)):
    cpu_util[i] /= count
    cpu_txt += str(cpu_util[i]) + ", "
    mem_util[i] /= count
    mem_txt += str(mem_util[i]) + ", "

fname = "%s_cpu_mem.txt"% (sys.argv[4])
f = open(fname, "a")

for i in sys.argv:
    f.write(i + ", ")

f.write(str(count) + ", ")
f.write(cpu_txt)
f.write(mem_txt)
f.write("\n")
print sys.argv
print cpu_util
print mem_util
'''
if sys.argv[8] == 'yes':
    with open('cpu_time_series_%s_%s%s.txt'% (sys.argv[3], sys.argv[4], sys.argv[9]), 'a') as fw:
        print "Writing to file for ", sys.argv[3], sys.argv[4], sys.argv[9]
        for i in move_base_cpu_arr:
            fw.write(str(i) + ', ')
        fw.write('\n')
        for i in amcl_cpu_arr:
            fw.write(str(i) + ', ')
        fw.write('\n')
'''
