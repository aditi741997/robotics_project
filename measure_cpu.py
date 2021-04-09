import psutil
import time
import sys

# Nav : gzserver, move_base, amcl, robo state pub, rosout, mapsrv
# ObjTrack : gzserver, subscribr, objdetector, objtracker, controller
# Nav2D : stage, navigator, operator, mapper, rviz, joy, controller
cpu_util = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
mem_util = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

print(sys.argv)

# t = int(sys.argv[1])
# freq = int(sys.argv[2])

# tim = int(sys.argv[3])
# r = int(sys.argv[5])

sleep_time = 0.4
# n_o = tim/sleep_time

count = 0

def is_illixr_proc(proc) -> bool:
    try:
        exe = proc.exe()
    except psutil.AccessDenied:
        exe = ""
    return "main.opt.exe" in exe

def is_running():
    return any(map(is_illixr_proc, psutil.process_iter()))
        
def get_cpu_mem_nav2d():
    for proc in filter(is_illixr_proc, psutil.process_iter()):
        cpu_util[0] += proc.cpu_percent()

ts_arr = []

while not is_running():
    time.sleep(0.01)

print("Detected process launch")

while is_running():
    get_cpu_mem_nav2d()
    count += 1
    # print once every 10s i.e. 25*0.4s.
    if (count % 25 == 15):
        cpu = [x/count for x in cpu_util]
        mem = [x/count for x in mem_util]
        cms = "###Count: " + str(count) + "Avg CPU: " + str(cpu) + ", Mem: " + str(mem)
        print(cms)
        ts_arr.append(cms)
    time.sleep(sleep_time)

print("ADDED all observations", count)
cpu_txt = ""
mem_txt = ""

for i in range(len(cpu_util)):
    cpu_util[i] /= count
    cpu_txt += str(cpu_util[i]) + ", "
    mem_util[i] /= count
    mem_txt += str(mem_util[i]) + ", "

# fname = "%s_cpu_mem.txt"% (sys.argv[4])
f = sys.stdout
# f = open(fname, "w")

for i in sys.argv:
    f.write(i + ", ")

for j in ts_arr:
    f.write(j)
    f.write("\n")

f.write(str(count) + ", ")
f.write(cpu_txt)
f.write(mem_txt)
f.write("\n")
print(sys.argv)
print(cpu_util)
print(mem_util)
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
