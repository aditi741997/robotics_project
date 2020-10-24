import sys
import matplotlib.pyplot as plt

fname_pre_str = sys.argv[1]
fname_post_str = sys.argv[2]

for fname in ["local_map", "mapper_mapUpdate", "mapper_scanCB", "navigator_cmd", "navigator_plan", "operator_loop"]:
    times = []
    ts = []
    scan_count = []
    with open(fname_pre_str + fname + fname_post_str, 'r') as f:
        for fl in f.readlines():
            if "imes:" in fl:
                times += [ float(x) for x in fl.split(" ")[2:-1] ]
            elif "ts:" in fl:
                ts += [ float(x) for x in fl.split(" ")[2:-1] ]
            elif "ScanCOunt" in fl:
                scan_count += [ int(x) for x in fl.split(" ")[2:-1] ]
    # plot times,ts and scan_count.
    plt.plot(ts, times, 'bo-', label=fname + " compute time")
    plt.title("Nav2d Node : %s"%(fname) )
    plt.legend()
    plt.show()
    if len(scan_count) > 0:
        msc = max(scan_count)
        # scan_count = [ (x*0.1/msc) for x in scan_count]
        plt.plot(ts, scan_count, 'g^:', label=fname + " Scan Count")
        plt.title("Nav2d NOde : %s #Scans"%(fname))
        plt.legend()
        plt.show()