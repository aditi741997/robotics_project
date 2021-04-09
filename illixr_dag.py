
open_vins = 2
gtsam_integrator = 3
pose_prediction = 4
gldemo = 5
timewarp_gl = 6
offline_imu = 7
offline_cam = 8
debugview = 9

with open("illixr_dag.txt", "w") as f:
    f.write(f"""
N {offline_imu} 0.02 5 0 0
N {gtsam_integrator} 0.04 5 0 0
N {timewarp_gl} 0.15 0 8.33 8.33 
N {gldemo} 0.115 0 0 0
N {offline_cam} 3.5 0 0 0
N {open_vins} 19.3 0 0 0
E {offline_imu} {gtsam_integrator} {open_vins} X
E {open_vins} {gtsam_integrator}  X
E {gldemo} {timewarp_gl} X
E {gtsam_integrator} {gldemo} {timewarp_gl} X
E {offline_cam} {open_vins} X
C 17 {offline_imu} {gtsam_integrator} {timewarp_gl} X
C 22 {offline_imu} {gtsam_integrator} {gldemo} {timewarp_gl} X
C 69 {offline_cam} {open_vins} {timewarp_gl} X
C 77 {offline_cam} {open_vins} {gldemo} {timewarp_gl} X
C 69 {offline_cam} {open_vins} {gtsam_integrator} {timewarp_gl} X
C 77 {offline_cam} {open_vins} {gtsam_integrator} {gldemo} {timewarp_gl} X
constr
""".lstrip())
#C 500 {offline_imu} {open_vins} {gtsam_integrator} {timewarp_gl} X
#C 500 {offline_imu} {open_vins} {gtsam_integrator} {gldemo} {timewarp_gl} X
