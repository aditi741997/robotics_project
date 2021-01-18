
open_vins = 2
gtsam_integrator = 3
pose_prediction = 4
gldemo = 5
timewarp_gl = 6
debugview = 7
offline_imu = 8
offline_cam = 9

with open("illixr_dag.txt", "w") as f:
    f.write(f"""
N {offline_imu} 0.1 5
N {gtsam_integrator} 0.25 5
N {timewarp_gl} 4.0 0
N {gldemo} 2.0 0
N {offline_cam} 2 0
N {open_vins} 15 0
E {offline_imu} {gtsam_integrator} {open_vins} X
E {open_vins} {gtsam_integrator} X
E {gldemo} {timewarp_gl} X
E {gtsam_integrator} {gldemo} {timewarp_gl} X
E {offline_cam} {open_vins} X
C 40 {offline_imu} {gtsam_integrator} {timewarp_gl} X
C 400 {offline_imu} {gtsam_integrator} {gldemo} {timewarp_gl} X
C 500 {offline_imu} {open_vins} {gtsam_integrator} {timewarp_gl} X
C 500 {offline_imu} {open_vins} {gtsam_integrator} {gldemo} {timewarp_gl} X
C 500 {offline_cam} {open_vins} {gtsam_integrator} {timewarp_gl} X
C 500 {offline_cam} {open_vins} {gtsam_integrator} {gldemo} {timewarp_gl} X
""".lstrip())
