open_vins = 2
gtsam_integrator = 3
pose_prediction = 4
gldemo = 5
timewarp_gl = 6
offline_imu = 7
offline_cam = 8
debugview = 9

dag = {
    "nodes": [
        (offline_imu,      {5300:   .02,  2600:   .020}, 5, 0,    0,    [gtsam_integrator, open_vins]),
        (gtsam_integrator, {5300:   .04,  2600:   .075}, 5, 0,    0,    [gldemo, timewarp_gl]),
        (timewarp_gl,      {5300:   .15,  2600:   .05 }, 0, 8.33, 8.33, []),
        (gldemo,           {5300:   .115, 2600:   .102}, 0, 0,    0,    [timewarp_gl]),
        (offline_cam,      {5300:  3.5,   2600:  6.0  }, 0, 0,    0,    [open_vins]),
        (open_vins,        {5300: 19.3,   2600: 22.7  }, 0, 0,    0,    [gtsam_integrator]),
    ],
    "chains": [
        (17, [offline_imu, gtsam_integrator, timewarp_gl]),
        (22, [offline_imu, gtsam_integrator, gldemo, timewarp_gl]),
        (69, [offline_cam, open_vins, timewarp_gl]),
        (77, [offline_cam, open_vins, gldemo, timewarp_gl]),
        (69, [offline_cam, open_vins, gtsam_integrator, timewarp_gl]),
        (77, [offline_cam, open_vins, gtsam_integrator, gldemo, timewarp_gl]),
    ],
    "constr": True,
    "fc": {5300: 3, 2600: 4},
}
freqs = {5300, 2600}

for freq in freqs:
    with open(f"illixr_dag_{freq}.txt", "w") as f:
        for node, ctmap, b, c, d, _ in dag["nodes"]:
            f.write(f"N {node} {ctmap[freq]} {b} {c} {d}\n")
        for node, _, _, _, _, neighbors in dag["nodes"]:
            if neighbors:
                f.write(f"E {node} {' '.join(map(str, neighbors))} X\n")
        for weight, chain in dag["chains"]:
            if chain:
                f.write(f"C {weight} {' '.join(map(str, chain))} X\n")
        if dag["constr"]:
            f.write("constr\n")
    with open(f"illixr_fc_{freq}.txt", "w") as f:
        f.write(str(dag["fc"][freq]))
