# Progress

- Fundamentals
- Switchboard 2.0 not merged in.

# Changes done core ILLIXR

- Scheduler plugin
  - Plugin vs core? tentatively: plugin
- Threadloop be "triggerable", so that they can be scehduler
- Published thread info (ID and name)
- Critical chain signal completion

# Changes to do to core ILLIXR

- DAG changes
  - gtsam should be split into: buferring thread (possibly fused with IMU thread) and triggered by our scheduler
  - gtsam should trigger timewarp
  - Timewarp should trigger hologram
  - Make gldemo triggered by our scheduler (not vsync)


# Questions

- Why does Hologram need to wait for vsync?
- Other things that application does?
- Can we switch to Realsense?
  - It is mainlined
- Do IMU and camera need harmonicity?
