# Progress

- Fundamentals
- New Switchboard not merged in.
- (Almost) have response-times, latencies, and throughputs.
- Scheduler plugin
- Threadloop be "triggerable", so that they can be scheduler, unless marked otherwise
- Threadloop and switchboard threads send their thread_id at startup
- Threadloop and switchboard threads send a completion notification
- Published thread info (ID and name)
- Critical chain signal completion
- Make gldemo triggered by our scheduler (not vsync)
- DAG changes
  - Leave as is, IMU int is sync on IMU

# Changes to do to core ILLIXR
