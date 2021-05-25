# robotics_project

A repository for running scheduling experiments on robotics applications over ROS and Switchboard.

The code is divided into the back-end scheduler (`src/beginner_tutorials`) and the front-end plugin. The backend can be static with a priori compute-times or dynamic with observed compute times. There is a frontend for ROS (`src/$EXP` where `$EXP` is a ROS experiment configuration) and Switchboard (`scheduler_plugin`).

## The backend

- The class `DAGControllerBE` is the interface that the backend exposes to the frontend.
- `src/beginner_tutorials/src/dag_controller_be.cpp` is where the interesting scheduling decisions are made.
- `DAGControllerBE` takes a "DAG description file." See `./*_dag.txt` for examples.
  - Note that for ILLIXR, the runner sets the DAG file based on `config.yaml`.

## The Switchboard front-end

- The Switchboard front-end is defined as a plugin in `src/scheduler_plugin`. It must be loaded before all other plugins.
- It subscribes to `${PLUGIN_ID}_thread_start`, where `${PLUGIN_ID}_thread_start`, to receive the TID of each plugin.
  - The threadloop class when `self_scheduled=true` has been modified to comply.
  - Switchboard subscribers have been modified to comply.
- It publishes to `${PLUGIN_ID}_trigger` when it wants `${PLUGIN_ID}` to go.
  - The threadloop class (excluding cases where `self_scheduled=true`) is a wrapper around a Switchboard subcriber to `${PLUGIN_ID}_trigger`.
- It subscribes to `${PLUGIN_ID}_completion`, so that it knows when the plugin is done.
  - The threadloop class when `self_scheduled=true` has been modified to comply.
  - Switchboard subscribers have been modified to comply.

## ILLIXR-specific variabels
- Swap-strategies: We had an uncertainty about what order to run the nodes in the critical chain, and whether to wait for render to complete. We chose a strategy that had theoretical advantages and empirically performed the best. Aditi can elaborate more here.

