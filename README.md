# robotics_project

This section is up-to-date as of 2022-06-15. It may be out-of-date when you read it.

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

## The config file

The backend scheduler needs to know about ILLIXR's DAG. This is how it does so:

1. ILLIXR's DAG information is defined in the config YAML: [configs/native.yaml:scheduler](https://github.com/ILLIXR/ILLIXR/blob/91c6ff35db9fc48a6407fea2131aa35429383095/configs/native.yaml#L59)
2. The runner reads YAML on this line and generates a config file that Aditi's code knows how to read on this line: [runner/runner/main.py:116](https://github.com/ILLIXR/ILLIXR/blob/91c6ff35db9fc48a6407fea2131aa35429383095/runner/runner/main.py#L116)
  - This is an example of a file that Aditi's code knows how to read; the generated file is quite similar: [illixr\_dag.txt](https://github.com/aditi741997/robotics_project/blob/6c9813ec911bd418d2fd4380375362811bf06f3f/illixr_dag.txt)
3. ILLIXR runtime starts up.
4. ILLIXR starts the "scheduler plugin" defined in Aditi's code: [scheduler\_plugin/plugin.cpp](https://github.com/aditi741997/robotics_project/blob/b6ce775ba473590935380d8ed229f8b692e6d5c1/scheduler_plugin/plugin.cpp).
5. The plugin passes the generated file (hardcoded location) to the rest of Aditi's code to the `DAGControllerBE` on this line: [scheduler\_plugin/plugin.cpp:15](https://github.com/aditi741997/robotics_project/blob/b6ce775ba473590935380d8ed229f8b692e6d5c1/scheduler_plugin/plugin.cpp#L15)
6. The DAG frontend passes the file to `DAGMultiCore`: [src/beginner\_tutorials/src/dag\_controller\_be.cpp:L97](https://github.com/aditi741997/robotics_project/blob/master/src/beginner_tutorials/src/dag_controller_be.cpp#L97)
7. `DAGMultiCore` actually parses the file here: [src/beginner\_tutorials/src/dag.cpp:226](https://github.com/aditi741997/robotics_project/blob/3f7b77d1a3b3cec4876545e14f13f54da9a47593/src/beginner_tutorials/src/dag.cpp#L226)

## ILLIXR-specific variabels
- Swap-strategies: We had an uncertainty about what order to run the nodes in the critical chain, and whether to wait for render to complete. We chose a strategy that had theoretical advantages and empirically performed the best. Aditi can elaborate more here.

