# Robot

all brains use multi-modal chat bot for understanding

all brains subscribe to sensor datas
all brains use tool calling to publish message

central brain publish global plan message

mini brain subscribe to global plan message
mini brain publish local plan message

simulator subscribe to local plan message
simulator publish the simulation result message

mini brain send action
during the action:
    decide if we have to stop action when global action update 
    decide if we have to stop when the simulator result message is bad
        do in-context learning while continue moving, formulate new combination of actions with simulation service

- Could not import 'rosidl_typesupport_c' for package 'interfaces'
  - Cmake python different one, therefore a problem due to venv: https://github.com/ros2/examples/issues/303#issuecomment-2221348998
  - https://robotics.stackexchange.com/questions/105587/ros2-on-windows-10-colcon-build-uses-wrong-python-version-no-module-named

- sound device
  - sudo apt-get install libportaudio2
  - sudo apt-get install python3-pyaudio 

- How to setup venv python for ros2
  - https://medium.com/ros2-tips-and-tricks/running-ros2-nodes-in-a-python-virtual-environment-b31c1b863cdb
```bash
source setup_workspace.sh 

# TODO don't forget to complete our launch file in the robot pkg

```

