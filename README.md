# OpenVMP

[![License](./license.svg)](./LICENSE.txt)

This package is a part of [the OpenVMP project](https://github.com/openvmp/openvmp).
But it's designed to be universal and usable independently from the rest of OpenVMP or in a combination with select OpenVMP packages.

## ROS2 serial driver

This is an ultimate C++ implementation of serial port driver for ROS2.

It can be used either as a library or a standalone process. In both cases it
provides ROS2 interfaces for inter-process access, introspection and
debugging.
It performs wisely in case of serial line saturation in any of
the I/O directions, minimizing data loses and blocking behavior,
ensuring maximum performance.


### Basic setup

Launch it as a separate node for each serial port:

```
$ ros2 run serial serial_standalone
```

or

```
$ ros2 run serial serial_standalone \
  --ros-args \
  --remap serial:__node:=serial_com1 \
  -p serial_prefix:=/serial/com1 \
  -p serial_dev_name:=/dev/ttyS0 \
  -p serial_baud_rate:=115200 \
  -p serial_data:=8 \
  -p serial_parity:=false \
  -p serial_stop:=1 \
  -p serial_flow_control:=true
```

```mermaid
flowchart TB
    cli["$ ros2 topic echo /serial/com1"] -. "DDS" .-> topic_serial[/ROS2 interfaces:\n/serial/com1/.../]
    app["Your process"] -- "DDS\n(with context switch)" --> topic_serial
    subgraph serial["Process: serial_standalone"]
      topic_serial --> driver["Serial port driver"]
    end
    driver --> file{{"Character device: /dev/ttyS0"}}
```

### Advanced setup

The more advanced setup is to initialize it as a separate node in the very executable which will be communicating with the port all the time
(e.g. a MODBUS RTU implementation).

This setup allows DDS to forward the messages between nodes
without context switches should your DDS implementation support that.
See an example of such a setup in the Modbus RTU package:

```mermaid
flowchart TB
    cli_serial["# Serial debugging\n$ ros2 topic echo /serial"] -. "DDS" ..-> topic_serial[/ROS2 interfaces:\n/serial/.../]
    subgraph modbus_exe["Your process"]
      subgraph serial["Library: serial"]
        topic_serial --> driver["Serial port driver"]
      end
      code["Your code"] -- "DDS\n(potentially without\ncontext switch)" --> topic_serial
      code -- "or native API calls" ---> driver
      driver --> file{{"Character device"}}
    end
```


### Implementation details

```mermaid
flowchart TB
    owner["Native API"]
    external["ROS2 Interface API"]

    subgraph node["serial::Node"]

      subgraph interface_ros2["serial::InterfaceRemote"]
        subgraph topics["Topics: /serial"]
          published_input[//inspect/input/]
          published_output[//inspect/output/]
        end

        subgraph services["Services: /serial"]
          service_send[//inject/output/]
          service_recv[//inject/input/]
        end
      end

      subgraph worker["serial::Worker"]
        output_queue["Output Queue"]
        thread["Worker thread\nrunning select()"]

        subgraph interface_native["serial::Implementation"]
          subgraph register_input_cb["register_input_cb()"]
            input_cb["input_cb()"]
          end
          output("output()")
        end

        fd["File Descriptor"]
      end

    end
    file{{"Character device"}}

    %% owner -- "Constructor\nwith parameters" ----> node
    owner --> output --> output_queue
    %% owner --> register_input_cb

    external .-> published_input
    external .-> published_output
    external .-> service_send
    external .-> service_recv

    service_recv --> published_input
    service_recv --> input_cb

    service_send --> output_queue
    output_queue --> thread
    thread --> published_input
    thread --> published_output
    thread ---> input_cb --> owner

    file -- "input" ---> fd -- "input" ----> thread
    thread -- "output" --> fd -- "output" --> file

```

