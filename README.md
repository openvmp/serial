# ROS2 serial

## Introduction

This is a part of [the OpenVMP project](https://github.com/openvmp/openvmp).

This package implements a node to configure and to communicate with serial ports on Linux.

## User's guide

### Basic setup

Launch it as an executable for each serial port.

```
TODO(clairbee): finish this
```


### Advanced setup

The more advanced setup is to initialize it as a node in the executable that will be communicating with it all the time (e.g. an RS485 implementation).

```
TODO(clairbee): finish this
```

This setup allows DDS to forward the messages between nodes without context switches should your DDS implementation support this.
