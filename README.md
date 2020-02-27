# HMI_SL4_CAN_PUBLISHER

## Goal:

1. Convert HMI-SL4 messages to CAN messages

2. Convert the CAN messages back to HMI-SL4 messages

3. Create a ros node listen the necessary ros topics and write to can

## Compile:

```bash
    $ ln -s $HOME/catkin_ws/src/hmi_can_publisher ../hmi_can_publisher
    $ cd $HOME/catkin_ws
    $ catkin_make install
```    

CAN Message Definition was in `hmi_message/const_vars.h`


## Install

This repo creates debian package `ros-kinetic-hmi-message-publisher`. To install this package, do
```bash
    $ sudo apt-get install ros-kinetic-hmi-message-publisher
```

## Example usage:
    
1. write hmi message and generate CAN message

``` 
    VehicelStatusGeneralInfo tmp;
    tmp.setSpeed(100);
    CanMsg = tmp.getVectorData();    
```
2. convert CAN message to hmi message
```
    VehicelStatusGeneralInfo rec_tmp(CanMsg)
```

More example is in  `hmi_message/test`

3. roslaunch: 
```
    $ roslaunch hmi_message_publisher hmi_message_publisher.launch
```
