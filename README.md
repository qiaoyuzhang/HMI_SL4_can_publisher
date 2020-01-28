# HMI_SL4_CAN_PUBLISHER

Goal:

1. Convert HMI-SL4 messages to CAN messages

2. Convert the CAN messages back to HMI-SL4 messages

3. Create a ros node listen the necessary ros topics and write to can

Compile:
```
    $cd HMI_SL4_can_publisher/
    $make
```    

CAN Message Definition was in `hmi_message/const_vars.h`

Example usage:
    
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

3. ros node: 
```
    $./build/debug/hmi_message_publisher/ros_hmi_message_can_publisher --HMI_SL4_can_index 0x100
```
