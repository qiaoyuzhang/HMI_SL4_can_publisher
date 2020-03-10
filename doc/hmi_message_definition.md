# HMI-CAN-MESSAGE-DEFINITION

definition is at [const_vars.h](./include/hmi_can_publisher/const_vars.h)


### Vehicle_General_INFO(0x300)

|Name|Type|Start|Len|Offset|Res|Min|Max|Unit|Note|
|---|---|---|---|---|---|---|---|---|---|
|speed  |int| 0| 8| 0 |1 |0| 255 |mph or kph, User-defined | |
|steering angle  |int| 8| 11| -1024 |1 |-1024| 1023 | degree | |
|engage_status  |int| 19| 2| 0 |1 |0| 3 | | 0: not ready<br> 1: ready<br>2: engaged |

### Obstacle_GeneraInfo(0x400)

|Name|Type|Start|Len|Offset|Res|Min|Max|Unit|Note|
|---|---|---|---|---|---|---|---|---|---|
|ob_num  |int| 0| 8| 0 |1 |0| 255 | | |

### Obstacle_ExtendedInfo(0x401)

|Name|Type|Start|Len|Offset|Res|Min|Max|Unit|Note|
|---|---|---|---|---|---|---|---|---|---|
|ob_id  |int| 0| 8| 0 |1 |0| 255 | | |
|lane_assignment |int| 8| 2| 0 |1 |0| 3 | |0: not used<br>1: left_lane<br>2: ego_lane<br>3: right_lane|
|ob_dist_long  |double| 10| 10| -10.0 |0.25 |-10.0| 245.75 |meter | |
|ob_dist_lat  |double| 20| 6| -3.2 | 0.1 |-3.2| 3.1 | meter | |
|obj_class  |int| 26| 5| 0 |1 |0| 31 | | [proto definition](https://github.com/PlusAI/drive/blob/master/common/proto/perception/obstacle_detection.proto#L44)|
|obj_Threat  |bool| 31| 1| 0 |1 |0| 1 | |False: Not threat<br>True: threat|

### Lane_General_INFO(0x500)
|Name|Type|Start|Len|Offset|Res|Min|Max|Unit|Note|
|---|---|---|---|---|---|---|---|---|---|
|left_lane_exist  |bool| 0| 1| 0 |1 |0|1 | | False: not exist<br> True: exist|
|right_lane_exist  |bool| 1| 1| 0 |1 |0|1 | | False: not exist<br> True: exist|

### Planning_General_INFO(0x600)

|Name|Type|Start|Len|Offset|Res|Min|Max|Unit|Note|
|---|---|---|---|---|---|---|---|---|---|
|allow_lane_change  |bool| 0| 1| 0 |1|0| 1 | | False: not allow<br> True: allow|
|left_lane_change_cmd  |bool| 1| 1| 0 |1 |0| 1 | | False: not allow<br> True: allow|
|right_lane_change_cmd  |bool| 2| 1| 0 |1 |0|1 | | False: not allow<br> True: allow|
|set_speed  |int| 3| 8| 0 |1 |0| 255 | mph or kph, User-defined| |
|speed_limit  |int| 11| 8| 0 |1 |0| 255 | mph or kph, User-defined| |
