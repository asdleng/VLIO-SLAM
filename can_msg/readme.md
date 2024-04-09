
-----
# CAN_MSG的说明

----

### 结构说明

- 1.  包含msg文件夹及编译说明文件CMakeLists.
- 2.  msg文件夹内, 定义了多种msg文件类型, 包括原始CAN帧msg, 基于具体CAN_ID定义的msg, 超声波msg, 及底盘状态类型的msg. 


### 使用说明

- 1. 使用时, 要先在自身CMakeLists文件中, 寻找find(can_msg.cmake)包, 然后在.文件中, 要添加include<can_msgs/CANFrame.h>.

- 2. 如果需要添加, 请注意书写规范, 及适当添加说明在md文件中.


### 历史记录

- 1. 添加CAN相关信号, 用于解析VCU转发的报文, 解析后可用于pad屏幕显示底盘状态信号, 以及给规划控制反馈底盘各部件状态. -2022.8.15