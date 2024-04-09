# 由VCU发给DCU的反应车辆整车工作状态的信号
# ID_0x306
# 周期50ms, VCU发, DCU收

uint8     vehReadySt                    #车辆Ready状态
uint8     vehShiftPosition              #车辆档位状态 
uint8     vehRunningMode                #车辆运行模式, 转矩开环-转速开环
float64   vehSpeed                      #车速速度: km/h
float64   vehFrontSteeringAngle         #车辆前轮转角: °
float64   vehRearSteeringAngle          #车辆后轮转角: °
uint8     vehLightSt                    #车辆近光灯状态
uint8     vehLeftSteeringLamp           #车辆左转向灯状态
uint8     vehRightSteeringLamp          #车辆右转向灯状态
uint8     vehBrakingLamp                #车辆制动灯状态
uint8     vehEmergencyStop              #车辆急停请求状态

uint8     batFaultLampSt                #电池故障状态灯
uint8     brakeFaultLampSt              #制动故障灯
uint8     steerFaultLampSt              #转向故障灯
uint8     motorFaultLampSt              #电机故障灯
uint8     chargeLampSt                  #充电状态灯
