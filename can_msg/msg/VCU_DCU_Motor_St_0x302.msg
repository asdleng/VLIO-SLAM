# 由VCU发给DCU的反应车辆电机工作状态的信号
# ID_0x302
# 周期50ms, VCU发, DCU收

float64     motorSpeed                  #电机转速
float64     motorTorque                 #电机扭矩
float64     motorTemp                   #电机温度
uint8       motorBrake                  #电机报闸状态
uint8       motorWorkst                 #电机工作模式
uint8       motorFaultCode              #电机故障码