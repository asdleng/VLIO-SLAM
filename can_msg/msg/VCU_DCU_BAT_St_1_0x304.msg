# 由VCU发给DCU的反应车辆电池工作状态的信号
# ID_0x304
# 周期50ms, VCU发, DCU收

float64    batPackVolt              #电池母线电压: V
float64    batPackCurr              #电池母线电流: A
uint8      batPackSOC               #电池剩余电量: %
uint8      batFaultCode             #电池故障码
uint8      batChargeSt              #电池充电状态