

float64     dcu_AccPedal                 #DCU发送的加速请求开度命令(油门踏板开度): %
uint8       dcu_BrakePedal               #DCU发送的制动请求开度命令(油门踏板开度): %
float64     dcu_FrontSteeringAngle       #DCU发送的前转向角开度命令: °
float64     dcu_RearSteeringAngle        #DCU发送的后转向角开度命令: °
float64     dcu_TrgSpdReq                #DCU发送的目标车速请求: km/h
uint8       dcu_GearCmd                  #DCU发送的档位命令;(P档自动拉起EPB)
uint8       dcu_FrontLamp           	 #前照灯状态
uint8       dcu_SteerLamp                #转向灯状态       :1左转向, 2右转向, 3双闪 
uint8       dcu_RunMode                  #车辆运行模式     :1遥控, 2自动驾驶
uint8       dcu_CtrMode                  #车辆控制模式     :1转矩模式(发油门踏板), 2转速模式(发送目标车速)
uint8       dcu_SteerMode                #转向模式         :0 4WS模式, 1 FWS模式
uint8       dcu_LiveCounter              #生命信号         :0~15滚动(必须要发的)
uint8       dcu_AdScreenCmd              #远程开启或关闭广告屏
