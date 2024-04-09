# 由VCU发给DCU的反应遥控器状态的信号
# ID_0x301
# 周期50ms, VCU发, DCU收

float64    IDM_Steer_Angle_Front      #前轮方向盘转角   :-500~500
float64    IDM_Steer_Angle_Rear       #后轮方向盘转角   :-500~500
uint8      IDM_Live_Counter           #生命信号        :0~15滚动
uint8      IDM_Acc_Pedal              #驾驶员油门开度   :0~100%
uint8      IDM_Brk_Pedal              #驾驶员制动开度   :0~100%
uint8      IDM_Run_Mode_St            #车辆运行模式     :1遥控, 2自动驾驶
uint8      IDM_Ctr_Mode_St            #车辆控制模式     :1转矩模式, 2转速模式
uint8      IDM_EPB_St                 #EPB状态         :0松开, 1拉起
uint8      IDM_RND_Position_St        #档位状态         :0P，1R，2N, 3D
uint8      IDM_Front_Lamp_St          #前照灯状态
uint8      IDM_Steer_Lamp_St          #转向灯状态       :1左转向, 2右转向, 3双闪 
uint8      IDM_Sbus_Connect_St        #遥控连接状态      :0信号正常, 1信号丢失
uint8      IDM_Reserved               #保留