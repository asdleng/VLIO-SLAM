Header header   # standard ROS message header


#Front and Rear uss sensors with CAN frame ID: ECU 0x5C8
#There are 3 uss sensors in the front and rear side of vehicle respectively.

float32     UDAS_RA_RearR           # Rear right uss sensor
float32     UDAS_RB_RearM			# Rear middle uss sensor 
float32     UDAS_RC_RearL           # Rear left uss sensor
float32     UDAS_FA_FrontL          # Front left uss sensor
float32     UDAS_FB_FrontM			# Front middle uss sensor
float32     UDAS_FC_FrontR          # Front right uss sensor

# Left and Right uss sensors with CAN frame ID: ECU 0x5C9
# There are 4 uss sensors in the left and right side of vehicle respectively.

float32     UDAS_RA_RightF			# The nearest uss sensor from vehicle head
float32     UDAS_RB_RightFM         # The second nearest uss sensor from vehicle head
float32     UDAS_RC_RightRM			# The third nearest uss sensor from vehicle head
float32     UDAS_RD_RightR          # The farthest uss sensor from vechile head

float32     UDAS_FA_LeftR           # The farthest uss sensor from vehicle head
float32     UDAS_FB_LeftRM          # The third nearest uss sensor from vehicle head
float32     UDAS_FC_LeftFM          # The second nearest uss sensor from vehicle head
float32     UDAS_FD_LeftF           # The nearest uss sensor from vechile head








