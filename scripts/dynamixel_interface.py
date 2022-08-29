#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import * 
from std_msgs.msg import Int64
import random
from dynamixel_sdk import * # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 104
    TTL = 100
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 1500         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 1500        # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    ADDR_PRESENT_POSITION       = 611
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    ADDR_PRESENT_POSITION       = 580
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'XL320':
    ADDR_TORQUE_ENABLE          = 24
    ADDR_GOAL_POSITION          = 30
    ADDR_PRESENT_POSITION       = 37
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
    BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

PROTOCOL_VERSION            = 2.0

# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyACM0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

DXL_ID_1 = 1
DXL_ID_2 = 2
DXL_ID_3 = 3
DXL_ID_4 = 4

def apply_pwm(DXL_ID, goal):
    print(goal)
    dxl_goal_position = [goal, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")

    # Write goal position
    if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])    
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])      
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))      
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))  

def read_pwm_motors(DXL_ID):
     # Read present position
    if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    else:
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))

    return dxl_present_position

def pwm_motor_1_callback(data):
    apply_pwm(DXL_ID_1, data.data)
          
def pwm_motor_2_callback(data):
    apply_pwm(DXL_ID_2, data.data)
    
def pwm_motor_3_callback(data):
    apply_pwm(DXL_ID_3, data.data)
        
def pwm_motor_4_callback(data):
    apply_pwm(DXL_ID_4, data.data)
    
if __name__=="__main__":
    rospy.init_node("dynamixel_interface_node", anonymous=False)

    index = 0
    dxl_goal_position = [1500, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")

    rospy.Subscriber("/ssl_robot/motor_1/write_pwm", Int64, pwm_motor_1_callback)
    rospy.Subscriber("/ssl_robot/motor_2/write_pwm", Int64, pwm_motor_2_callback)
    rospy.Subscriber("/ssl_robot/motor_3/write_pwm", Int64, pwm_motor_3_callback)
    rospy.Subscriber("/ssl_robot/motor_4/write_pwm", Int64, pwm_motor_4_callback)

    #rospy.spin()

    pub_read_1 = rospy.Publisher("/ssl_robot/motor_1/read_pwm", Int64, queue_size=10)
    pub_read_2 = rospy.Publisher("/ssl_robot/motor_2/read_pwm", Int64, queue_size=10)
    pub_read_3 = rospy.Publisher("/ssl_robot/motor_3/read_pwm", Int64, queue_size=10)
    pub_read_4 = rospy.Publisher("/ssl_robot/motor_4/read_pwm", Int64, queue_size=10)
    r = rospy.Rate(5)
    velocity = Twist()
    while not rospy.is_shutdown():
        pub_read_1.publish(read_pwm_motors(DXL_ID_1))
        pub_read_2.publish(read_pwm_motors(DXL_ID_2)) 
        pub_read_3.publish(read_pwm_motors(DXL_ID_3)) 
        pub_read_4.publish(read_pwm_motors(DXL_ID_4)) 
        r.sleep()
