import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
from pyquaternion import Quaternion           #四元数转化
import sys

rospy.init_node("communication")  #输出
rate = rospy.Rate(30)

class Communication:

    def __init__(self):
        
        self.vehicle_type = "drone"
        self.vehicle_id = 0
        self.current_position = None
        self.current_yaw = 0
        self.hover_flag = 0
        self.target_motion = PositionTarget()
        self.arm_state = False
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        self.last_cmd = None
        
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback,queue_size=10)
        self.cmd_sub = rospy.Subscriber("/cmd",String,self.cmd_callback,queue_size=30)
        self.cmd_vel_flu_sub = rospy.Subscriber("/cmd_vel_flu", Twist, self.cmd_vel_flu_callback,queue_size=10)
        self.cmd_pose_enu_sub = rospy.Subscriber("/cmd_pose_enu", Pose, self.cmd_pose_enu_callback,queue_size=1)
        #  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        #     ("mavros/setpoint_position/local", 10);
        
        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)     
        self.flightModeService = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        print("communication initialized")
    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion =self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)
        
    def local_pose_callback(self, msg):
        self.current_position = msg.pose.position
        self.current_yaw = self.q2yaw(msg.pose.orientation)   #保持现在的位置
    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def start(self):
        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            self.target_motion_pub.publish(self.target_motion)
            rate.sleep()
    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame
        
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz
        
        target_raw_pose.acceleration_or_force.x = afx
        target_raw_pose.acceleration_or_force.y = afy
        target_raw_pose.acceleration_or_force.z = afz

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate #速度

        if(self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE
        if(self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW
        # if(self.motion_type == 2):
        #     target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
        #                     + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
        #                     + PositionTarget.IGNORE_YAW

        return target_raw_pose
    def cmd_vel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8  #坐标系
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z) 

    def hover_state_transition(self,x,y,z,w):
        if abs(x) > 0.02 or abs(y)  > 0.02 or abs(z)  > 0.02 or abs(w)  > 0.005:
            self.hover_flag = 0
            self.flight_mode = 'OFFBOARD'
        elif not self.flight_mode == "HOVER":   #一段时间
            self.hover_flag = 1
            self.flight_mode = 'HOVER'
            self.hover()

    def cmd_callback(self, msg):
        if msg.data == self.last_cmd or msg.data == '' :
            return

        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print("Armed "+str(self.arm_state))

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            print("Armed "+str(self.arm_state))

        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

        self.last_cmd = msg.data

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("disarming failed!")
            return False

    def hover(self):
        self.coordinate_frame = 1
        self.motion_type = 0
        self.target_motion = self.construct_target(x=self.current_position.x,y=self.current_position.y,z=self.current_position.z,yaw=self.current_yaw)
        print(self.flight_mode)

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
        elif self.flightModeService(custom_mode=self.flight_mode):                   #使用括号给他传递参数
            print(self.flight_mode)
            return True
        else:
            print(self.flight_mode+"failed")
            return False
    
if __name__ == '__main__':
    communication = Communication()
    communication.start()       
