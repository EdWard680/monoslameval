import lcm
import rospy
import rosbag
import math
import sys

from lcmtypes import odometry_t, pose_xyt_t, occupancy_grid_t, mbot_imu_t
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import Imu


def rpy_to_quat(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q

def convert_utime(utime):
    ret = Header()
    ret.stamp = rospy.Time.from_sec(utime / 1E6)
    ret.frame_id = "m_bot"
    
    return ret
    
def convert_pose(p):
    ret = PoseStamped()
    ret.header = convert_utime(p.utime)
    ret.pose.position.x = p.x
    ret.pose.position.y = p.y
    ret.pose.position.z = 0
    ret.pose.orientation.w = math.cos(p.theta/2)
    ret.pose.orientation.x = 0
    ret.pose.orientation.y = 0
    ret.pose.orientation.z = math.sin(p.theta/2)
    
    return ret

def logodds_to_percent(logodds):
    if logodds == 0:
        return -1
    else:
        odds = math.exp(logodds)
        return int(100*odds/(1 + odds))

def convert_map(m, t):
    ret = OccupancyGrid()
    ret.header.stamp = t
    ret.header.frame_id = "m_bot"
    
    ret.info.map_load_time = ret.header.stamp
    ret.info.resolution = m.meters_per_cell
    ret.info.width = m.width
    ret.info.height = m.height
    ret.info.origin.position.x = m.origin_x
    ret.info.origin.position.y = m.origin_y
    ret.info.origin.position.z = 0
    ret.info.origin.orientation.w = 1
    ret.info.origin.orientation.x = 0
    ret.info.origin.orientation.y = 0
    ret.info.origin.orientation.z = 0
    
    ret.data = [logodds_to_percent(lo) for lo in m.cells]
    
    return ret

def convert_imu(imu):
    ret = Imu()
    ret.header = convert_utime(imu.utime)
    ret.orientation = rpy_to_quat(*imu.tb_angles)
    
    ret.orientation_covariance = [0]*9
    
    gyro = imu.gyro
    w = ret.angular_velocity
    w.x, w.y, w.z = gyro
    
    # 5 dps tolerance at 0 with 3 percent linear tolerance
    # 2 percent cross axis sensitivity
    rps = 5/180*math.pi
    cross = 0.02**2
    ret.angular_velocity_covariance = [
        (rps + gyro[0]*.03)**2, cross*(gyro[0]*gyro[1]), cross*(gyro[0]*gyro[2]),
        cross*(gyro[0]*gyro[1]), (rps + gyro[1]*.03)**2, cross*(gyro[1]*gyro[2]),
        cross*(gyro[0]*gyro[2]), cross*(gyro[1]*gyro[2]), (rps + gyro[2]*.03)**2
    ]
    
    accel = imu.accel
    a = ret.linear_acceleration
    a.x, a.y, a.z = accel
    
    # 3 percent sensitivity with 2 percent cross axis sensitivity
    ret.linear_acceleration_covariance = [
        (accel[0]*0.03)**2, cross*(accel[0]*accel[1]), cross*(accel[0]*accel[2]),
        cross*(accel[0]*accel[1]), (accel[1]*0.03)**2, cross*(accel[1]*accel[2]),
        cross*(accel[0]*accel[2]), cross*(accel[1]*accel[2]), (accel[2]*0.03)**2
    ]
    
    return ret
    

def convert(in_filename, out_filename):
    log = lcm.EventLog(in_filename, "r")
    
    seq = 0
    prev_time = None
    with rosbag.Bag(out_filename, 'w') as bag:
        for event in log:
            ros_msg = None
            channel = None
            if event.channel == "ODOMETRY":
                lcm_msg = odometry_t.decode(event.data)
                ros_msg = convert_pose(lcm_msg)
                channel = "odometry"
            elif event.channel == "SLAM_POSE":
                lcm_msg = pose_xyt_t.decode(event.data)
                ros_msg = convert_pose(lcm_msg)
                channel = "slam_pose"
            elif event.channel == "SLAM_MAP":
                lcm_msg = occupancy_grid_t.decode(event.data)
                ros_msg = convert_map(lcm_msg, prev_time)
                channel = "map"
            elif event.channel == "MBOT_IMU":
                lcm_msg = mbot_imu_t.decode(event.data)
                ros_msg = convert_imu(lcm_msg)
                channel = "imu"
            else:
                continue
            
            ros_msg.header.seq = seq
            seq += 1
            prev_time = ros_msg.header.stamp
            bag.write(channel, ros_msg, t=ros_msg.header.stamp)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <lcm log> <bag filename>")
    else:
        convert(sys.argv[1], sys.argv[2])
