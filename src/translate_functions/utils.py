from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point

def Yaw_to_Quaternion(yaw):
    roll, pitch = 0, 0
    return quaternion_from_euler(roll, pitch, yaw)

def Position_to_Point(position):
    point = Point()
    point.x , point.y, point.z = position.x, position.y, position.z
    return point