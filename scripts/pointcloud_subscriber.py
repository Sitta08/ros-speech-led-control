import rospy
from sensor_msgs.msg import PointCloud

def callback(data):
    rospy.loginfo("Received PointCloud with %d points", len(data.points))
    for i, point in enumerate(data.points):
        rospy.loginfo("Point %d: x=%.2f, y=%.2f, z=%.2f", i, point.x, point.y, point.z)

def listener():
    rospy.init_node('pointcloud_subscriber', anonymous=True)
    rospy.Subscriber('pointcloud_topic', PointCloud, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
