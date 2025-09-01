import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

def publish_pointcloud():
    pub = rospy.Publisher('pointcloud_topic', PointCloud, queue_size=10)
    rospy.init_node('pointcloud_publisher', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        cloud = PointCloud()
        cloud.header.stamp = rospy.Time.now()
        cloud.header.frame_id = "map"
        cloud.points.append(Point32(1.0, 2.0, 0.0))
        cloud.points.append(Point32(2.0, 3.0, 0.0))
        cloud.points.append(Point32(3.0, 4.0, 1.0))
        rospy.loginfo("Publishing PointCloud with %d points", len(cloud.points))
        pub.publish(cloud)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pointcloud()
    except rospy.ROSInterruptException:
        pass


