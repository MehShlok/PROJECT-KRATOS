import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

rospy.init_node('tf2_broadcaster_and_visualizer')

# Create a TransformBroadcaster object
tf_broadcaster = tf2_ros.TransformBroadcaster()

# Create a Marker publisher for RViz visualization
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

while not rospy.is_shutdown():
    # Create a TransformStamped message
    transform_stamped = TransformStamped()

    # Fill in the header
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = 'base_link'  # Base frame (e.g., robot base or world frame)
    transform_stamped.child_frame_id = 'mesh_frame'  # Mesh frame (frame of your mesh)

    # Set the transformation (modify these values accordingly)
    transform_stamped.transform.translation.x = 1.0  # X translation
    transform_stamped.transform.translation.y = 2.0  # Y translation
    transform_stamped.transform.translation.z = 0.0  # Z translation
    transform_stamped.transform.rotation.x = 0.0  # X rotation
    transform_stamped.transform.rotation.y = 0.0  # Y rotation
    transform_stamped.transform.rotation.z = 0.0  # Z rotation
    transform_stamped.transform.rotation.w = 1.0  # W rotation

    # Publish the TF transformation
    tf_broadcaster.sendTransform(transform_stamped)

    # Create a Marker message
    marker_msg = Marker()

    # Fill in the Marker message
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.header.frame_id = 'mesh_frame'  # Frame in which the marker is defined
    marker_msg.type = Marker.MESH_RESOURCE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position.x = 0  # X position in mesh frame
    marker_msg.pose.position.y = 0  # Y position in mesh frame
    marker_msg.pose.position.z = 0  # Z position in mesh frame
    marker_msg.pose.orientation.x = 0  # X orientation
    marker_msg.pose.orientation.y = 0  # Y orientation
    marker_msg.pose.orientation.z = 0  # Z orientation
    marker_msg.pose.orientation.w = 1  # W orientation
    marker_msg.scale.x = 1  # X scale factor
    marker_msg.scale.y = 1  # Y scale factor
    marker_msg.scale.z = 1  # Z scale factor
    marker_msg.color.a = 1.0  # Alpha
    marker_msg.color.r = 1.0  # Red
    marker_msg.color.g = 1.0  # Green
    marker_msg.color.b = 1.0  # Blue
    marker_msg.mesh_resource = "package://mesh.obj"  # Path to your mesh file
    marker_msg.mesh_use_embedded_materials = True  # Use embedded materials

    # Publish the Marker message
    marker_pub.publish(marker_msg)

    # Sleep or use a rate to control the publishing frequency
    rospy.sleep(1.0)  # Publish once per second (adjust as needed)
