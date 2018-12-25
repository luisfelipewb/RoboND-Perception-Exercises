#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling 
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(0.01,0.01,0.01)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0.75,1.1)
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.02)

    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)

    print(white_cloud)
    tree = white_cloud.make_kdtree()
    #print(tree)
    ec = white_cloud.make_EuclideanClusterExtraction()
    #print(ec)
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    #ec.set_MaxClusterSize(250)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    #print(cluster_indices)

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    #print("cluster_color")
    #print(cluster_color)
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            #print("for ",i,j)
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    print (cluster_cloud)
    filename = 'cluster_cloud.pcd'
    pcl.save(cluster_cloud, filename)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_object = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages

    pcl_objects_pub.publish(ros_cloud_object)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud",pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
