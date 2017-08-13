#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *
from sensor_msgs.msg import JointState

import rospy
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

import math


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:
    rospy.loginfo('pcl_callback')
    # Convert ROS msg to PCL data
    pcl_msg = ros_to_pcl(pcl_msg)
    
    # Statistical Outlier Filtering
    outlier_filter = pcl_msg.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(10)
    outlier_filter.set_std_dev_mul_thresh(0.01)
    cloud_filtered = outlier_filter.filter()

    # Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.007
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough Filter
    passthrough_top = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough_top.set_filter_field_name (filter_axis)
    axis_min = 0.82
    axis_max = 1.1
    passthrough_top.set_filter_limits (axis_min, axis_max)
    cloud_filtered_top = passthrough_top.filter()

    passthrough_bottom = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough_bottom.set_filter_field_name (filter_axis)
    axis_min = 0.55
    axis_max = 0.76
    passthrough_bottom.set_filter_limits (axis_min, axis_max)
    cloud_filtered_bottom = passthrough_bottom.filter()

    passthrough_bottom = cloud_filtered_bottom.make_passthrough_filter()
    filter_axis = 'y'
    passthrough_bottom.set_filter_field_name (filter_axis)
    axis_min = -0.8
    axis_max = 0.85
    passthrough_bottom.set_filter_limits (axis_min, axis_max)
    cloud_filtered_bottom = passthrough_bottom.filter()

    # RANSAC Plane Segmentation
    seg = cloud_filtered_top.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    # Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_objects_top = cloud_filtered_top.extract(inliers, negative=True)

    seg = cloud_filtered_bottom.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    # Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_objects_bottom = cloud_filtered_bottom.extract(inliers, negative=True)

    points_list = []

    for data in cloud_objects_bottom:
        points_list.append([data[0], data[1], data[2], data[3]])

    for data in cloud_objects_top:
        points_list.append([data[0], data[1], data[2], data[3]])

    cloud_objects = pcl.PointCloud_PointXYZRGB()
    cloud_objects.from_list(points_list)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(1500)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # # Convert PCL data to ROS messages
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)


    # Publish ROS messages
    pcl_objects_pub.publish(pcl_to_ros(cloud_objects))
    pcl_cluster_pub.publish(pcl_to_ros(cluster_cloud))

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        cloud_objects_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(cloud_objects_cluster)
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

        if not any(x.label == label for x in all_detected_objects):
            all_detected_objects.append(do)


    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    rospy.loginfo('All detected {} '.format(len(all_detected_objects)))

    detected_objects_pub.publish(detected_objects)
    pcl_all_objects_pub.publish(all_detected_objects)

    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for object in all_detected_objects:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    detected_objects_list = dict(zip(labels, centroids))

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(detected_objects_list):

    rospy.loginfo('start separation')

    # Initialize variables
    test_scene_num = String()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    pick_pose_point = Point()
    place_pose_point = Point()
    place_pose = Pose()
    dict_list = []
    pick_list = 'challenge'

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    try:
        mover()
    except rospy.ROSInterruptException:
        pass


    for object in object_list_param:

        group = object['group']
        name = object['name']
        pos = detected_objects_list.get(name)
        
        if pos is not None:

            test_scene_num.data = pick_list
            object_name.data = name
            pick_pose_point.x = np.asscalar(pos[0])
            pick_pose_point.y = np.asscalar(pos[1])
            pick_pose_point.z = np.asscalar(pos[2])
            pick_pose.position = pick_pose_point

            # Create 'place_pose' for the object
            place_pose_point.x = dropbox_pos_dict[group][0]
            place_pose_point.y = dropbox_pos_dict[group][1]
            place_pose_point.z = dropbox_pos_dict[group][2]
            place_pose.position = place_pose_point

            # Assign the arm to be used for pick_place
            arm_name.data = dropbox_arm_dict[group]

            # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            # Populate various ROS messages
            yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
            dict_list.append(yaml_dict)

    # Output your request parameters into output yaml file

    rospy.loginfo('write to output')
    send_to_yaml("output_" + pick_list + ".yaml", dict_list)

def mover():
    global pr2_joint

    rospy.loginfo('move the pr2 left and right')

    rate = rospy.Rate(50)

    if pr2_joint == 0:
        pr2_joint = -math.pi/2
        pr2_joint_pub.publish(pr2_joint)
        rate.sleep()

    print("out:", pr2_joint)

def controller_callback(msg):
    rate = rospy.Rate(1)
    world_joint_state = msg.position[19]
    rate.sleep()

if __name__ == '__main__':

    all_detected_objects = []
    cloud_all_objects = pcl.PointCloud_PointXYZRGB()
    pr2_joint = 0
    dropbox_pos_dict = {}
    dropbox_arm_dict = {}
    object_list_dict = {}
    world_joint_state = 0

    # ROS node initialization
    rospy.init_node('pr2', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    rospy.Subscriber("/joint_states", JointState, controller_callback, queue_size=1)

    # Create Publishers    
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pcl_all_objects_pub = rospy.Publisher("/all_detected_objects", DetectedObjectsArray, queue_size=1)
    pr2_joint_pub = rospy.Publisher('/pr2/world_joint_controller/command', Float64, queue_size= 1)

    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # Parse parameters into individual variables
    for dropbox in dropbox_param:
        dropbox_pos_dict[dropbox['group']] = dropbox['position']
        dropbox_arm_dict[dropbox['group']] = dropbox['name']

    for object in object_list_param:
        group = object['group']
        name = object['name']
        object_list_dict[name] = group

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []


    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
