#include <grasp_generator/detect_server.h>


GraspDetectionServer::GraspDetectionServer(ros::NodeHandle& node)
{
    cloud_camera_ = NULL;

    // Load CFG Parameters, [requires additional parameters via launch file]. 
    std::string cfg_file;
    node.param("config_file", cfg_file, std::string(""));
    grasp_detector_ = new gpd::GraspDetector(cfg_file);
    printf("Created GPD ...\n");

    // Load RVIZ parameters
    std::string rviz_topic;
    node.param("rviz_topic", rviz_topic, std::string("plot_grasps"));

    // Check whether to publish grasps or not to RVIZ
    if (!rviz_topic.empty())
    {
        rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);
        use_rviz_ = true;
    }
    else  {
        use_rviz_ = false;
    }    

    // Advertise ROS topic for detected grasps.
    grasps_pub_ = node.advertise<grasp_generator::GraspConfigList>("clustered_grasps", 10);

    node.getParam("workspace", workspace_);
}

bool GraspDetectionServer::grasp_detection(grasp_generator::DetectGrasps::Request& req, grasp_generator::DetectGrasps::Response& res)
{
    // Retrieve point cloud
    const sensor_msgs::PointCloud2& point_cloud = req.cloud;

    // Process point cloud information
    delete cloud_camera_;      
    cloud_camera_ = NULL;

    Eigen::Matrix3Xd view_points(3,1);
    view_points.col(0) = view_point_;

    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(point_cloud, *cloud);
    cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
    cloud_camera_header_ = point_cloud.header;
    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
    
    frame_ = point_cloud.header.frame_id;

    // Preprocess point cloud + generate grasping positions. 
    grasp_detector_->preprocessPointCloud(*cloud_camera_);
    std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

    // Visualize and publish the detected grasps. 
    if (grasps.size() > 0){
        if (use_rviz_){
            rviz_plotter_->drawGrasps(grasps, frame_);
        }

        // Publish the detected grasps.
        grasp_generator::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
        res.grasp_configs = selected_grasps_msg;
        ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
        return true;
    }
    return false;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "detect_grasping_server");
 
    // Create a variable node handle.
    ros::NodeHandle node("~");
    GraspDetectionServer detect_server(node);

    // A service with name "~/detect_object"
    ros::ServiceServer service = node.advertiseService("detect_object", &GraspDetectionServer::grasp_detection,
                                                     &detect_server);

    // Wait till point cloud has been sent.     
    ROS_INFO("Grasp detection service is waiting for a point cloud ...");

    ros::spin();
    return 0; 
}