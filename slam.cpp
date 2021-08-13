/****************************************************************************\
 
        Simple ROS2 node consumming pointcloud2 message

 \****************************************************************************/

#include <memory>
#include <stdio.h>
#include <unistd.h>
#include <chrono>
#include <string>
#include <cmath>
#include <iostream>
#include <random>

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "std_msgs/msg/float32.hpp"

#include <cilantro/utilities/timer.hpp>
#include <cilantro/utilities/point_cloud.hpp>
#include <cilantro/registration/icp_common_instances.hpp>
#include <cilantro/model_estimation/ransac_hyperplane_estimator.hpp>

using namespace std;
using namespace std::chrono_literals;

//#define debugmode_

class slam_node : public rclcpp::Node
{
public:
    slam_node() : Node("slam_node")
    {
        RCLCPP_INFO(this->get_logger(), "%s started", this->get_name());

        count = 0;
        start_RANSAC = 0; 

        distance = 2.0;
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "raw_plc_stream", 1, std::bind(&slam_node::plc2_ros2_to_array, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointCloud2_repeater", 1);

        // data for odm_node
        pub_coor_x = this->create_publisher<std_msgs::msg::Float32>("map_odom/x", 1);
        pub_coor_y = this->create_publisher<std_msgs::msg::Float32>("map_odom/y", 1);
        pub_coor_z = this->create_publisher<std_msgs::msg::Float32>("map_odom/z", 1);
    }

private:
    void plc2_ros2_to_array(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        // ******************************************
        //
        //              CONVERT_MESSAGE
        //
        // ******************************************
        // start global_timer publish_registred_point_cloud
		// ******************************************

        cilantro::Timer publish_registred_point_cloud;
		publish_registred_point_cloud.start();

        int size_cloud = (msg->data.size() / 32) + 1;

        uint32_t pcd_storage_counter = 0;
        float x, y, z;
        unsigned char *pc_x, *pc_y, *pc_z;
        pc_x = (unsigned char *)&x;
        pc_y = (unsigned char *)&y;
        pc_z = (unsigned char *)&z;

        point_cloud.points.resize(Eigen::NoChange, size_cloud);

        for (size_t k = 0u; k < msg->data.size(); k += 32)
        {
            pc_x[0] = msg->data[k];
            pc_x[1] = msg->data[k + 1];
            pc_x[2] = msg->data[k + 2];
            pc_x[3] = msg->data[k + 3];

            pc_y[0] = msg->data[k + 4];
            pc_y[1] = msg->data[k + 5];
            pc_y[2] = msg->data[k + 6];
            pc_y[3] = msg->data[k + 7];

            pc_z[0] = msg->data[k + 8];
            pc_z[1] = msg->data[k + 9];
            pc_z[2] = msg->data[k + 10];
            pc_z[3] = msg->data[k + 11];

            point_cloud.points.col(pcd_storage_counter) = Eigen::Vector3f(x, y, z);
            pcd_storage_counter += 1;
        }
		// ******************************************			

        // ******************************************
        //
        //              DATA_PROCESSING
        //
        // ******************************************

        // ******************************************
        //      create a source and target
        // ******************************************

        cloud = point_cloud;
        //int point_dist;
        point_cloud.gridDownsample(0.01f).removeInvalidData(); 
        if (count == 0)
            {              
            source = point_cloud;        
            count = 1;
            return;
        }
        target = point_cloud;
    // ******************************************
    //
    //              NORMAL_ESTIMATION
    //
    // ******************************************
    //compute normal pipeline
    //initiation of vector normal of target point cloud
    // ******************************************
        target.normals.resize(Eigen::NoChange, 0);
        cilantro::KDTree3f<> tree(target.points);
        target.estimateNormalsKNN(tree, 7);

    // ******************************************
    // ******************************************

    // ******************************************
    //
    //              ICP algorithm
    //
    // ******************************************
        cilantro::Timer timer_icp_registration;
        timer_icp_registration.start();
        cilantro::RigidTransform3f tf_ref = cilantro::RigidTransform3f::Identity();
        source.transform(tf_ref);
        cilantro::SimpleCombinedMetricRigidICP3f icp(target.points, target.normals, source.points);

        // Parameter setting
        icp.setMaxNumberOfOptimizationStepIterations(1).setPointToPointMetricWeight(0.0f).setPointToPlaneMetricWeight(1.0f);
        icp.correspondenceSearchEngine().setMaxDistance(0.1f * 0.1f);
        icp.setConvergenceTolerance(1e-4f).setMaxNumberOfIterations(30);

        // tranformation matrix estimation
        cilantro::RigidTransform3f tf_est = icp.estimate().getTransform();

        timer_icp_registration.stop();
        std::cout <<"   timer_icp_registration " << timer_icp_registration.getElapsedTime() << std::endl;
        // control loop to avoid transofrming the point cloud with matrix containing NAN
        for (size_t iter = 0; iter <= 2; iter++)
        {
            if ((isnan(tf_est.matrix().col(3)(iter)) == 1) || (tf_est.matrix().col(3)(iter) >= 1))
            {
                std::cout <<" debug 0.0!  "<< std::endl;
                return;
            }
        }
        
        source.transform(tf_est);
        cilantro::SimpleCombinedMetricRigidICP3f icp_fine(target.points, target.normals, source.points);
        icp_fine.setMaxNumberOfOptimizationStepIterations(1).setPointToPointMetricWeight(0.0f).setPointToPlaneMetricWeight(1.0f);
        icp_fine.correspondenceSearchEngine().setMaxDistance(0.1f * 0.1f);
        icp_fine.setConvergenceTolerance(1e-3f).setMaxNumberOfIterations(30);
        cilantro::RigidTransform3f tf_est_fine = icp_fine.estimate().getTransform();
        std::cout <<" debug 0.1!  "<< std::endl;
        // tranformation matrix estimation
         for (size_t iter = 0; iter <= 2; iter++)
        {
            if ((isnan(tf_est_fine.matrix().col(3)(iter)) == 1) || (tf_est_fine.matrix().col(3)(iter) >= 1))
            {
                return;
            }
        }
        std::cout << "ESTIMATED transformation: ICP 1   :  " << std::endl << tf_est.matrix() << std::endl;
        std::cout << "ESTIMATED transformation: ICP 2   :  " << std::endl << tf_est_fine.matrix() << std::endl;
        publish_registred_point_cloud.stop();
        std::cout << "\n	publish_registred_point_cloud  :  " << publish_registred_point_cloud.getElapsedTime()<<  " ms\n" << std::endl;
    
   // tf_ref_fine = tf_est;

        auto message = std_msgs::msg::Float32();

        // publish transformation coordinates in ::x
        message.data = tf_est.matrix().col(3)(0) ;
        std::cout << "\n                                      message.data  ::x   :  " << message.data << std::endl;
        pub_coor_x->publish(message);
        // publish transformation coordinates in ::y
        message.data = tf_est.matrix().col(3)(1);
      //  message.data = tf_est_fine.matrix().col(3)(1)+tf_est.matrix().col(3)(1);
        pub_coor_y->publish(message);

        // publish transformation coordinates in ::z
        message.data = tf_est.matrix().col(3)(2);
        pub_coor_z->publish(message);

        // apply the transformation matrix on  point cloud
        target.transform(tf_est_fine);
        // rotate data
        source = target ;

        cloud.transform(tf_est_fine);
        
        cilantro::Timer Ransc_timer;
        Ransc_timer.start();
        
           

        // ******************************************
        //
        //             RANSAC_PLANE_ESTIMATOR
        //
        // *****************************************


        // increment the RANSAC count for condition start, it's something like if( 10 msgs received than --> start) 
        start_RANSAC++;
        // and then we fill it in a std::vector of cilantro::PointCloud3f
        global_cloud.push_back(cloud);  
        //    RANSAC  condition                                               
        if(start_RANSAC <= 10)
            {   
                return;
        }
        //  the  std::vector should keep the same size, so we need to rotate data we can do it with this line below
        global_cloud.vector::erase(global_cloud.cbegin()); 
        // concatenate all the point cloud in a single point cloud of type cilantro::PointCloud3f
        cilantro::PointCloud3f data_cloud;
        for(size_t i = 0; i<global_cloud.size();i++ ){
            data_cloud.append(global_cloud[i]);
        }



        
		cilantro::PointCloud3f planar_cloud;
       // data_cloud.gridDownsample(0.001f);

        // Setting parameters
		
        cilantro::PlaneRANSACEstimator3f<> pe(data_cloud.points);
        pe.setMaxInlierResidual(0.01f)
        .setTargetInlierCount((size_t)(0.1*data_cloud.size()))
        .setMaxNumberOfIterations(250)
        .setReEstimationStep(true);

        // plane estimation
        Eigen::Hyperplane<float,3> plane = pe.estimate().getModel();
        const auto& inliers = pe.getModelInliers();

        std::cout << "	     RANSAC iterations: " << pe.getNumberOfPerformedIterations() << std::endl;
        planar_cloud = cilantro::PointCloud3f(data_cloud, inliers);
        float point_dist;
       // for (size_t t = 0; t < point_cloud.size() ; t++ ){
            //point_dist = std::sqrt(std::pow(point_cloud.points.row(0)(t),2)+std::pow(point_cloud.points.col(1)(t),2));
            //std::cout <<" distance" << point_dist << std::endl;;
           /* if (point_dist <= distance)
                {
                    
                
            } */
        //}
        

        planar_cloud.normals.resize(Eigen::NoChange, 0);
        cilantro::KDTree3f<> tree_plane(planar_cloud.points);
        planar_cloud.estimateNormalsKNN(tree_plane, 7);
        std::cout << "\n	     RANSAC plan:  " << inliers.size() << ", inlier  " << pe.getNumberOfInliers() << std::endl;


        Ransc_timer.stop();
        std::cout << "\n	Ransc_timer :  " << Ransc_timer.getElapsedTime()<<  " ms\n" << std::endl;
        // ******************************************

        // ******************************************
        //
        //              Message construction
        //
        // ******************************************
        size_cloud = planar_cloud.size();

        // Create a PointCloud2
        sensor_msgs::msg::PointCloud2 cloud_msg;

        // Fill some internals of the PoinCloud2 like the header/width/height ...
        cloud_msg.height = 1;
        cloud_msg.width = 4;

        // Set the point fields to xyzrgb and resize the vector with the following command
        // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
        // the number of occurrences of the type in the PointField, the type of the PointField
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        // For convenience and the xyz, rgb, rgba fields, you can also use the following overloaded function.
        // You have to be aware that the following function does add extra padding for backward compatibility though
        // so it is definitely the solution of choice for PointXYZ and PointXYZRGB
        // 2 is for the number of fields to add
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        // You can then reserve / resize as usual
        modifier.resize(size_cloud);
        // Define the iterators. When doing so, you define the Field you would like to iterate upon and
        // the type of you would like returned: it is not necessary the type of the PointField as sometimes
        // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
        // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
        // and RGBA as A,R,G,B)
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
        // Fill the PointCloud2

        uint8_t r = 0, g = 0, b = 0;

        for (int i = 0; i < size_cloud; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
        {
            /// fill the point cloud to publish
            *iter_x = planar_cloud.points.col(i)(0);
            *iter_y = planar_cloud.points.col(i)(1);
            *iter_z = planar_cloud.points.col(i)(2);

            *iter_r = r;
            *iter_g = g;
            *iter_b = b;
        }

        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = now();

        pub_->publish(cloud_msg);

        
    
/*

               
  std::cout << "\n          value of x  : "<< global_cloud.points.col(0)(2)<< std::endl;
        std::cout << "\n          value of y  : "<< global_cloud.points.col(1)(2)<< std::endl; 
        std::cout << "\n          value of z  : "<< global_cloud.points.col(2)(2)<< std::endl;
*/
       /*
        source = point_cloud; 
       // 
        
        
        
        //free the global point cloud
        //global_cloud.clear();
		plan_estimation_timer.stop();
		std::cout << "\n	     plan_estimation_timer " << plan_estimation_timer.getElapsedTime() << "ms\n" << std::endl;
        std::cout << "\n	     RANSAC plan:  " << inliers.size() << ", inlier  " << pe.getNumberOfInliers() << std::endl;


		 
        
        
        auto dist =  std::sqrt(std::pow(planar_cloud.normals.col(0)(0),2)+std::pow(planar_cloud.normals.col(1)(0),2)+std::pow(planar_cloud.normals.col(2)(0),2));
        std::cout << "distance " << dist << std::endl;

        auto mean_nx = planar_cloud.normals.row(0).mean();
        auto mean_ny = planar_cloud.normals.row(1).mean();
        auto mean_nz = planar_cloud.normals.row(2).mean();

        std::cout << "\n	     value of nx "<< mean_nx << std::endl;
        std::cout << "\n	     value of ny "<< mean_ny << std::endl;
        std::cout << "\n	     value of nz "<< mean_nz<< std::endl;

/// *
#ifdef debugmode_
        std::cout << "Iterations performed: " << icp.getNumberOfPerformedIterations() << std::endl;
        std::cout << "Has converged: " << icp.hasConverged() << std::endl;
        //std::cout << "TRUE transformation:" << std::endl
        //          << tf_ref.inverse().matrix() << std::endl;
        std::cout << "ESTIMATED transformation:" << std::endl
                  << tf_est.matrix() << std::endl;
        cilantro::Timer timer_residuals;
        timer_residuals.start();
#endif

        // ******************************************
        //   std::cout << "ESTIMATED transformation:" << std::endl
        //                  << tf_est.matrix() << std::endl;
        auto residuals = icp.getResiduals();

// ******************************************
#ifdef debugmode_
        timer_residuals.stop();
        std::cout << "Residual computation time: " << timer_residuals.getElapsedTime() << "ms" << std::endl;
#endif
     
        // end timer
        std::cout << " point_cloud.points all X's  :  " << point_cloud.points.row(0)(0) std::endl;    
                std::cout << " point_cloud.points all Y's  :  " << point_cloud.points.row(1)(0)<< std::endl;
                std::cout << " point_cloud.points all Z's  :  " << point_cloud.points.row(2)(0)<< std::endl;
        
    

        publish_registred_point_cloud.stop();
        std::cout << "      performed in  " << publish_registred_point_cloud.getElapsedTime() << " ms\n"
             << std::endl; 
			 
*/      
    }
    float distance;
    int count,start_RANSAC;
    bool pcdStorage_publish_ready = false;
    std::vector<cilantro::PointCloud3f>  global_cloud;
    cilantro::PointCloud3f point_cloud, target, source, cloud;
    rclcpp::Clock::SharedPtr clock = this->get_clock();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr pub_timer;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_coor_x;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_coor_y;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_coor_z;
};

int main(int argc, char *argv[])
{
    // start node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<slam_node>());
    rclcpp::shutdown();
}

