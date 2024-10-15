// #include <iostream>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>

// #include <Eigen/Eigen>
// #include <math.h>
// #include <random>

// //ROS
// #include <ros/ros.h>
// #include <ros/console.h>
// #include <nav_msgs/OccupancyGrid.h> // Include for 2D map

// //Octomap
// #include <octomap_msgs/Octomap.h>
// #include <octomap_msgs/conversions.h>
// #include <octomap/OcTree.h>
// #include <dynamicEDT3D/dynamicEDTOctomap.h>


// using namespace std;

// random_device rd;
// default_random_engine eng(rd());
// uniform_real_distribution<double>  rand_x;
// uniform_real_distribution<double>  rand_y;
// uniform_real_distribution<double>  rand_w;
// uniform_real_distribution<double>  rand_h;

// ros::Publisher map_2d_pub; // Publisher for 2D map

// int obs_num;
// double margin;
// double x_min, y_min, z_min, x_max, y_max, z_max;
// double r_min, r_max, h_min, h_max, resolution;

// double grid_resolution = 0.05; // Resolution for 2D map
// double grid_width, grid_height;

// pcl::PointCloud<pcl::PointXYZ> cloudMap;
// nav_msgs::OccupancyGrid grid_map; // 2D map

// void RandomMapGenerate()
// {
//     double numel_e = 0.00001;
//     pcl::PointXYZ pt_random;

//     rand_x = uniform_real_distribution<double>(x_min, x_max);
//     rand_y = uniform_real_distribution<double>(y_min, y_max);
//     rand_w = uniform_real_distribution<double>(r_min, r_max);
//     rand_h = uniform_real_distribution<double>(h_min, h_max);

//     int obs_iter = 0;  
//     while(obs_iter < 6)
//     {
//         double x, y, w, h;
//         if (obs_iter%2==0)
//             x=-3;
//         else
//             x=3;
//         if (obs_iter%3==0)
//             y=0;
//         else if(obs_iter%3==1)
//             y=2;
//         else y=-2;

//         x = floor(x/resolution) * resolution + resolution / 2.0;
//         y = floor(y/resolution) * resolution + resolution / 2.0;

//         int widNum = ceil(0.6/resolution);
//         int longNum = ceil(3.0/resolution);

//         for(int r = -longNum/2.0; r < longNum/2.0; r ++ ) {
//             for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
//                 h = rand_h(eng);
//                 int heiNum = ceil(h / resolution);
//                 for (int t = 0; t < heiNum; t++) {
//                     pt_random.x = x + (r + 0.5) * resolution + numel_e;
//                     pt_random.y = y + (s + 0.5) * resolution + numel_e;
//                     pt_random.z = (t + 0.5) * resolution + numel_e;
//                     cloudMap.points.push_back(pt_random);
//                 }
//             }
//         }

//         obs_iter++;
//     }

//     // obs_iter = 0;
//     // while(obs_iter < 6)
//     // {
//     //     double x, y, w, hh,h;
//     //     if (obs_iter==0)
//     //     {
//     //         x=-5;
//     //         y=0;
//     //         w=10;
//     //         hh=0.05;
//     //     }
//     //     else if (obs_iter==1)
//     //     {
//     //         x=5;
//     //         y=0;
//     //         w=10;
//     //         hh=0.05;
//     //     }
//     //     else if (obs_iter==2)
//     //     {
//     //         x=-2.5;
//     //         y=5;
//     //         w=0.05;
//     //         hh=5;
//     //     }
//     //     else if (obs_iter==3)
//     //     {
//     //         x=2.5;
//     //         y=5;
//     //         w=0.05;
//     //         hh=5;
//     //     }
//     //     else if (obs_iter==4)
//     //     {
//     //         x=-2.5;
//     //         y=-5;
//     //         w=0.05;
//     //         hh=5;
//     //     }
//     //     else if (obs_iter==5)
//     //     {
//     //         x=2.5;
//     //         y=-5;
//     //         w=0.05;
//     //         hh=5;
//     //     }

//     //     x = floor(x/resolution) * resolution + resolution / 2.0;
//     //     y = floor(y/resolution) * resolution + resolution / 2.0;

//     //     int widNum = ceil(w/resolution);
//     //     int longNum = ceil(hh/resolution);

//     //     for(int r = -longNum/2.0; r < longNum/2.0; r ++ ) {
//     //         for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
//     //             h = rand_h(eng);
//     //             int heiNum = ceil(0.2 / resolution);
//     //             for (int t = 0; t < heiNum; t++) {
//     //                 pt_random.x = x + (r + 0.5) * resolution + numel_e;
//     //                 pt_random.y = y + (s + 0.5) * resolution + numel_e;
//     //                 pt_random.z = (t + 0.5) * resolution + numel_e;
//     //                 cloudMap.points.push_back(pt_random);
//     //             }
//     //         }
//     //     }

//     //     obs_iter++;
//     // }

//     cloudMap.width = cloudMap.points.size();
//     cloudMap.height = 1;
//     cloudMap.is_dense = true;

//     ROS_INFO("Finished generate  map ");
// }

// void generate2DMap()
// {
//     // Initialize the 2D map
//     grid_width = (x_max - x_min) / grid_resolution;
//     grid_height = (y_max - y_min) / grid_resolution;

//     grid_map.info.resolution = grid_resolution;
//     grid_map.info.width = grid_width;
//     grid_map.info.height = grid_height;
//     // grid_map.info.origin.position.x = x_min;
//     // grid_map.info.origin.position.y = y_min;

//     /*NOTE:: The origin is set to (0,0) in this project*/

//     grid_map.info.origin.position.x = 0;
//     grid_map.info.origin.position.y = 0;
//     grid_map.info.origin.position.z = 0;
//     grid_map.header.frame_id = "map";
//     grid_map.data.assign(grid_width * grid_height, 0);

//     // Mark the occupied cells
//     for (auto& pt : cloudMap.points)
//     {
//         int grid_x = (pt.x - x_min) / grid_resolution;
//         int grid_y = (pt.y - y_min) / grid_resolution;

//         if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height)
//         {
//             int index = grid_y * grid_width + grid_x;
//             grid_map.data[index] = 100; // Mark as occupied
//         }
//     }

//     ROS_INFO("Generated 2D occupancy grid map");
//     map_2d_pub.publish(grid_map);
// }

// int main (int argc, char** argv) {
//     ros::init (argc, argv, "map_generation");
//     ros::NodeHandle n( "~" );
//     ros::V_string args;
//     ros::removeROSArgs(argc, argv, args);

//     map_2d_pub    = n.advertise<nav_msgs::OccupancyGrid>("/map", 1); // Advertise 2D map

//     n.param<double>("world/x_min", x_min, -5);
//     n.param<double>("world/y_min", y_min, -5);
//     n.param<double>("world/z_min", z_min, 0);
//     n.param<double>("world/x_max", x_max, 5);
//     n.param<double>("world/y_max", y_max, 5);
//     n.param<double>("world/z_max", z_max, 2.5);
//     n.param<double>("world/margin", margin, 1.5);

//     n.param<int>("world/obs_num", obs_num,  6);
//     n.param<double>("world/resolution",  resolution, 0.1);
//     n.param<double>("world/r_min", r_min,   0.3);
//     n.param<double>("world/r_max", r_max,   0.8);
//     n.param<double>("world/h_min", h_min,   1.0);
//     n.param<double>("world/h_max", h_max,   2.5);


//     // generate map msg
//     RandomMapGenerate();
//     generate2DMap(); // Generate 2D map

//     ros::Rate rate(10);
//     while (ros::ok())
//     {
//         generate2DMap(); // Publish 2D map periodically
//         ros::spinOnce();
//         rate.sleep();
//     }
// }
















// #include <iostream>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>

// #include <Eigen/Eigen>
// #include <math.h>
// #include <random>

// //ROS
// #include <ros/ros.h>
// #include <ros/console.h>
// #include <nav_msgs/OccupancyGrid.h> // Include for 2D map

// //Octomap
// #include <octomap_msgs/Octomap.h>
// #include <octomap_msgs/conversions.h>
// #include <octomap/OcTree.h>
// #include <dynamicEDT3D/dynamicEDTOctomap.h>

// using namespace std;

// random_device rd;
// default_random_engine eng(rd());
// uniform_real_distribution<double> rand_x;
// uniform_real_distribution<double> rand_y;
// uniform_real_distribution<double> rand_area;

// ros::Publisher map_2d_pub; // Publisher for 2D map

// int obs_num; // Number of obstacles
// double x_min, y_min, z_min, x_max, y_max;
// double r_min, r_max, h_min, h_max, resolution;
// double obstacle_height = 1.5; // Fixed height for obstacles

// double grid_resolution = 0.05; // Resolution for 2D map
// double grid_width, grid_height;

// pcl::PointCloud<pcl::PointXYZ> cloudMap;
// nav_msgs::OccupancyGrid grid_map; // 2D map

// void RandomMapGenerate()
// {
//     double numel_e = 0.00001;
//     pcl::PointXYZ pt_random;

//     rand_x = uniform_real_distribution<double>(x_min, x_max);
//     rand_y = uniform_real_distribution<double>(y_min, y_max);
//     rand_area = uniform_real_distribution<double>(r_min, r_max);

//     int obs_iter = 0;
//     while(obs_iter < obs_num)
//     {
//         double x = rand_x(eng);
//         double y = rand_y(eng);
//         double area = rand_area(eng);

//         x = floor(x / resolution) * resolution + resolution / 2.0;
//         y = floor(y / resolution) * resolution + resolution / 2.0;

//         int widNum = ceil(area / resolution);
//         int longNum = ceil(area / resolution);

//         for(int r = -longNum / 2.0; r < longNum / 2.0; r++) {
//             for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
//                 int heiNum = ceil(obstacle_height / resolution);
//                 for (int t = 0; t < heiNum; t++) {
//                     pt_random.x = x + (r + 0.5) * resolution + numel_e;
//                     pt_random.y = y + (s + 0.5) * resolution + numel_e;
//                     pt_random.z = (t + 0.5) * resolution + numel_e;
//                     cloudMap.points.push_back(pt_random);
//                 }
//             }
//         }

//         obs_iter++;
//     }

//     cloudMap.width = cloudMap.points.size();
//     cloudMap.height = 1;
//     cloudMap.is_dense = true;

//     ROS_INFO("Finished generating random map with obstacles");
// }

// void generate2DMap()
// {
//     // Initialize the 2D map
//     grid_width = (x_max - x_min) / grid_resolution;
//     grid_height = (y_max - y_min) / grid_resolution;

//     grid_map.info.resolution = grid_resolution;
//     grid_map.info.width = grid_width;
//     grid_map.info.height = grid_height;

//     /*NOTE:: The origin is set to (0,0) in this project*/

//     grid_map.info.origin.position.x = 0;
//     grid_map.info.origin.position.y = 0;
//     grid_map.info.origin.position.z = 0;
//     grid_map.header.frame_id = "map";
//     grid_map.data.assign(grid_width * grid_height, 0);

//     // Mark the occupied cells
//     for (auto& pt : cloudMap.points)
//     {
//         int grid_x = (pt.x - x_min) / grid_resolution;
//         int grid_y = (pt.y - y_min) / grid_resolution;

//         if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height)
//         {
//             int index = grid_y * grid_width + grid_x;
//             grid_map.data[index] = 100; // Mark as occupied
//         }
//     }

//     ROS_INFO("Generated 2D occupancy grid map");
//     map_2d_pub.publish(grid_map);
// }

// int main (int argc, char** argv) {
//     ros::init (argc, argv, "map_generation");
//     ros::NodeHandle n( "~" );
//     ros::V_string args;
//     ros::removeROSArgs(argc, argv, args);

//     map_2d_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1); // Advertise 2D map

//     n.param<double>("world/x_min", x_min, -5);
//     n.param<double>("world/y_min", y_min, -5);
//     n.param<double>("world/z_min", z_min, 0);
//     n.param<double>("world/x_max", x_max, 5);
//     n.param<double>("world/y_max", y_max, 5);

//     n.param<int>("world/obs_num", obs_num, 10);
//     n.param<double>("world/resolution", resolution, 0.1);
//     n.param<double>("world/r_min", r_min, 0.5);
//     n.param<double>("world/r_max", r_max, 1);
//     n.param<double>("world/h_min", h_min, 1.0);
//     n.param<double>("world/h_max", h_max, 2.5);

//     // generate map msg
//     RandomMapGenerate();
//     generate2DMap(); // Generate 2D map

//     ros::Rate rate(10);
//     while (ros::ok())
//     {
//         generate2DMap(); // Publish 2D map periodically
//         ros::spinOnce();
//         rate.sleep();
//     }
// }




















// #include <iostream>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>

// #include <Eigen/Eigen>
// #include <math.h>
// #include <random>

// //ROS
// #include <ros/ros.h>
// #include <ros/console.h>
// #include <nav_msgs/OccupancyGrid.h> // Include for 2D map

// //Octomap
// #include <octomap_msgs/Octomap.h>
// #include <octomap_msgs/conversions.h>
// #include <octomap/OcTree.h>
// #include <dynamicEDT3D/dynamicEDTOctomap.h>

// using namespace std;

// random_device rd;
// default_random_engine eng(rd());
// uniform_real_distribution<double> rand_x;
// uniform_real_distribution<double> rand_y;
// uniform_real_distribution<double> rand_area;

// ros::Publisher map_2d_pub; // Publisher for 2D map

// int obs_num = 8; // Number of obstacles
// double margin;
// double x_min, y_min, z_min, x_max, y_max, z_max;
// double r_min, r_max, h_min, h_max, resolution;
// double obstacle_height = 1.5; // Fixed height for obstacles

// double grid_resolution = 0.05; // Resolution for 2D map
// double grid_width, grid_height;

// pcl::PointCloud<pcl::PointXYZ> cloudMap;
// nav_msgs::OccupancyGrid grid_map; // 2D map
// octomap::OcTree* octree; // Pointer to OcTree object

// void RandomMapGenerate()
// {
//     double numel_e = 0.00001;
//     pcl::PointXYZ pt_random;

//     rand_x = uniform_real_distribution<double>(x_min, x_max);
//     rand_y = uniform_real_distribution<double>(y_min, y_max);
//     rand_area = uniform_real_distribution<double>(r_min, r_max);

//     int obs_iter = 0;
//     while(obs_iter < obs_num)
//     {
//         double x = rand_x(eng);
//         double y = rand_y(eng);
//         double area = rand_area(eng);

//         x = floor(x / resolution) * resolution + resolution / 2.0;
//         y = floor(y / resolution) * resolution + resolution / 2.0;

//         int widNum = ceil(area / resolution);
//         int longNum = ceil(area / resolution);

//         for(int r = -longNum / 2.0; r < longNum / 2.0; r++) {
//             for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
//                 int heiNum = ceil(obstacle_height / resolution);
//                 for (int t = 0; t < heiNum; t++) {
//                     pt_random.x = x + (r + 0.5) * resolution + numel_e;
//                     pt_random.y = y + (s + 0.5) * resolution + numel_e;
//                     pt_random.z = (t + 0.5) * resolution + numel_e;
//                     cloudMap.points.push_back(pt_random);
//                 }
//             }
//         }

//         obs_iter++;
//     }

//     cloudMap.width = cloudMap.points.size();
//     cloudMap.height = 1;
//     cloudMap.is_dense = true;

//     ROS_INFO("Finished generating random map with obstacles");
// }

// void generate2DMap()
// {
//     // Initialize the 2D map
//     grid_width = (x_max - x_min) / grid_resolution;
//     grid_height = (y_max - y_min) / grid_resolution;

//     grid_map.info.resolution = grid_resolution;
//     grid_map.info.width = grid_width;
//     grid_map.info.height = grid_height;

//     /*NOTE:: The origin is set to (0,0) in this project*/

//     grid_map.info.origin.position.x = 0;
//     grid_map.info.origin.position.y = 0;
//     grid_map.info.origin.position.z = 0;
//     grid_map.header.frame_id = "map";
//     grid_map.data.assign(grid_width * grid_height, 0);

//     // Mark the occupied cells
//     for (auto& pt : cloudMap.points)
//     {
//         int grid_x = (pt.x - x_min) / grid_resolution;
//         int grid_y = (pt.y - y_min) / grid_resolution;

//         if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height)
//         {
//             int index = grid_y * grid_width + grid_x;
//             grid_map.data[index] = 100; // Mark as occupied
//         }
//     }

//     ROS_INFO("Generated 2D occupancy grid map");
//     map_2d_pub.publish(grid_map);
// }

// void saveMapAsBT()
// {
//     // Initialize the octree
//     double octree_resolution = 0.1;
//     octree = new octomap::OcTree(octree_resolution);

//     // Add points from the point cloud to the octree
//     for (auto& pt : cloudMap.points)
//     {
//         octree->updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
//     }

//     // Update the occupancy
//     octree->updateInnerOccupancy();

//     // Save the octree to a .bt file
//     octree->writeBinary("/home/zt/catkin_ws/src/multi_robot_traj_planner/multi_robot_traj_planner/mapfile/generated_map.bt");
//     ROS_INFO("Saved map as .bt file");
// }

// int main (int argc, char** argv) {
//     ros::init (argc, argv, "map_generation");
//     ros::NodeHandle n( "~" );
//     ros::V_string args;
//     ros::removeROSArgs(argc, argv, args);

//     map_2d_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1); // Advertise 2D map

//     n.param<double>("world/x_min", x_min, -5);
//     n.param<double>("world/y_min", y_min, -5);
//     n.param<double>("world/z_min", z_min, 0);
//     n.param<double>("world/x_max", x_max, 5);
//     n.param<double>("world/y_max", y_max, 5);
//     n.param<double>("world/z_max", z_max, 2.5);
//     n.param<double>("world/margin", margin, 1.5);

//     n.param<double>("world/resolution", resolution, 0.1);
//     n.param<double>("world/r_min", r_min, 0.3);
//     n.param<double>("world/r_max", r_max, 0.8);
//     n.param<double>("world/h_min", h_min, 1.0);
//     n.param<double>("world/h_max", h_max, 2.5);

//     // generate map msg
//     RandomMapGenerate();
//     generate2DMap(); // Generate 2D map
//     saveMapAsBT(); // Save map as .bt file

//     ros::Rate rate(10);
//     while (ros::ok())
//     {
//         generate2DMap(); // Publish 2D map periodically
//         ros::spinOnce();
//         rate.sleep();
//     }

//     delete octree; // Clean up the octree
// }



#include <iostream>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Eigen>
#include <math.h>
#include <random>

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h> // Include for 2D map

//Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

using namespace std;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_area;

ros::Publisher map_2d_pub; // Publisher for 2D map

int obs_num; // Number of obstacles
double margin;
double x_min, y_min, z_min, x_max, y_max, z_max;
double r_min, r_max, h_min, h_max, resolution;
double obstacle_height = 1.5; // Fixed height for obstacles

double grid_resolution = 0.05; // Resolution for 2D map
double grid_width, grid_height;

pcl::PointCloud<pcl::PointXYZ> cloudMap;
nav_msgs::OccupancyGrid grid_map; // 2D map
octomap::OcTree* octree; // Pointer to OcTree object

vector<pair<double, double>> obstacle_centers; // Store obstacle centers

void RandomMapGenerate()
{
    double numel_e = 0.00001;
    pcl::PointXYZ pt_random;

    rand_x = uniform_real_distribution<double>(x_min, x_max);
    rand_y = uniform_real_distribution<double>(y_min, y_max);
    rand_area = uniform_real_distribution<double>(r_min, r_max);

    int obs_iter = 0;
    while(obs_iter < obs_num)
    {
        double x = rand_x(eng);
        double y = rand_y(eng);
        double area = rand_area(eng);

        x = floor(x / resolution) * resolution + resolution / 2.0;
        y = floor(y / resolution) * resolution + resolution / 2.0;

        // Store the center of the obstacle
        obstacle_centers.push_back(make_pair(x, y));

        int widNum = ceil(area / resolution);
        int longNum = ceil(area / resolution);

        for(int r = -longNum / 2.0; r < longNum / 2.0; r++) {
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                int heiNum = ceil(obstacle_height / resolution);
                for (int t = 0; t < heiNum; t++) {
                    pt_random.x = x + (r + 0.5) * resolution + numel_e;
                    pt_random.y = y + (s + 0.5) * resolution + numel_e;
                    pt_random.z = (t + 0.5) * resolution + numel_e;
                    cloudMap.points.push_back(pt_random);
                }
            }
        }

        obs_iter++;
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    ROS_INFO("Finished generating random map with obstacles");
}

void generate2DMap()
{
    // Initialize the 2D map
    grid_width = (x_max - x_min) / grid_resolution;
    grid_height = (y_max - y_min) / grid_resolution;

    grid_map.info.resolution = grid_resolution;
    grid_map.info.width = grid_width;
    grid_map.info.height = grid_height;

    /*NOTE:: The origin is set to (0,0) in this project*/

    grid_map.info.origin.position.x = 0;
    grid_map.info.origin.position.y = 0;
    grid_map.info.origin.position.z = 0;
    grid_map.header.frame_id = "map";
    grid_map.data.assign(grid_width * grid_height, 0);

    // Mark the occupied cells
    for (auto& pt : cloudMap.points)
    {
        int grid_x = (pt.x - x_min) / grid_resolution;
        int grid_y = (pt.y - y_min) / grid_resolution;

        if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height)
        {
            int index = grid_y * grid_width + grid_x;
            grid_map.data[index] = 100; // Mark as occupied
        }
    }

    ROS_INFO("Generated 2D occupancy grid map");
    map_2d_pub.publish(grid_map);
}

void saveMapAsBT()
{
    // Initialize the octree
    double octree_resolution = 0.1;
    octree = new octomap::OcTree(octree_resolution);

    // Add points from the point cloud to the octree
    for (auto& pt : cloudMap.points)
    {
        octree->updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
    }

    // Update the occupancy
    octree->updateInnerOccupancy();

    // Save the octree to a .bt file
    octree->writeBinary("/home/zt/catkin_ws/src/multi_robot_traj_planner/multi_robot_traj_planner/mapfile/generated_map.bt");
    ROS_INFO("Saved map as .bt file");
}

void saveObstacleCentersToTxt()
{
    ofstream outfile("/home/zt/multi-robot-planner/src/multi-robot-planner/mapfile/obstacle_centers.txt");
    if (outfile.is_open())
    {
        for (const auto& center : obstacle_centers)
        {
            outfile  << center.first + x_max    << " " << center.second + y_max << endl;
        }
        outfile.close();
        ROS_INFO("Saved obstacle centers to obstacle_centers.txt");
    }
    else
    {
        ROS_ERROR("Unable to open file to save obstacle centers");
    }
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "map_generation");
    ros::NodeHandle n( "~" );
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    map_2d_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1); // Advertise 2D map

    n.param<double>("world/x_min", x_min, -5);
    n.param<double>("world/y_min", y_min, -5);
    n.param<double>("world/z_min", z_min, 0);
    n.param<double>("world/x_max", x_max, 5);
    n.param<double>("world/y_max", y_max, 5);
    n.param<double>("world/z_max", z_max, 2.5);
    n.param<double>("world/margin", margin, 1.5);

    n.param<double>("world/resolution", resolution, 0.1);
    n.param<double>("world/r_min", r_min, 0.3);
    n.param<double>("world/r_max", r_max, 0.8);
    n.param<double>("world/h_min", h_min, 1.0);
    n.param<double>("world/h_max", h_max, 2.5);
    n.param<int>("world/obs_num", obs_num, 8);

    // generate map msg
    RandomMapGenerate();
    generate2DMap(); // Generate 2D map
    saveMapAsBT(); // Save map as .bt file
    saveObstacleCentersToTxt(); // Save obstacle centers to .txt file

    ros::Rate rate(10);
    while (ros::ok())
    {
        generate2DMap(); // Publish 2D map periodically
        ros::spinOnce();
        rate.sleep();
    }

    delete octree; // Clean up the octree
}