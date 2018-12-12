#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <ce30_driver/ce30_driver.h>

using namespace std;
using namespace ce30_driver;

ros::Publisher gPub;
std::string gFrameID = "ce30";
#if 0
void DataReceiveCB(shared_ptr<PointCloud> cloud)
{
    sensor_msgs::PointCloud pointcloud;
    pointcloud.header.stamp = ros::Time::now();
    pointcloud.header.frame_id = gFrameID;
    static int point_num = 320 * 20;
    pointcloud.points.reserve(point_num);
    for (auto& point : cloud->points)
    {
        static geometry_msgs::Point32 ros_point;
        ros_point.x = point.x;
        ros_point.y = point.y;
        ros_point.z = point.z;
        pointcloud.points.push_back(ros_point);
    }
    if (gPub.getNumSubscribers() > 0)
    {
        gPub.publish(pointcloud);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ce30_node");
    ros::NodeHandle nh;
    nh.param<std::string>("frame_id", gFrameID, gFrameID);
    gPub = nh.advertise<sensor_msgs::PointCloud>("ce30_points", 1);
    UDPServer server;
    server.RegisterCallback(DataReceiveCB);
    if (!server.Start())
    {
        return -1;
    }
    while (ros::ok())
    {
        server.SpinOnce();
    }
}
#else
void DataReceiveCB(shared_ptr<PointCloud> cloud)
{
    sensor_msgs::PointCloud pointcloud;
    pointcloud.header.stamp = ros::Time::now();
    pointcloud.header.frame_id = gFrameID;
    static int point_num = 320 * 20;
    pointcloud.points.reserve(point_num);
    for (auto& point : cloud->points)
    {
        static geometry_msgs::Point32 ros_point;
        ros_point.x = point.x;
        ros_point.y = point.y;
        ros_point.z = point.z;
        pointcloud.points.push_back(ros_point);
    }
    if (gPub.getNumSubscribers() > 0)
    {
        gPub.publish(pointcloud);
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ce30_node");
    ros::NodeHandle nh;
    nh.param<std::string>("frame_id", gFrameID, gFrameID);
    gPub = nh.advertise<sensor_msgs::PointCloud>("ce30_points", 1);

    int point_num = 0;
    geometry_msgs::Point32 ros_point;
    sensor_msgs::PointCloud ros_point_cloud;
    int i;
    UDPSocket socket;
    PointCloud point_cloud;
    cluster cluster_mgr;
    vector<int> labels;
    Scan distance_scan;
    Scan gray_scan;

    if (!Connect(socket))
    {
        return -1;
    }

    // get CE30-D version
    VersionRequestPacket version_request;
    if (!SendPacket(version_request, socket))
    {
        return -1;
    }

    VersionResponsePacket version_response;
    if (!GetPacket(version_response, socket))
    {
        return -1;
    }

    cout << "CE30-D version: " << version_response.GetVersionString() << endl;

    // start get distance frame
    StartRequestPacket start_request;
    if (!SendPacket(start_request, socket))
    {
        return -1;
    }


    Packet packet;
    while (true)
    {
        labels.clear();
        point_cloud.points.clear();
        gray_scan.Reset();
        distance_scan.Reset();
        ros_point_cloud.points.clear();
        while (!distance_scan.Ready() && !gray_scan.Ready())
        {
            if (!GetPacket(packet, socket))
            {
                continue;
            }
            // parse packet
            unique_ptr<ParsedPacket> parsed = packet.Parse();
            if (parsed)
            {
                if (parsed->grey_image)
                {
                    gray_scan.AddColumnsFromPacket(*parsed);
                }
                else
                {
                    distance_scan.AddColumnsFromPacket(*parsed);
                }
            }
        }

        if (distance_scan.Ready())
        {
            for (int x = 0; x < distance_scan.Width(); ++x)
            {
                for (int y = 0; y < distance_scan.Height(); ++y)
                {
                    auto channel = distance_scan.at(x, y);
                    Point p = channel.point();
                    if (sqrt(p.x * p.x + p.y * p.y) < 0.1f)
                    {
                        continue;
                    }
                    point_cloud.points.push_back(p);
                }
            }

            ros_point_cloud = sensor_msgs::PointCloud();

            // feed point cloud to cluster
            cluster_mgr.DBSCAN_2steps(CLUSTER_KD_TREE, 0.05, 40, 0.30, 20, point_cloud, labels);

            point_num = labels.size();
            ros_point_cloud.header.stamp = ros::Time::now();
            ros_point_cloud.header.frame_id = gFrameID;
            ros_point_cloud.points.reserve(point_num);
            for (i = 0; i < labels.size(); ++i)
            {
                ros_point.x = point_cloud.points[i].x;
                ros_point.y = point_cloud.points[i].y;
                ros_point.z = point_cloud.points[i].z;

                ros_point_cloud.points.push_back(ros_point);
            }

            if (gPub.getNumSubscribers() > 0)
            {
                cout << "publish one cluster packet" << endl;
                gPub.publish(ros_point_cloud);
            }
        }
    }

}



#endif
