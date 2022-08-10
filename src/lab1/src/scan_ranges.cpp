#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"

class ScanRanges {
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pubClosest, pubFarthest;

    void processScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
        float minData = INFINITY, maxData = -INFINITY;
        for (float data : msg->ranges) {
            if (data >= msg->range_min && data <= msg->range_max && std::isfinite(data)) {
                // Update `minData` and `maxData`
                minData = minData < data ? minData : data;
                maxData = maxData > data ? maxData : data;
            }
        }

        if (minData == INFINITY || maxData == -INFINITY)
            // No data came through
            return;

        std_msgs::Float64 min, max;
        min.data = minData;
        max.data = maxData;

        pubClosest.publish(min);
        pubFarthest.publish(max);
    }

public:
    ScanRanges(ros::NodeHandle nh) : nh(nh) {
        // Here, we have to use `&ScanRanges::processScan` and `this` as arguments
        // because a non-static member function requires a reference to the object
        // it's a part of.
        sub = nh.subscribe("scan", 10, &ScanRanges::processScan, this);

        pubClosest = nh.advertise<std_msgs::Float64>("closest_point", 10);
        pubFarthest = nh.advertise<std_msgs::Float64>("farthest_point", 10);
    }

    void run() {
        ros::spin();
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "scan_ranges", ros::init_options::AnonymousName);
    
    ros::NodeHandle nh;
    ScanRanges(nh).run();
}
