#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sstream>

std::string pub_topic = "";
ros::Publisher roi_pub;

static const std::string OPENCV_WINDOW = "Image window";

class ObjDetector
{
    ros::NodeHandle nh;
    ros::Publisher roi_pub;
    ros::Subscriber img_sub;
    std::string pub_topic = "/roi";
    std::string sub_topic = "/camera1/rgb/image_raw";

    int pub_queue_len, sub_queue_len, num_msgs, limit;
    bool publish, do_heavy;

    std_msgs::Header hdr;

// for stats :
    int hit, total;
    double latency_sum = 0.0;
    std::vector<double> latency_arr;
    double compute_sum = 0.0;
    std::vector<double> compute_arr;

public:
    ObjDetector(int pql, int sql, int num_msg, bool pub, bool doheavy, int lim)
    {
        // ros::NodeHandle nh;
        publish = pub;
        pub_queue_len = pql;
        sub_queue_len = sql;
        num_msgs = num_msg;
        do_heavy = doheavy;
        limit = lim;

        hit = 0;
        total = 0;

        if (publish)
        {
            roi_pub = nh.advertise<std_msgs::Header>(pub_topic, pub_queue_len);

            while (0 == roi_pub.getNumSubscribers())
            {
                ROS_INFO("Waiting for subscribers to connect");
                    ros::Duration(0.1).sleep();
            }
            ros::Duration(1.0).sleep();                    
        }

        img_sub = nh.subscribe(sub_topic, sub_queue_len, &ObjDetector::objDetectCB, this, ros::TransportHints().tcpNoDelay());
        std::cout << "Subscribed to images, about to call ros::spin \n";
        // cv::namedWindow(OPENCV_WINDOW);
    }

    ~ObjDetector()
    {
        ROS_INFO("IN destructor of ObjDetector!!!");
        print_stats();
    }

    std::string get_string(int x)
    {
        std::stringstream ss;
        ss << x;
        return ss.str();
    }

    void objDetectCB(const sensor_msgs::Image::ConstPtr& msg)
    {
        double lat = (ros::Time::now() - msg->header.stamp).toSec();
        ros::Time cb_start = ros::Time::now();
        if (lat > 0)
        {
            latency_sum += lat;
            latency_arr.push_back(lat);
        }
        else
            ROS_INFO("Negative latency! %f, %d, ros time : %f", lat, msg->header.seq, ros::Time::now().toSec());

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        cv::Mat frame_hsv;
        cv::cvtColor( cv_ptr->image, frame_hsv, cv::COLOR_BGR2HSV );

        cv::Mat img_threshold;
        cv::inRange(frame_hsv, cv::Scalar(29, 86, 6), cv::Scalar(64, 255, 255), img_threshold);
        cv::erode(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::dilate(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        cv::dilate(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::erode(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        std::vector<std::vector<cv::Point> > v;
        cv::findContours(img_threshold, v, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        // grabcontours?
        total += 1;
        cv::Rect best_boundingrect;
        double max_cnt_area = 0.0;
        for (int i = 0; i < v.size(); i++)
        {
            // get the bounding rect, check if contourArea is better than current
            if (cv::contourArea(v[i]) > max_cnt_area)
            {
                max_cnt_area = cv::contourArea(v[i]);
                best_boundingrect = cv::boundingRect(v[i]);
            }
        }
        if (v.size() > 0)
        {
            hit += 1;
            cv::Mat drawing = cv::Mat::zeros(img_threshold.size(), CV_8UC3);
            cv::rectangle(drawing, best_boundingrect.tl(), best_boundingrect.br(), cv::Scalar(50, 50, 50));
            // cv::imshow(OPENCV_WINDOW, drawing);
            // cv::waitKey(3);
        }
        if (total%100 == 3)
        {
            if (v.size() > 0)
                ROS_INFO("Num contours %d, max cnt area %f, rect x %d, y %d, h %d, w %d", v.size(), max_cnt_area, best_boundingrect.x, best_boundingrect.y, best_boundingrect.height, best_boundingrect.width);
            ROS_INFO("Hit rate : %f, num msgs %d", (float)hit/total, total);
        }

        if (do_heavy)
        {
            // call the primes function
            calcPrimes();
        }

        if(total%800 == 3)
        {
            ROS_INFO("Num msgs %d, Msg seq : %d", total, msg->header.seq);
            print_stats();
        }

        if (publish)
        {
            // make Header object and publish. encode x,y,h,w in string
            hdr.seq = msg->header.seq;
            hdr.stamp = msg->header.stamp;
            if (v.size() > 0)
                hdr.frame_id = get_string(best_boundingrect.x) + " " + get_string(best_boundingrect.y) + " " + get_string(best_boundingrect.width) + " " + get_string(best_boundingrect.height);
            else
                hdr.frame_id = "0 0 0 0";

            roi_pub.publish(hdr);
        }

        double compute = (ros::Time::now() - cb_start).toSec();
        compute_sum += compute;
        compute_arr.push_back(compute);

        if (total >= ((num_msgs*98)/100))
        {
            // exit stuff
            ROS_INFO("Last img received msg seq %i, #msgs received : %i", msg->header.seq, total);
            print_stats();
            ros::shutdown();
        }
    }

    void print_stats()
    {
        // latency
        int num_lat = latency_arr.size();
        std::sort(latency_arr.begin(), latency_arr.end());
        double avg_lat = latency_sum/num_lat;
        double perc_lat = latency_arr[(95*num_lat)/100];
        double med_lat = latency_arr[num_lat/2];

        // compute
        int num_com = compute_arr.size();
        std::sort(compute_arr.begin(), compute_arr.end());
        double avg_comp = compute_sum/num_com;
        double perc_comp = compute_arr[(95*num_com)/100];
        double med_comp = compute_arr[(95*num_com)/100];

        ROS_INFO("Mean, median, tail Latency at N2: %f %f %f ", avg_lat, med_lat, perc_lat);
        ROS_INFO("Mean, median, tail Compute time (c2) : %f %f %f ", avg_comp, med_comp, perc_comp);
    }

    void calcPrimes()
    {
        int i, num = 1, primes = 0;

        while (num <= limit) { 
            i = 2; 
            while (i <= num) { 
                if(num % i == 0)
                    break;
                i++; 
            }
            if (i == num)
                primes++;
            num++;
        }
    }
};

int main(int argc, char **argv)
{
    // This node will receive image messages from PreProcess node [/camera1/rgb/image_raw]
    // and detects an object in the image and publishes the roi box [/roi]
    int pub_queue_len = atoi(argv[1]);
    int sub_queue_len = atoi(argv[2]);
    int num_msgs = atoi(argv[3]);
    std::string node_name = argv[4];
    bool publish = atoi(argv[5]) == 1;
    bool doheavy = atoi(argv[6]) == 1;
    int lim = atoi(argv[7]);

    ROS_INFO("Init node %s, pub queue len %d, sub_queue_len %d, num_msgs %d, pub %d, doheavy %d, limit %d", node_name, pub_queue_len, sub_queue_len, num_msgs, publish, doheavy, lim);

    ros::init(argc, argv, node_name);
    ObjDetector od(pub_queue_len, sub_queue_len, num_msgs, publish, doheavy, lim);
    ros::spin();

    return 0;
}