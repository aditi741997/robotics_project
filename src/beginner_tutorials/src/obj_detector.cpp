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
#include <time.h>
#include <Python.h>
#include <bits/stdc++.h>
#include <sys/times.h>

#include <dynamic_reconfigure/server.h>
#include <beginner_tutorials/BinSizeConfig.h>

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
    double compute_rt_sum = 0.0;
    std::vector<double> compute_rt_arr;
    double tput_sum = 0.0;
    std::vector<double> tput_arr;
    double last_pub_time = 0.0;
    double compute_only_sum = 0.0;
    std::vector<double> compute_only_arr;
    double compute_ts_sum = 0.0;
    std::vector<double> compute_ts_arr;

    double td_latency_sum;
    std::vector<double> td_latency_arr;
    
   double td_clock_lat_sum;
    std::vector<double> td_clock_lat_arr; 

    std::string haar_cascade1 = "/home/ubuntu/catkin_ws/src/rbx/data/haar_detectors/haarcascade_frontalface_alt.xml";
    std::string haar_cascade2 = "/home/ubuntu/catkin_ws/src/rbx/data/haar_detectors/haarcascade_frontalface_alt2.xml";
    cv::CascadeClassifier hcascade1;
    cv::CascadeClassifier hcascade2;

    bool write_imgs_to_file = false;

    // For changing bin size (#cores allowed to use) at RunTime:
    int num_cores, bin_size;
    dynamic_reconfigure::Server<beginner_tutorials::BinSizeConfig> dyn_server;

public:
    ObjDetector(int pql, int sql, int num_msg, bool pub, bool doheavy, int lim, int nc)
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

        img_sub = nh.subscribe(sub_topic, sub_queue_len, &ObjDetector::objDetectCB, this, ros::TransportHints().tcpNoDelay(), true);
	cv::setNumThreads(0);
        std::cout << "Subscribed to images, about to call ros::spin, cv numThreads : " << cv::getNumThreads() << ", " << cv::getNumberOfCPUs() << std::endl;
        // cv::namedWindow(OPENCV_WINDOW);
	if( !hcascade1.load( haar_cascade1 ) ){ ROS_INFO("ERROR Loading haar cascade1"); };
        if( !hcascade2.load( haar_cascade2 ) ){ ROS_INFO("ERROR Loading haar cascade2"); };
	std::cout << "duh0\n";
	/* try
	{
		Py_Initialize();
	}
	catch (const std::exception& e)
	{
		
	} 
	std::cout << "duh1\n";
        pName = PyString_FromString("/home/ubuntu/catkin_ws/Sample");
        pModule = PyImport_Import(pName);
        pDict = PyModule_GetDict(pModule);
        pFunc = PyDict_GetItemString(pDict, "add");
        pArgs = PyTuple_New(2);
        pValue = PyInt_FromLong(2);
        PyTuple_SetItem(pArgs, 0, pValue);
        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        if (pResult == NULL)
            ROS_INFO("NULL result from py call");
        else
        {
            int result = PyInt_AsLong(pResult);
            ROS_INFO("Result : %i", result);            
        }
        Py_Finalize();
	*/

	num_cores = nc;
	bin_size = 1;
	// dyn_server = new dynamic_reconfigure::Server<beginner_tutorials::BinSizeConfig>(nh);
	dynamic_reconfigure::Server<beginner_tutorials::BinSizeConfig>::CallbackType f = boost::bind(&ObjDetector::configCallback, this, _1, _2);
	dyn_server.setCallback(f);
    }

    ~ObjDetector()
    {
	Py_Finalize();
	
        ROS_INFO("IN destructor of ObjDetector!!!");
        print_stats();
    }

    void configCallback(beginner_tutorials::BinSizeConfig &config, uint32_t level)
    {
    	ROS_INFO("RECONFIGURE request : IN config callback in ObjDetector node. Current bin size : %i, New bin size : %i", bin_size, config.bin_size);
	if (config.bin_size < num_cores)
	{
		cv::setNumThreads(config.bin_size);
		ROS_INFO("Updated bin sz param and cv #threads : %i", cv::getNumThreads());
	}
	else
	{
		cv::setNumThreads(-1); // to set it to default
	}
	bin_size = config.bin_size;
    }

    std::string get_string(int x)
    {
        std::stringstream ss;
        ss << x;
        return ss.str();
    }

     std::string get_string_d(double x)
    {
        std::stringstream ss;
        ss << x;
        return ss.str();
    }

    void haar(cv::Mat img, bool run_cascade2)
    {
        std::vector<cv::Rect> faces;
        cv::Mat frame_gray;

        cv::cvtColor( img, frame_gray, CV_BGR2GRAY );
        cv::equalizeHist( frame_gray, frame_gray );

        hcascade1.detectMultiScale( frame_gray, faces, 1.1, 3, 0|cv::CASCADE_DO_CANNY_PRUNING, cv::Size(30, 30) );
        if (run_cascade2)
            hcascade2.detectMultiScale( frame_gray, faces, 1.3, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30) );
    }

    void py_haar(bool run_twice)
    {
	// PyRun_SimpleString("import sys;sys.path.append('/home/ubuntu/catkin_ws');import cv2;cv2.setNumThreads(1);import haar_detector;haar_detector.do_random_haar()");
	// std::string s = "import sys;sys.path.append('/home/ubuntu/catkin_ws');import cv2;cv2.setNumThreads(1);import haar_detector;haar_detector.do_haar_ob_track(" + get_string((total%449) + 1) + ")";
	std::string num_c = "";
	if (bin_size < num_cores)
		num_c = "cv2.setNumThreads(" + get_string(bin_size) + ");";

	std::string s = "import sys;sys.path.append('/home/ubuntu/catkin_ws');import cv2;import haar_detector;" + num_c + "haar_detector.do_haar_vw_img(" + get_string((total%1000)) + ", " + get_string((int)run_twice) + ")"; // cv2.setNumThreads(1);
	PyRun_SimpleString(s.c_str());
    }

   void objDetectCB(const sensor_msgs::Image::ConstPtr& msg)
    {
        double lat = (ros::Time::now() - msg->header.stamp).toSec();
        ros::Time cb_start = ros::Time::now();
        clock_t cb_start_rt = clock();

	struct timespec cb_start_ts, cb_end_ts;
        clock_gettime(CLOCK_MONOTONIC, &cb_start_ts);

	 // compute latency w.r.t. TD publishing node
    std::stringstream ss;
    ss << msg->header.frame_id;
    std::string frame_name;
    double td_ts;
    double td_lat = 0.0;
    double td_ts_clock;
    ss >> frame_name >> td_ts;

	if (lat > 0)
        {
            latency_sum += lat;
            latency_arr.push_back(lat);

	    td_lat = ros::Time::now().toSec() - td_ts;
            td_latency_sum += td_lat;
            td_latency_arr.push_back(td_lat);

/*	    double td_clock_lat = ( cb_start_ts.tv_sec + 1e-9*cb_start_ts.tv_nsec) - td_ts_clock;
		td_clock_lat_sum += td_clock_lat;
		td_clock_lat_arr.push_back(td_clock_lat);
*/  
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
	// haar(cv_ptr->image, false);
        py_haar(false);
	std::vector<std::vector<cv::Point> > v;
        total += 1;
        cv::Rect best_boundingrect;
        double max_cnt_area = 0.0;
	struct timespec cb_only_end;
	clock_gettime(CLOCK_MONOTONIC, &cb_only_end);
	
	cv::Mat frame_hsv;
	if (write_imgs_to_file)
	{
		// save to new file
		
		cv::FileStorage fs("/home/ubuntu/catkin_ws/obj_track_imgs/img_" + get_string(total) + ".xml", cv::FileStorage::WRITE);
		fs << "img" << cv_ptr->image;
		fs.release();	
	}
        cv::cvtColor( cv_ptr->image, frame_hsv, cv::COLOR_BGR2HSV );

        cv::Mat img_threshold;
        cv::inRange(frame_hsv, cv::Scalar(29, 86, 6), cv::Scalar(64, 255, 255), img_threshold);
        cv::erode(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::dilate(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        cv::dilate(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::erode(img_threshold, img_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        cv::findContours(img_threshold, v, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        // grabcontours?
        
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
            // cv::Mat drawing = cv::Mat::zeros(img_threshold.size(), CV_8UC3);
            // cv::rectangle(drawing, best_boundingrect.tl(), best_boundingrect.br(), cv::Scalar(50, 50, 50));
            // cv::imshow(OPENCV_WINDOW, drawing);
            // cv::waitKey(3);
        }
        if (total%400 == 3)
        {
            if (v.size() > 0)
                ROS_INFO("Num contours %d, max cnt area %f, rect x %d, y %d, h %d, w %d", v.size(), max_cnt_area, best_boundingrect.x, best_boundingrect.y, best_boundingrect.height, best_boundingrect.width);
            ROS_INFO("Hit rate : %f, num msgs %d", (float)hit/total, total);
        }

        if (do_heavy)
        {
            // call the primes function
            calcPrimes(true);
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

	    hdr.frame_id += " " + get_string_d(td_ts);
            roi_pub.publish(hdr);
        }
	// PyRun_SimpleString("print('Hello World from Embedded Python!!!')");
        double compute = (ros::Time::now() - cb_start).toSec();
        compute_sum += compute;
        compute_arr.push_back(compute);

	double compute_rt = (double)(clock() - cb_start_rt)/CLOCKS_PER_SEC;
	compute_rt_sum += compute_rt;
	compute_rt_arr.push_back(compute_rt);

	clock_gettime(CLOCK_MONOTONIC,&cb_end_ts);
	double compute_ts =  cb_end_ts.tv_sec + 1e-9*cb_end_ts.tv_nsec - ( cb_start_ts.tv_sec + 1e-9*cb_start_ts.tv_nsec);
    	compute_ts_sum += compute_ts;
    	compute_ts_arr.push_back(compute_ts);

	double cb_only = cb_only_end.tv_sec + 1e-9*cb_only_end.tv_nsec - ( cb_start_ts.tv_sec + 1e-9*cb_start_ts.tv_nsec);
	compute_only_sum += cb_only;
	compute_only_arr.push_back(cb_only);

	if ( (total >= 2) && (last_pub_time > 0.0) )
	{
		double op_del = ros::Time::now().toSec() - last_pub_time;
		tput_sum += op_del;
		tput_arr.push_back(op_del);	
	}
	last_pub_time = ros::Time::now().toSec(); 

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
        double med_comp = compute_arr[(num_com)/2];

	// compute_rt
	int num_crt = compute_rt_arr.size();
	std::sort(compute_rt_arr.begin(), compute_rt_arr.end());
	double avg_crt = compute_rt_sum/num_crt;
	double perc_crt = compute_rt_arr[(95*num_crt)/100];
	double med_crt = compute_rt_arr[num_crt/2];

        ROS_INFO("Mean, median, tail Latency at N2: %f %f %f ", avg_lat, med_lat, perc_lat);
        ROS_INFO("Mean, median, tail Compute time (c2) : %f %f %f ", avg_comp, med_comp, perc_comp);
	ROS_INFO("Mean, median, tail of RT Compute time (c2) : %f %f %f", avg_crt, med_crt, perc_crt);
	print_arr(tput_sum, tput_arr, "N2 Tput");
	print_arr(td_latency_sum, td_latency_arr, "Lat at N2 w.r.t. TD node");
	print_arr(compute_ts_sum, compute_ts_arr, "RT (TS) Compute Time");
    	print_arr(compute_only_sum, compute_only_arr, "TS compute only time!!");
	// print_arr(td_clock_lat_sum, td_clock_lat_arr, "RT (TS) Latency at N2 w.r.t. TD node");
    }


    void print_arr(double s, std::vector<double> a, std::string m)
    {
	int num = a.size();
	double avg = s/num;
	std::sort(a.begin(), a.end());
	double med = a[num/2];
	double tail = a[(95*num)/100];
	ROS_INFO("Mean, median, tail of %s is %f %f %f  ArrSz %i #", m.c_str(), avg, med, tail, num);
    }

    void calcPrimes(bool y)
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
	if (y)
	    ROS_INFO("Found %i primes", primes);
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

  PyObject* pInt;

	Py_Initialize();
	std::cout << "py envt init\n";
	// PyRun_SimpleString("print('Hello World from Embedded Python!!!');import cv2;haar_scaleFactor = 1.3;print('haar:%f, cv2 numThr : %i'%(haar_scaleFactor, cv2.getNumThreads()));cascade='/home/ubuntu/catkin_ws/src/rbx/data/haar_detectors/';cs1 =  cv2.CascadeClassifier(cascade);image=cv2.imread('/home/ubuntu/catkin_ws/src/rbx/data/images_fei/Haar_FaceDetected/47-08.jpg');cs1.detectMultiScale(image, **haar_params);");
	PyRun_SimpleString("print('Hello World from Embedded Python!!!');import cv2;import sys;sys.path.append('/home/ubuntu/catkin_ws');import haar_detector;haar_detector.do_haar(cv2.imread('/home/ubuntu/catkin_ws/src/rbx/data/images_fei/Haar_FaceDetected/47-08.jpg'))");
	/*  std::string ps = "taskset -c " + std::string(argv[8]) + " python /home/ubuntu/catkin_ws/Sample.py";
	system(ps.c_str());	
*/	
    ROS_INFO("Init node %s, pub queue len %d, sub_queue_len %d, num_msgs %d, pub %d, doheavy %d, limit %d", node_name.c_str(), pub_queue_len, sub_queue_len, num_msgs, publish, doheavy, lim);

    ros::init(argc, argv, node_name);
    ObjDetector od(pub_queue_len, sub_queue_len, num_msgs, publish, doheavy, lim, 2);
    ros::spin();

    return 0;
}
