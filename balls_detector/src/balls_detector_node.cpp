#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"


using namespace cv;
using namespace std;

// detection parameters (global)
int threshold_step;
int min_threshold;
int max_threshold;
int min_repeatability;
int min_dist_between_blobs;
bool filter_by_color;
int blob_color;
bool filter_by_area;
int min_area;
int max_area;
bool filter_by_circularity;
double min_circularity;
double max_circularity;
bool filter_by_inertia;
double min_inertia_ratio;
double max_inertia_ratio;
bool filter_by_convexity;
double min_convexity;
double max_convexity;


class Receiver
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher locs_pub_;

  const unsigned long hardware_threads;
  cv::Mat color;
  std::vector<std::thread> threads;
  std::thread imageDispThread;
  bool running, updateImage;
  mutable std::string windowName;
  ros::AsyncSpinner spinner;

  std::mutex mutex;
  std::string camera;

public:
  Receiver()
  : it_(nh_), hardware_threads(std::thread::hardware_concurrency()),
   spinner(hardware_threads/2), updateImage(false)
  {

    windowName = "cam_rgb";
    image_sub_ = it_.subscribe("/cam/image/rgb", 1, &Receiver::imageCallback, this);
    locs_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/locs/detected", 1000);

    imageDispThread = std::thread(&Receiver::imageDisp, this);
  }

  ~Receiver()
  {
    cv::destroyAllWindows();
  }

  Receiver(Receiver const&) = delete;
  Receiver& operator=(Receiver const&) = delete;

  void run() {
    begin();
    end();
  }

private:
  void begin()
  {
    if(spinner.canStart())
    {
      spinner.start();
    }
    running = true;
    while(!updateImage)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    //spawn the threads
    if (imageDispThread.joinable())    {
      imageDispThread.join();
    }
  }

  void end()
  {
    spinner.stop();
    running = false;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat color;
    std::lock_guard<std::mutex> lock(mutex);
    getImage(msg, color);
    this->color = color;
    updateImage = true;
  }

  void getImage(const sensor_msgs::ImageConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msgImage, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_ptr->image.copyTo(image);
  }

  void imageDisp()
  {
    cv::Mat color, blob_circs(640, 480, CV_8UC3);
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 640, 480) ;

    for(; running && ros::ok() ;)
    {
      if(updateImage)
      {
        std::lock_guard<std::mutex> lock(mutex);
        color = this->color;
        updateImage = false;

        detectBlobs(std::move(color), std::move(blob_circs));
        cv::imshow(windowName, blob_circs);
      }

      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
        case 27:
          break;
        case 'q':
          running = false;
          break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(5);
  }

  static String Legende(SimpleBlobDetector::Params &pAct)
  {
    String s = "";
    if (pAct.filterByArea)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << pAct.minArea).str();
        String sup = static_cast<const ostringstream&>(ostringstream() << pAct.maxArea).str();
        s = " Area range [" + inf + " to  " + sup + "]";
    }
    if (pAct.filterByCircularity)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << pAct.minCircularity).str();
        String sup = static_cast<const ostringstream&>(ostringstream() << pAct.maxCircularity).str();
        if (s.length() == 0)
            s = " Circularity range [" + inf + " to  " + sup + "]";
        else
            s += " AND Circularity range [" + inf + " to  " + sup + "]";
    }
    if (pAct.filterByColor)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << (int)pAct.blobColor).str();
        if (s.length() == 0)
            s = " Blob color " + inf;
        else
            s += " AND Blob color " + inf;
    }
    if (pAct.filterByConvexity)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << pAct.minConvexity).str();
        String sup = static_cast<const ostringstream&>(ostringstream() << pAct.maxConvexity).str();
        if (s.length() == 0)
            s = " Convexity range[" + inf + " to  " + sup + "]";
        else
            s += " AND  Convexity range[" + inf + " to  " + sup + "]";
    }
    if (pAct.filterByInertia)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << pAct.minInertiaRatio).str();
        String sup = static_cast<const ostringstream&>(ostringstream() << pAct.maxInertiaRatio).str();
        if (s.length() == 0)
            s = " Inertia ratio range [" + inf + " to  " + sup + "]";
        else
            s += " AND  Inertia ratio range [" + inf + " to  " + sup + "]";
    }
    return s;
  }

  void detectBlobs(cv::Mat && img, cv::Mat && result)
  {
    SimpleBlobDetector::Params pDefaultBLOB;
    // This is the default parameters set for SimpleBlobDetector
    pDefaultBLOB.thresholdStep = threshold_step;
    pDefaultBLOB.minThreshold = min_threshold;
    pDefaultBLOB.maxThreshold = max_threshold;
    pDefaultBLOB.minRepeatability = min_repeatability;
    pDefaultBLOB.minDistBetweenBlobs = min_dist_between_blobs;
    pDefaultBLOB.filterByColor = filter_by_color;
    pDefaultBLOB.blobColor = blob_color;
    pDefaultBLOB.filterByArea = filter_by_area;
    pDefaultBLOB.minArea = min_area;
    pDefaultBLOB.maxArea = max_area;
    pDefaultBLOB.filterByCircularity = filter_by_circularity;
    pDefaultBLOB.minCircularity = min_circularity;
    pDefaultBLOB.maxCircularity = max_circularity;
    pDefaultBLOB.filterByInertia = filter_by_inertia;
    pDefaultBLOB.minInertiaRatio = min_inertia_ratio;
    pDefaultBLOB.maxInertiaRatio = max_inertia_ratio;
    pDefaultBLOB.filterByConvexity = filter_by_convexity;
    pDefaultBLOB.minConvexity = min_convexity;
    pDefaultBLOB.maxConvexity =max_convexity;
    // Descriptor array for BLOB
    vector<String> typeDesc;
    // Param array for BLOB
    vector<SimpleBlobDetector::Params> pBLOB;
    vector<SimpleBlobDetector::Params>::iterator itBLOB;
    // Color palette
    vector< Vec3b >  palette;
    for (int i = 0; i<65536; i++)
    {
        palette.push_back(Vec3b((uchar)rand(), (uchar)rand(), (uchar)rand()));
    }

    // Param for the BLOB detector
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);

    itBLOB = pBLOB.begin();
    vector<double> desMethCmp;
    Ptr<Feature2D> b;
    String label;
    // Descriptor loop
    vector<String>::iterator itDesc;
    for (itDesc = typeDesc.begin(); itDesc != typeDesc.end(); ++itDesc)
    {
        vector<KeyPoint> keyImg1;
        if (*itDesc == "BLOB")
        {
            b = SimpleBlobDetector::create(*itBLOB);
            label = Legende(*itBLOB);
            ++itBLOB;
        }
        try
        {
            // We can detect keypoint with detect method
            vector<KeyPoint>  keyImg;
            vector<Rect>  zone;
            vector<vector <Point> >  region;
            Mat     desc; //, result(img.rows, img.cols, CV_8UC3);
            if (b.dynamicCast<SimpleBlobDetector>() != NULL)
            {
                Ptr<SimpleBlobDetector> sbd = b.dynamicCast<SimpleBlobDetector>();
                sbd->detect(img, keyImg, Mat());
                drawKeypoints(img, keyImg, result);
                std_msgs::Float64MultiArray locs;
                locs.data.clear();

                int i = 0;
                for (vector<KeyPoint>::iterator k = keyImg.begin(); k != keyImg.end(); ++k, ++i)
                {
                  cout << i << ": "<< keyImg[i].pt.x<<";"<<keyImg[i].pt.y<<endl;
                  locs.data.push_back(keyImg[i].pt.x);
                  locs.data.push_back(keyImg[i].pt.y);

                  circle(result, k->pt, (int)k->size, Scalar(0, 255, 255), 1);
                  putText(result, "hele", k->pt + Point2f(0, 2 * k->size), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 3);
                }
                // TODO would this ever publish more balls than there is?
                locs_pub_.publish(locs);
            }
        }
        catch (Exception& e)
        {
            cout << "Feature : " << *itDesc << "\n";
            cout << e.msg << endl;
        }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sphero_detector_node");
  ros::NodeHandle nh;

  nh.getParam("/balls_detector_node/Blob_Params/threshold_step", threshold_step);
  nh.getParam("/balls_detector_node/Blob_Params/min_threshold", min_threshold);
  nh.getParam("/balls_detector_node/Blob_Params/max_threshold", max_threshold);
  nh.getParam("/balls_detector_node/Blob_Params/min_repeatability", min_repeatability);
  nh.getParam("/balls_detector_node/Blob_Params/min_dist_between_blobs", min_dist_between_blobs);
  nh.getParam("/balls_detector_node/Blob_Params/filter_by_color", filter_by_color);
  nh.getParam("/balls_detector_node/Blob_Params/blob_color", blob_color);
  nh.getParam("/balls_detector_node/Blob_Params/filter_by_area", filter_by_area);
  nh.getParam("/balls_detector_node/Blob_Params/max_area", max_area);
  nh.getParam("/balls_detector_node/Blob_Params/min_area", min_area);
  nh.getParam("/balls_detector_node/Blob_Params/filter_by_circularity", filter_by_circularity);
  nh.getParam("/balls_detector_node/Blob_Params/min_circularity", min_circularity);
  nh.getParam("/balls_detector_node/Blob_Params/max_circularity", max_circularity);
  nh.getParam("/balls_detector_node/Blob_Params/filter_by_inertia", filter_by_inertia);
  nh.getParam("/balls_detector_node/Blob_Params/min_inertia_ratio", min_inertia_ratio);
  nh.getParam("/balls_detector_node/Blob_Params/max_inertia_ratio", max_inertia_ratio);
  nh.getParam("/balls_detector_node/Blob_Params/filter_by_convexity", filter_by_convexity);
  nh.getParam("/balls_detector_node/Blob_Params/min_convexity", min_convexity);
  nh.getParam("/balls_detector_node/Blob_Params/max_convexity", max_convexity);

  // int num_rob;
  // nh.getParam("/balls_detector_node/Rob_Params/num_rob", num_rob);

  Receiver r;
  r.run();

  if(!ros::ok())
  {
    return 0;
  }

  ros::shutdown();
}
