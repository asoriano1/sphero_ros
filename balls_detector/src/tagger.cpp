#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <std_msgs/ColorRGBA.h>
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
bool done = false;


class Receiver
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher tags_pub_;

  cv::Mat frame;
  mutable std::string windowName;

  int numRob;
  std_msgs::Float64MultiArray locs_tagged;

public:
  Receiver()
  : it_(nh_)
  {
    nh_.getParam("/tagger/Rob_Params/num_rob", numRob);
    windowName = "cam_rgb_tagged";
    image_sub_ = it_.subscribe("/cam/image/rgb", 1, &Receiver::imageCallback, this);
    tags_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/locs/tagged", 10);
    locs_tagged.data.clear();
  }

  ~Receiver()
  {
    cv::destroyAllWindows();
  }

  void tag()
  {
	cout << "Will assign tags" << endl;
    assign_tags();
	cout << "Tag assignment complete" << endl;
  }

  void publish_data()
  {
    for(int i = 0; i<=1000 ; i++)
    {
      cout << "publishing" << endl;
      for(int li = 0; li < locs_tagged.data.size(); ++li) {
        cout << locs_tagged.data[li] << endl;
      }

      tags_pub_.publish(locs_tagged);
      ros::Duration(0.01).sleep();
    }
  }


private:

  void assign_tags()
  {
    float camDelay = 5;
    float btDelay = 5;

    // grab a frame and display it
    ros::spinOnce();
    imageDisp();

    //// Defining LED colors
    std_msgs::ColorRGBA led_off, led_on;
    led_off.r = 0; led_off.g = 0; led_off.b = 0; led_off.a = 0; // robot LEDs off
    led_on.r = 1; led_on.g = 1; led_on.b = 1; led_on.a = 0; // robots LEDs ON

    // Initializing advertising topics for spheros' LED color
    std::string led_color_topic [numRob];
    ros::Publisher color_pub_array [numRob];
    for (int num_rob = 0; num_rob < numRob; ++num_rob)
    {
      std::string rob_id = "sph" + std::to_string(num_rob);
      std::string rob_name;
      nh_.getParam("/tagger/Rob_Params/"+rob_id, rob_name);
      std::string topic_name = "/" + rob_name + "/set_color";
      led_color_topic [num_rob] = topic_name;
      color_pub_array [num_rob] = nh_.advertise<std_msgs::ColorRGBA>(led_color_topic[num_rob], 1);
    }

    //// Turn off LEDs
    for (int num_rob = 0; num_rob < numRob; ++num_rob)
    {
      ros::Duration(btDelay).sleep();
      color_pub_array[num_rob].publish(led_off);
      ros::Duration(btDelay).sleep();
      cout << "Turned off " << num_rob << endl;
    }
    cout << "Turned all off" << endl;

    ros::Duration(camDelay).sleep();

    // grab a frame and display it
    ros::spinOnce();
    imageDisp();

    vector<KeyPoint> stationaryKeyPoints;

    getBlobLocations(std::move(frame), stationaryKeyPoints);

    cout << "Num stationary: " << stationaryKeyPoints.size() << endl;

    for(int i = 0; i < stationaryKeyPoints.size(); ++i) {
      cout << " " << stationaryKeyPoints[i].pt.x << "," << stationaryKeyPoints[i].pt.y << endl;
    }

    ros::Duration(camDelay).sleep();

    // Loop through the spheros: turn on one robot, detect it, turn it off, repeat
    for (int num_rob = 0; num_rob < numRob; ++num_rob)
    {
      // Turn on one robot
      ros::Duration(btDelay).sleep();
      color_pub_array[num_rob].publish(led_on);
      cout << "Turned on " << num_rob << endl;
      ros::Duration(btDelay).sleep();

      ros::Duration(camDelay).sleep();

      // grab a frame and display it
      ros::spinOnce();
      imageDisp();
      cout << "Displayed image with " << num_rob << " on" << endl;

      // Detect robot/blob in frame
      detectSingleBlob(std::move(frame),std::move(frame), stationaryKeyPoints);
      imageDisp();

      cout << "Detected Blobs" << endl;
      cout << "Num detected is currently " << locs_tagged.data.size() << " at " << num_rob << endl;

      // Turn that robot off
      ros::Duration(btDelay).sleep();
      color_pub_array[num_rob].publish(led_off);
      cout << "Turned off " << num_rob << endl;
      ros::Duration(btDelay).sleep();
    }

    //// Turn on LEDs
    for (int num_rob = 0; num_rob < numRob; ++num_rob)
    {
      ros::Duration(btDelay).sleep();
      color_pub_array[num_rob].publish(led_on);
      cout << "Turned " << num_rob << " back on" << endl;
      ros::Duration(btDelay).sleep();
    }

    done = true;
  }


  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr frame_ptr;
    try
    {
      frame_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    frame_ptr->image.copyTo(this->frame);
  }

  void imageDisp()
  {
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 640, 480);
    if (!frame.empty())
    {
      cv::imshow(windowName, frame); 
      int key = cv::waitKey(1);
    }
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
    pDefaultBLOB.maxConvexity = max_convexity;
    // Descriptor array for BLOB
    vector<String> typeDesc;
    // Param array for BLOB
    vector<SimpleBlobDetector::Params> pBLOB;
    vector<SimpleBlobDetector::Params>::iterator itBLOB;

    // Param for the BLOB detector
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);

    // Detect that robot
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
        cv::Mat desc; //, result(img.rows, img.cols, CV_8UC3);
        if (b.dynamicCast<SimpleBlobDetector>() != NULL)
        {
          Ptr<SimpleBlobDetector> sbd = b.dynamicCast<SimpleBlobDetector>();
          sbd->detect(img, keyImg, Mat());
          drawKeypoints(img, keyImg, result);

          int i = 0;
          float x = 0;
          float y = 0;
          for (vector<KeyPoint>::iterator k = keyImg.begin(); k != keyImg.end(); ++k, ++i)
          {
            x += keyImg[i].pt.x;
            y += keyImg[i].pt.y;

            circle(result, k->pt, (int)k->size, Scalar(0, 255, 255), 1);

          }
          // avrg the centers
          x = x/(i);
          y = y/(i);
          // store center
          this->locs_tagged.data.push_back(x);
          this->locs_tagged.data.push_back(y);
        }
      }
      catch (Exception& e)
      {
        cout << "Feature : " << *itDesc << "\n";
        cout << e.msg << endl;
      }
    }
  }

  KeyPoint getLargest(const vector<KeyPoint> &remaining) {
    int largestIndex = 0;

    for (int i = 1; i < remaining.size(); ++i) {
      if (remaining[i].size > remaining[largestIndex].size) {
        largestIndex = i;
      }
    }

    return remaining[largestIndex];
  }

  void getNonIntersecting(vector<KeyPoint> &remaining, const vector<KeyPoint> &a, const vector<KeyPoint> &b) {
    // get a-b

    for (int i = 0; i < a.size(); ++i) {
      KeyPoint pa = a[i];
      bool isIntersectingWithAny = false;
      for (int j = 0; j < b.size(); ++j) {
        KeyPoint pb = b[j];

        bool isIntersecting = norm(pa.pt - pb.pt) < (pa.size + pb.size) / 2;

        if (isIntersecting) {
          isIntersectingWithAny = true;
          break;
        }
      }

      if (!isIntersectingWithAny) {
        remaining.push_back(pa);
      }
    }
  }

  // go over each b. remove from a the one that's closest to it. remaining in a are our guys.
  void getRemaining(vector<KeyPoint> &remaining, const vector<KeyPoint> &a, const vector<KeyPoint> &b) {
    vector<KeyPoint> acp = a;

    for (int i = 0; i < b.size(); ++i) {
      Point2f bp = b[i].pt;

      int closestIndex = 0;
      float closestDistance = 0;
      for (int j = 0; j < acp.size(); ++j) {
        Point2f ap = acp[j].pt;

        float distance = norm(ap - bp);

        if (j == closestIndex) { // the first one
          closestDistance = distance;
        } else {
          if (distance < closestDistance) {
            closestIndex = j;
            closestDistance = distance;
          }
        }
      }

      acp.erase(acp.begin() + closestIndex);
    }

    remaining = acp;
  }




  void detectSingleBlob(cv::Mat && img, cv::Mat && result, const vector<KeyPoint> &stationaryKeyPoints)
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
    pDefaultBLOB.maxConvexity = max_convexity;
    // Descriptor array for BLOB
    vector<String> typeDesc;
    // Param array for BLOB
    vector<SimpleBlobDetector::Params> pBLOB;
    vector<SimpleBlobDetector::Params>::iterator itBLOB;

    // Param for the BLOB detector
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);

    // Detect that robot
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
        cv::Mat desc; //, result(img.rows, img.cols, CV_8UC3);
        if (b.dynamicCast<SimpleBlobDetector>() != NULL)
        {
          Ptr<SimpleBlobDetector> sbd = b.dynamicCast<SimpleBlobDetector>();
          sbd->detect(img, keyImg, Mat());
          drawKeypoints(img, keyImg, result);

          // stationaryKeyPoints have got to be there?
          // well, I'm trying to get only one ball here.
          // ignore stationary ones and use the strongest blob
          // if multiple, find closest to stationary ones and ignore them
          // then use the largest among the remaining ones
          //
          // stationary

          int stationaryCount = stationaryKeyPoints.size();
          int foundCount = keyImg.size();

          cout << "foundCount: " << foundCount << " stat: " << stationaryCount << endl;

          cout << "found: " << endl;

          for(int i = 0; i < keyImg.size(); ++i) {
            cout << " " << keyImg[i].pt.x << "," << keyImg[i].pt.y << endl;
          }

          cout << "stationary: " << endl;

          for(int i = 0; i < stationaryKeyPoints.size(); ++i) {
            cout << " " << stationaryKeyPoints[i].pt.x << "," << stationaryKeyPoints[i].pt.y << endl;
          }


          KeyPoint found;

          if (stationaryCount + 1 == foundCount) {
            // ideal case
            // (first remove intersects)
            // go over each stationary, remove the closest found to it, remaining is our guy

            vector<KeyPoint> remaining;
            getRemaining(remaining, keyImg, stationaryKeyPoints); // remaining = key - st

            if (remaining.size() != 1) {
              cerr << "EXPECTED SINGLE REMAINING SINCE stationaryCount + 1 == foundCount. got " << remaining.size() << endl;
            }

            found = remaining[0];
          } else if (stationaryCount + 1 < foundCount) {
            // weird case, extras are there
            // go over each stationary, remove the closest found to it, remaining ones have our guy
            // get the largest one

            vector<KeyPoint> remaining;
            getRemaining(remaining, keyImg, stationaryKeyPoints); // remaining = key - st

            if (remaining.size() <= 1) {
              cerr << "EXPECTED 2+ REMAININGS SINCE stationaryCount + 1 < foundCount. got " << remaining.size() << endl;
            }

            found = getLargest(remaining);
          } else {
            // weird case. stationary ones may be blocking our guy.
            // get the largest one that does not intersect with a stationary.
            // if all do, then fail.

            vector<KeyPoint> remaining;
            getNonIntersecting(remaining, keyImg, stationaryKeyPoints);

            if (remaining.size() == 0) {
              cerr << "EXPECTED A REMAINING IN ELSE" << endl;
            }

            found = getLargest(remaining);
          }

          circle(result, found.pt, (int)found.size, Scalar(0, 255, 255), 1);

          cout << "Adding found point " << found.pt.x << " " << found.pt.y << endl;

          this->locs_tagged.data.push_back(found.pt.x);
          this->locs_tagged.data.push_back(found.pt.y);
//          int i = 0;
//          float x = 0;
//          float y = 0;
//          for (vector<KeyPoint>::iterator k = keyImg.begin(); k != keyImg.end(); ++k, ++i)
//          {
//            x += keyImg[i].pt.x;
//            y += keyImg[i].pt.y;
//
//            circle(result, k->pt, (int)k->size, Scalar(0, 255, 255), 1);
//
//          }
//          // avrg the centers
//          x = x/(i);
//          y = y/(i);
//          // store center
//          this->locs_tagged.data.push_back(x);
//          this->locs_tagged.data.push_back(y);
        }
      }
      catch (Exception& e)
      {
        cout << "Feature : " << *itDesc << "\n";
        cout << e.msg << endl;
      }
    }
  }

  void getBlobLocations(cv::Mat && img, vector<KeyPoint> &keyPoints)
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
    pDefaultBLOB.maxConvexity = max_convexity;
    // Descriptor array for BLOB
    vector<String> typeDesc;
    // Param array for BLOB
    vector<SimpleBlobDetector::Params> pBLOB;
    vector<SimpleBlobDetector::Params>::iterator itBLOB;

    // Param for the BLOB detector
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);

    // Detect that robot
    itBLOB = pBLOB.begin();
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
        if (b.dynamicCast<SimpleBlobDetector>() != NULL)
        {
          Ptr<SimpleBlobDetector> sbd = b.dynamicCast<SimpleBlobDetector>();
          sbd->detect(img, keyPoints, Mat());
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

    cout << "tagger starting" << endl;

    nh.getParam("/tagger/Blob_Params/threshold_step", threshold_step);
    nh.getParam("/tagger/Blob_Params/min_threshold", min_threshold);
    nh.getParam("/tagger/Blob_Params/max_threshold", max_threshold);
    nh.getParam("/tagger/Blob_Params/min_repeatability", min_repeatability);
    nh.getParam("/tagger/Blob_Params/min_dist_between_blobs", min_dist_between_blobs);
    nh.getParam("/tagger/Blob_Params/filter_by_color", filter_by_color);
    nh.getParam("/tagger/Blob_Params/blob_color", blob_color);
    nh.getParam("/tagger/Blob_Params/filter_by_area", filter_by_area);
    nh.getParam("/tagger/Blob_Params/max_area", max_area);
    nh.getParam("/tagger/Blob_Params/min_area", min_area);
    nh.getParam("/tagger/Blob_Params/filter_by_circularity", filter_by_circularity);
    nh.getParam("/tagger/Blob_Params/min_circularity", min_circularity);
    nh.getParam("/tagger/Blob_Params/max_circularity", max_circularity);
    nh.getParam("/tagger/Blob_Params/filter_by_inertia", filter_by_inertia);
    nh.getParam("/tagger/Blob_Params/min_inertia_ratio", min_inertia_ratio);
    nh.getParam("/tagger/Blob_Params/max_inertia_ratio", max_inertia_ratio);
    nh.getParam("/tagger/Blob_Params/filter_by_convexity", filter_by_convexity);
    nh.getParam("/tagger/Blob_Params/min_convexity", min_convexity);
    nh.getParam("/tagger/Blob_Params/max_convexity", max_convexity);

  Receiver r;
  ros::Duration(1.0).sleep();

  r.tag();
  r.publish_data();

  cout << "Tagging complete" << endl;

  if (done)
  {
	cout << "Done" << endl;
    ros::shutdown();
    return 0;
  }

  if(!ros::ok())
  {
	cout << "ros not ok" << endl;
    return 0;
  }

  cout << "ros shutdown in tagger" << endl;
  ros::shutdown();
}
