#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>
#include <iostream>
#include <cstdlib>
#include <sstream>

using namespace cv;
using namespace cv::dnn;
using namespace std;




static const String classes[] = {
    "Background", "Road", "Sidewalk", "Building", "Wall", "Fence", "Pole",
    "TrafficLight", "TrafficSign", "Vegetation", "Terrain", "Sky", "Person",
    "Rider", "Car", "Truck", "Bus", "Train", "Motorcycle", "Bicycle"
};


static const int kNumClasses = 20; // classes.length();


static const Vec3b colors[] = {
    Vec3b(0, 0, 0), Vec3b(244, 126, 205), Vec3b(254, 83, 132), Vec3b(192, 200, 189),
    Vec3b(50, 56, 251), Vec3b(65, 199, 228), Vec3b(240, 178, 193), Vec3b(201, 67, 188),
    Vec3b(85, 32, 33), Vec3b(116, 25, 18), Vec3b(162, 33, 72), Vec3b(101, 150, 210),
    Vec3b(237, 19, 16), Vec3b(149, 197, 72), Vec3b(80, 182, 21), Vec3b(141, 5, 207),
    Vec3b(189, 156, 39), Vec3b(235, 170, 186), Vec3b(133, 109, 144), Vec3b(231, 160, 96)
};


static void colorizeSegmentation(const Mat &score, Mat &segm)
{
    const int rows = score.size[2];
    const int cols = score.size[3];
    const int chns = score.size[1];

    Mat maxCl = Mat::zeros(rows, cols, CV_8UC1);
    Mat maxVal(rows, cols, CV_32FC1, score.data);
    for (int ch = 1; ch < chns; ch++)
    {
        for (int row = 0; row < rows; row++)
        {
            const float *ptrScore = score.ptr<float>(0, ch, row);
            uint8_t *ptrMaxCl = maxCl.ptr<uint8_t>(row);
            float *ptrMaxVal = maxVal.ptr<float>(row);
            for (int col = 0; col < cols; col++)
            {
                if (ptrScore[col] > ptrMaxVal[col])
                {
                    ptrMaxVal[col] = ptrScore[col];
                    ptrMaxCl[col] = (uchar)ch;
                }
            }
        }
    }

    segm.create(rows, cols, CV_8UC3);
    for (int row = 0; row < rows; row++)
    {
        const uchar *ptrMaxCl = maxCl.ptr<uchar>(row);
        Vec3b *ptrSegm = segm.ptr<Vec3b>(row);
        for (int col = 0; col < cols; col++)
        {
            ptrSegm[col] = colors[ptrMaxCl[col]];
        }
    }
};

// String modelFile = "/home/nadia/Downloads/opt_deeplabv3_mnv2_513.pb";

String modelFile = "/home/nadia/Documents/Programming/ssd_mobilenet_v3/ssd_mobil_v3.pb";
// String configFile = "/home/nadia/Documents/Programming/ssd_mobilenet_v3/pipeline.config";
String configFile = "/home/nadia/Documents/Programming/ssd_mobilenet_v3/ssd_mobil_v3.pbtxt";


Size inputSize = Size(513, 513);
Scalar mean_scalar = Scalar(127.5, 127.5, 127.5);
float std_variable = 0.007843;
bool swapRB = false;
float scale = 0.5; // TODO - What should this be?

cv::Mat semanticSegmentation(const cv::Mat& mono_img) {

    cv::Mat seg_img; 
    cv::inRange(mono_img, cv::Scalar(100,100,100), cv::Scalar(200,200,200), seg_img);

    return seg_img;

}; 

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

    std::cout << "111" << std::endl;
    cv::Mat mono_img = cv_bridge::toCvShare(msg, "bgr8")->image; // img

    std::cout << "oh god" << std::endl;
    dnn::Net net = dnn::readNetFromTensorflow(modelFile, configFile);
    std::cout << "211" << std::endl;


    /*
      img = cv.imread('example.jpg')
      rows = img.shape[0]
      cols = img.shape[1]
      cvNet.setInput(cv.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))
      cvOut = cvNet.forward()

      for detection in cvOut[0,0,:,:]:
          score = float(detection[2])
          if score > 0.3:
              left = detection[3] * cols
              top = detection[4] * rows
              right = detection[5] * cols
              bottom = detection[6] * rows
              cv.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), (23, 230, 210), thickness=2)

      cv.imshow('img', img)
      cv.waitKey()
    */    


    //Convert Mat to batch of images
    // cv::Mat inputBlob = blobFromImage(mono_img, 1 , mono_img.size() , Scalar(), true, false); // Size(1024, 512)
    

    // resize(mono_img, mono_img, Size(513, 513), 0, 0, INTER_LINEAR_EXACT);       //FCN accepts 500x500 BGR-images
    // Mat inputBlob = blobFromImage(img, 1, Size(), Scalar(), false);   //Convert Mat to batch of images
    // cv::Mat inputBlob = blobFromImage(mono_img, 1, Size(), Scalar(), false);   //Convert Mat to batch of images


    // resize(mono_img, mono_img, Size(513, 513), 0, 0, INTER_LINEAR_EXACT);       //FCN accepts 500x500 BGR-images
    cv::Mat inputBlob = blobFromImage(mono_img, std_variable, inputSize, mean_scalar, swapRB);
    std::cout << "311" << std::endl;
    net.setInput(inputBlob);
    
    std::cout << "2 " << std::endl;
    
    Mat result = net.forward();

    std::cout << "3 " << std::endl;

    // std::cout << "Output blob: " << result.size[0] << " x " << result.size[1] << " x " << result.size[2] << " x " << result.size[3] << "\n";
    // std::cout << "Inference time, ms: " << tm.getTimeMilli()  << std::endl;

    // cv::Mat segm, show;
    // colorizeSegmentation(result, segm);

    // cv::resize(segm, segm, mono_img.size(), 0, 0, cv::INTER_NEAREST);
    // addWeighted(mono_img, 0.1, segm, 0.9, 0.0, show);

    cv::Mat seg_img = semanticSegmentation(mono_img);

    cv::imshow("view" , seg_img);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "semantic_segmentation_node");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/tesse/left_cam/rgb/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
