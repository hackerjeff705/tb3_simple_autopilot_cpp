// ---------------- INSTRUCTIONS ----------------
// This must be run inside the rpi4 with ros noetic
// SSH to the rpi4 and copy "raspicam_pub.py" using
// "scp autopilot.cpp ubuntu@{IP_ADDRESS}:~/catkin_ws/src/{ROS_PKG}/src/scripts"
// ----------------------------------------------


// UNIVERSAL //
#include <cmath>
#include <fstream>
#include <iostream>
#include <ros/ros.h>

// CV //
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/ocl.hpp>
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/string_util.h"
#include "tensorflow/lite/model.h"

// CUSTOM ROS MESSAGE //
#include "burger/detection.h"

using namespace cv;
using namespace std;

const size_t width = 300;
const size_t height = 300;
float frame_area = 640 * 480;

std::vector<std::string> Labels;
std::unique_ptr<tflite::Interpreter> interpreter;
std::vector<std::string> target_labels{"trafficlight", "stopsign"};

struct detected_object
{
   string label = "empty";
   float y1 = 0.;
   float x1 = 0.;
   float y2 = 0.;
   float x2 = 0.;
};

static bool getFileContent(std::string fileName)
{
    // Open the File
    std::ifstream in(fileName.c_str());
    //std::ifstream in(fileName);

    // Check if object is valid
    if(!in.is_open()) return false;

    std::string str;
    
    // Read the next line from File untill it reaches the end.
    while (std::getline(in, str))
    {
        // Line contains string of length > 0 then save it in vector
        if(str.size()>0) Labels.push_back(str);
    }

    // Close The File
    in.close();
    return true;
}

detected_object detect_from_video(Mat &src)
{
    Mat image;
    detected_object obj;
    int cam_width = src.cols;
    int cam_height = src.rows;

    // copy image to input as input tensor
    cv::resize(src, image, Size(300,300));
    memcpy(interpreter->typed_input_tensor<uchar>(0), image.data, image.total() * image.elemSize());

    interpreter->SetAllowFp16PrecisionForFp32(true);
    interpreter->SetNumThreads(4); //quad core

    // run your model
    interpreter->Invoke();

    const float* detection_locations = interpreter->tensor(interpreter->outputs()[0])->data.f;
    const float* detection_classes = interpreter->tensor(interpreter->outputs()[1])->data.f;
    const float* detection_scores = interpreter->tensor(interpreter->outputs()[2])->data.f;
    const int    num_detections = *interpreter->tensor(interpreter->outputs()[3])->data.f;

    //there are ALWAYS 10 detections no matter how many objects are detectable
    const float confidence_threshold = 0.5;
    float highest_score = 0;
    for(int i = 0; i < num_detections; i++){
        float score = detection_scores[i];
        if(score > confidence_threshold) {
            int  det_index = (int)detection_classes[i]+1;
            string label = Labels[det_index].c_str();

            if(score > highest_score && *find(target_labels.begin(), target_labels.end(), label) == label) {
                obj.label = label;
                float y1 = detection_locations[4*i  ]*cam_height;
                float x1 = detection_locations[4*i+1]*cam_width;
                float y2 = detection_locations[4*i+2]*cam_height;
                float x2 = detection_locations[4*i+3]*cam_width;

                if (y1 < 0) { y1 = 0.; }
                if (x1 < 0) { x1 = 0.; }
                if (y2 > cam_height) { y2 = cam_height; }
                if (x2 > cam_width) { x2 = cam_width; }

                obj.y1 = y1;
                obj.x1 = x1;
                obj.y2 = y2;
                obj.x2 = x2;
                highest_score = score;

                Rect rec((int)obj.x1, (int)obj.y1, (int)(obj.x2 - obj.x1), (int)(obj.y2 - obj.y1));
                rectangle(src, rec, Scalar(0, 0, 0), 1, 8, 0);
                putText(src, label, Point(obj.x1, obj.y1-5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8, 0);
            }
        }
    }
    return obj;
}

int main(int argc,char ** argv)
{
    float f;
    float FPS[16];
    int i, Fcnt=0;
    Mat frame;
    sensor_msgs::ImagePtr img_msg;
    chrono::steady_clock::time_point Tbegin, Tend;

    // Initialize node handle
    ros::init(argc, argv, "SSD_MobileNetV1");
    ros::NodeHandle nh;

    for(i=0;i<16;i++) FPS[i]=0.0;

    // Load model
    std::unique_ptr<tflite::FlatBufferModel> model = tflite::FlatBufferModel::BuildFromFile("/home/ubuntu/catkin_ws/src/burger/src/models/detect.tflite");
    
    if(!model){
        ROS_INFO_STREAM("Failed to map model\n");
        exit(0);
    }

    // Build the interpreter
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder(*model.get(), resolver)(&interpreter);

    // Resize input tensors, if desired
    interpreter->AllocateTensors();

    // Get the names
    bool result = getFileContent("/home/ubuntu/catkin_ws/src/burger/src/models/COCO_labels.txt");
    if(!result) {
        ROS_INFO_STREAM("loading labels failed");
        exit(-1);
    }

    const int CAMERA_INDEX = 0;
    VideoCapture cap(CAMERA_INDEX, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_INFO_STREAM("ERROR: Unable to open the camera");
        return 0;
    }

    // Publish to the /camera topic
    ros::Publisher pub_obj = nh.advertise<burger::detection>("burger/message", 1);

    // Publish rate/fps
    ros::Rate loop_rate(15);

    ROS_INFO_STREAM("Start grabbing, press CTRL + C on live terminal to terminate");
    while (nh.ok()) {
        // Initialize message
        burger::detection msg;

        // Load image
        cap >> frame;
        if (frame_area != frame.size[1] * frame.size[0]) { frame_area = frame.size[1] * frame.size[0]; }
    
        // Check if grabbed frame has content
        if (frame.empty()) {
            ROS_ERROR_STREAM("Failed to capture image!");
            ros::shutdown();
        }

        // Object detection
        Tbegin = chrono::steady_clock::now();
        detected_object obj = detect_from_video(frame);
        Tend = chrono::steady_clock::now();

        //calculate frame rate
        f = chrono::duration_cast <chrono::milliseconds> (Tend - Tbegin).count();
        if(f > 0.0) FPS[((Fcnt++)&0x0F)] = 1000.0/f;
        for(f=0.0, i=0; i<16; i++){ f+=FPS[i]; }
        putText(frame, format("FPS %0.2f", f/16), Point(10,20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255));

        // Convert image from cv::Mat (OpenCV) type to sensor_msgs/Image (ROS) type
        msg.label = obj.label;
        msg.y1 = obj.y1;
        msg.x1 = obj.x1;
        msg.y2 = obj.y2;
        msg.x2 = obj.x2;
        img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg.image = *img_msg;

        // Publish Frames without detection
        pub_obj.publish(msg);
 
        ros::spinOnce();
        loop_rate.sleep();
    } 
    
    ROS_INFO_STREAM("Closing the camera");
    destroyAllWindows();
    ROS_INFO_STREAM("Bye!");

    return 0;
}

