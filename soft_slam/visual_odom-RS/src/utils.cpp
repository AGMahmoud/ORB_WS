#include "utils.h"
#include "evaluate_odometry.h"
#include <opencv2/opencv.hpp>


// --------------------------------
// Visualization
// --------------------------------
void drawFeaturePoints(cv::Mat image, std::vector<cv::Point2f>& points)
{
    int radius = 2;
    
    for (int i = 0; i < points.size(); i++)
    {
        circle(image, cvPoint(points[i].x, points[i].y), radius, CV_RGB(255,255,255));
    }
}

void display(int frame_id, cv::Mat& trajectory, cv::Mat& pose, std::vector<Matrix>& pose_matrix_gt, float fps, bool show_gt)
{
    char text[100];
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 0.8;
    int thickness = 1;  
    cv::Point textOrg(10, 40);
    // draw estimated trajectory 
    int x = int(pose.at<double>(0)*10) + 600;
    int y = int(pose.at<double>(2)*10) + 100;
    circle(trajectory, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);

    if (show_gt)
    {
      // draw ground truth trajectory 
      cv::Mat pose_gt = cv::Mat::zeros(1, 3, CV_64F);
      
      pose_gt.at<double>(0) = pose_matrix_gt[frame_id].val[0][3];
      pose_gt.at<double>(1) = pose_matrix_gt[frame_id].val[0][7];
      pose_gt.at<double>(2) = pose_matrix_gt[frame_id].val[0][11];
      x = int(pose_gt.at<double>(0)) + 300;
      y = int(pose_gt.at<double>(2)) + 100;
      circle(trajectory, cv::Point(x, y) ,1, CV_RGB(255,255,0), 2);
    }
    // print info

    rectangle( trajectory, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "FPS: %02f X: %f \t Y: %f ", fps,pose.at<double>(0),pose.at<double>(2));
    putText(trajectory, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

    cv::imshow( "Trajectory", trajectory );


    cv::waitKey(1);
}



void featureSetToPointClouds(cv::Mat& points3D,  PointCloud::Ptr cloud)
{
    // append
    int size = points3D.rows;
    for (int i = 0; i < size; ++i)
    {
        PointT point;
        point.x = points3D.at<float>(i, 0);
        point.y = points3D.at<float>(i, 1);
        point.z = points3D.at<float>(i, 2);
        cloud->points.push_back(point);
    }
}

void featureSetToPointCloudsValid(cv::Mat& points3D,  PointCloud::Ptr cloud, std::vector<bool>& valid)
{
    // append only valid points
    int size = points3D.rows;
    for (int i = 0; i < size; ++i)
    {
        if (valid[i])
        {
            PointT point;
            point.x = points3D.at<float>(i, 0);
            point.y = points3D.at<float>(i, 1);
            point.z = points3D.at<float>(i, 2);
            cloud->points.push_back(point);
        }
    }
}

void mapPointsToPointCloudsAppend(std::vector<MapPoint>& mapPoints,  PointCloud::Ptr cloud)
{
    // append only valid points
    size_t mapSize = mapPoints.size();
    size_t start = cloud->size();
    for (size_t i = start; i < mapSize; ++i)
    {
            PointT point;
            point.x = mapPoints[i].mWorldPos.at<float>(0);
            point.y = mapPoints[i].mWorldPos.at<float>(1);
            point.z = mapPoints[i].mWorldPos.at<float>(2);
            cloud->points.push_back(point);
    }
}

void simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  viewer->setBackgroundColor (0, 0, 0);
  viewer->removePointCloud ("sample cloud");

  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->spinOnce();
  // viewer->spin();
}

// --------------------------------
// Transformation
// --------------------------------


void integrateOdometryStereo(int frame_i, cv::Mat& rigid_body_transformation, cv::Mat& frame_pose, const cv::Mat& rotation, const cv::Mat& translation_stereo)
{

    // std::cout << "rotation" << rotation << std::endl;
    // std::cout << "translation_stereo" << translation_stereo << std::endl;

    
    cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

    cv::hconcat(rotation, translation_stereo, rigid_body_transformation);
    cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);

    // std::cout << "rigid_body_transformation" << rigid_body_transformation << std::endl;

    double scale = sqrt((translation_stereo.at<double>(0))*(translation_stereo.at<double>(0)) 
                        + (translation_stereo.at<double>(1))*(translation_stereo.at<double>(1))
                        + (translation_stereo.at<double>(2))*(translation_stereo.at<double>(2))) ;

    // frame_pose = frame_pose * rigid_body_transformation;
    std::cout << "scale: " << scale << std::endl;

    // rigid_body_transformation = rigid_body_transformation.inv();
    // if ((scale>0.1)&&(translation_stereo.at<double>(2) > translation_stereo.at<double>(0)) && (translation_stereo.at<double>(2) > translation_stereo.at<double>(1))) 
    if (scale > 0. && scale < 1) 
    {
      // std::cout << "Rpose" << Rpose << std::endl;

      frame_pose = frame_pose * rigid_body_transformation;

    }
    else 
    {
     std::cout << "[WARNING] scale below 0.1, or incorrect translation" << std::endl;
    }
}

bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{
 
    assert(isRotationMatrix(R));
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
     
}

// --------------------------------
// I/O
// --------------------------------

void loadGyro(std::string filename, std::vector<std::vector<double>>& time_gyros)
// read time gyro txt file with format of timestamp, gx, gy, gz
{
    std::ifstream file(filename);

    std::string value;
    double timestamp, gx, gy, gz;

    while (file.good())
    {    

         std::vector<double> time_gyro;

         getline ( file, value, ' ' );
         timestamp = stod(value);
         time_gyro.push_back(timestamp);

         getline ( file, value, ' ' );
         gx = stod(value);
         time_gyro.push_back(gx);

         getline ( file, value, ' ' );
         gy = stod(value);
         time_gyro.push_back(gy);

         getline ( file, value);
         gz = stod(value);
         time_gyro.push_back(gz);

         // printf("t: %f, gx: %f, gy: %f, gz: %f\n" , timestamp, gx, gy, gz);    

         time_gyros.push_back(time_gyro);
    }
}

void loadImageLeft(cv::Mat& image_color, cv::Mat& image_gary, int frame_id, std::string filepath){
    char file[200];
    sprintf(file, "left/left_%06d.png", frame_id);
    printf(file);
    // sprintf(file, "image_0/%010d.png", frame_id);
    std::string filename = filepath + std::string(file);

    image_gary = cv::imread(filename,  cv::IMREAD_GRAYSCALE);
   // cvtColor(image_color, image_gary, cv::COLOR_BGR2GRAY);
}

void loadImageRight(cv::Mat& image_color, cv::Mat& image_gary, int frame_id, std::string filepath){
    char file[200];
    sprintf(file, "right/right_%06d.png", frame_id);
    printf(file);
    // sprintf(file, "image_0/%010d.png", frame_id);
    std::string filename = filepath + std::string(file);

    image_gary = cv::imread(filename, cv::IMREAD_GRAYSCALE);
   // cvtColor(image_color, image_gary, cv::COLOR_BGR2GRAY);
}











