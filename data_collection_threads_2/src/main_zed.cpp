// ZED includes
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <pthread.h>
#include <ctime>
#include <time.h>
//#include <system.h>

// Using namespace
using namespace sl;
using namespace std;
FILE *imu_input, *img_timestamps,*pose_data;



cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

void main_zed(bool &start , int &zed_ready, double zed_sol[],unsigned long long &zed_time) {
	//cout<<"zed_start "<< start<<endl;
	system("gnome-terminal");
    // Create a ZED camera
    Camera zed;
    IMUData imu_data;
    sl::Mat img_rz, img_lz;
    cv::Mat img_r, img_l;
unsigned long prev_time=0,pose_prev_time=0;
pose_data=fopen("../Data/ZED/pose_data.txt","w");
imu_input=fopen("../Data/ZED/imu_readings.txt","w");
img_timestamps=fopen("../Data/ZED/img_timestamps.txt","w");
    // Set configuration parameters for the ZED
    InitParameters initParameters;
    initParameters.camera_resolution = RESOLUTION_HD720;
    //initParameters.depth_mode = DEPTH_MODE_NONE;
    initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
    initParameters.coordinate_units = UNIT_METER; // Set units in meters
    initParameters.camera_fps = 60;
    // Open the camera
    ERROR_CODE err = zed.open(initParameters);
    if (err != SUCCESS) {
        std::cout << toString(err) << std::endl;
        zed.close();
       // return ; // Quit if an error occurred
    }
 // Enable positional tracking with default parameters
    TrackingParameters tracking_parameters;
    err = zed.enableTracking(tracking_parameters);
    if (err != SUCCESS)
        exit(-1);

    // Enable recording with the filename specified in argument
    String path_output("../Data/ZED/record.svo");
    err = zed.enableRecording(path_output, SVO_COMPRESSION_MODE_AVCHD);

    if (err != SUCCESS) {
        std::cout << "Recording initialization error. " << toString(err) << std::endl;
        if (err == ERROR_CODE_SVO_RECORDING_ERROR)
            std::cout << " Note : This error mostly comes from a wrong path or missing writing permissions." << std::endl;
        if (err == ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION)
            std::cout << " Note : This error mostly comes from a non-compatible graphic card. If you are using HEVC compression (H265), please note that most of the graphic card below pascal architecture will not support it. Prefer to use AVCHD compression which is supported on most of NVIDIA graphic cards" << std::endl;

        zed.close();
        //return ;
    }

    // Start recording SVO, stop with Ctrl-C command
    std::cout << "SVO is Recording, use Ctrl-C to stop." << std::endl;
    
    int imu_count = 0,pose_count=0;
int frames_recorded=0;
    //while (!exit_app) {


Pose zed_pose;
usleep(1000000);
zed_ready+=1;

while(!start)
usleep(10);
//cout<<"zed_ready "<<zed_ready<<"\t start"<<start<<endl;
//cout<<"not start\n";
//cout<<"zed_ready "<<zed_ready<<"\t start"<<start<<endl;
cout<<"zed_started\n";
while (start) {

//cout<<"zed>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";
//exit_app=*ex_app;
	    zed.getPosition(zed_pose, REFERENCE_FRAME_WORLD); // Get the pose of the left eye of the camera with reference to the world frame
 if(zed_pose.timestamp!=pose_prev_time)
{
            pose_prev_time=zed_pose.timestamp;
            //printf("zed");
            // Display the translation and timestamp
           // printf("\nTranslation: Tx: %.3f, Ty: %.3f, Tz: %.3f, Timestamp: %llu\n", zed_pose.getTranslation().tx, zed_pose.getTranslation().ty, zed_pose.getTranslation().tz, zed_pose.timestamp);

            // Display the orientation quaternion
            //printf("Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n", zed_pose.getOrientation().ox,zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);
            fprintf(pose_data,"%d\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%lu\t%lu\n",pose_count++,zed_pose.getTranslation().tx,
                    zed_pose.getTranslation().ty, zed_pose.getTranslation().tz,zed_pose.getOrientation().ox,
                    zed_pose.getOrientation().oy, zed_pose.getOrientation().oz,zed_pose.timestamp, std::chrono::system_clock::now());
                    zed_sol[0]=zed_pose.getTranslation().tx;
                    zed_sol[1]=zed_pose.getTranslation().ty;
                    zed_sol[2]=zed_pose.getTranslation().tz;
}

             zed.getIMUData(imu_data, TIME_REFERENCE_CURRENT);
            if(imu_data.timestamp!=prev_time)
{
            prev_time=imu_data.timestamp;
        //    printf("IMU Acceleration: x: %.3f, y: %.3f, z: %.3f\n", imu_data.linear_acceleration.x,  imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
          //  printf("IMU angular velocity: x: %.3f, y: %.3f, z: %.3f\n", imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
            fprintf(imu_input,"%d\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%4lu\t%lu\n",imu_count++,imu_data.linear_acceleration.x,imu_data.linear_acceleration.y, imu_data.linear_acceleration.z,imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z,imu_data.timestamp, std::chrono::system_clock::now());
}
        if (zed.grab() == SUCCESS) {
           
             if (zed.record().status)
{
				zed_ready++;
				
				//cout<<"zed : "<<exit_app<<"\t"<< frames_recorded <<endl;
                unsigned long long timestamp = zed.getTimestamp(TIME_REFERENCE_IMAGE); // Get the timestamp at the time the image was captured
                frames_recorded++;
                zed_time=timestamp;
                //std::cout << "Frame count: " << frames_recorded << "\n";
                
                fprintf(img_timestamps,"%d\t%llu\t%lu\n",frames_recorded,timestamp, std::chrono::system_clock::now());
          
            }
        }

    }
    usleep(1000);
//std::cout<<"stop \n";

    // Stop recording
    zed.disableRecording();
    zed.close();

    std::cout<<"closing zed\n";
    return;

}
