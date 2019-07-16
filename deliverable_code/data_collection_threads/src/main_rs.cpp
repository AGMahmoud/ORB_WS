#include <librealsense2/rs.hpp>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <string>
#include <thread>
#include <math.h>
#include <signal.h>
#include <fstream>
//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "stb_image_write.h"
#include <iomanip>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <unistd.h>
#include <ctime>
#include <time.h>

struct arg_struct {
	int exit_app;
    int zed;
    double zed_x_pos;
    double zed_y_pos;
    double zed_z_pos;
    double rs_x_pos;
    double rs_y_pos;
    double rs_z_pos;
    int rs;
};

using namespace cv;
using namespace std;
using namespace rs2;


void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}


class GyroBias
{
  private:
    int calibrationUpdates;
    double minX, maxX;
    double minY, maxY;
    double minZ, maxZ;


  public:
    bool isSet;

    double x;
    double y;
    double z;

    GyroBias()
    {
        reset();
    }

    void reset()
    {
        calibrationUpdates = 0;

        minX = 1000;
        minY = 1000;
        minZ = 1000;
        maxX = -1000;
        maxY = -1000;
        maxZ = -1000;

        x = 0;
        y = 0;
        z = 0;

        isSet = false;
    }


    bool update(double gx, double gy, double gz)
    {
        if (calibrationUpdates < 50)
        {   
            maxX = std::max(gx, maxX);
            maxY = std::max(gy, maxY);
            maxZ = std::max(gz, maxZ);

            minX = std::min(gx, minX);
            minY = std::min(gy, minY);
            minZ = std::min(gz, minZ);

            calibrationUpdates++;
            return false;
        }
        else if (calibrationUpdates == 50)
        {
            x = (maxX + minX)/2.0;
            y = (maxY + minY)/2.0;
            z = (maxZ + minZ)/2.0;
            calibrationUpdates++;

            /*std::cout << "BIAS-X: " << minX << " - " << maxX << std::endl;
            std::cout << "BIAS-Y: " << minY << " - " << maxY << std::endl;
            std::cout << "BIAS-Z: " << minZ << " - " << maxZ << std::endl;
            std::cout << "BIAS: " << x << " " << y << " " << z << std::endl;
            */
            isSet = true;

            return true;
        }
        else
        {
            return false;
        }
    }
};
int main_rs(bool &start, int &rs_ready, double rs_sol[], double &rs_time)
{
	cout<<"rs_start"<<start<<endl;
//int* exit_app=(int*) e;
//int ex_app=*exit_app;
FILE *img_time,*accel_data,*POSE_data,*camera_params,*gyro_data;

cv::Matx33d K_l,K_r,K_l2;
cv::Vec4d D_l,D_r;
    rs2::pipeline pipe;
int count = 0;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_FISHEYE,1,RS2_FORMAT_Y8, 30 );
    cfg.enable_stream(RS2_STREAM_FISHEYE,2,RS2_FORMAT_Y8, 30 );
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    char img_no[80];
    GyroBias bias;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    img_time=fopen("../Data/Realsense/image_ts.txt","w");
    accel_data=fopen("../Data/Realsense/accel_data.txt","w");
    gyro_data=fopen("../Data/Realsense/gyro_data.txt","w");
    POSE_data=fopen("../Data/Realsense/pose_data.txt","w");
    camera_params=fopen("../Data/Realsense/camera_params.txt","w");
    //
    // Start capturing
    //
    std::cout<<"1 \n";
    pipe.start(cfg);
    std::cout<<"2 \n";
    bool firstAccel = true;
    double last_ts[RS2_STREAM_COUNT];
    double dt[RS2_STREAM_COUNT];
// Capture 30 frames to give autoexposure, etc. a chance to settle
    while (count<30) 
        {   
            
            rs2::frameset frames;
            //pipe.wait_for_frames(&frames);
            if (!pipe.poll_for_frames(&frames))
        {
            // Redraw etc
            
            continue;
        }
        
      for (auto f : frames)
        {
            rs2::stream_profile profile = f.get_profile();
            if(profile.stream_name()=="Fisheye 1" )
                {
                    count++;
                    auto video_stream_prof = profile.as<rs2::video_stream_profile>();
                    auto i = video_stream_prof.get_intrinsics();
                   //printf("left:  %f\t%f\t%f\t%f\n",i.ppx,i.ppy,i.fx,i.fy);
                    //printf("left: %f\t%f\t%f\t%f\n",i.coeffs[0],i.coeffs[1],i.coeffs[2],i.coeffs[3]);
                    K_l(0, 0) = i.ppx;
                    K_l(1, 1) = i.ppy;
                    K_l(2, 0) = i.fx;
                    K_l(2, 1) = i.fy;
                    D_l(0)=i.coeffs[0];
                    D_l(1)=i.coeffs[1];
                    D_l(2)=i.coeffs[2];
                    D_l(3)=i.coeffs[3];
                }
            else if(profile.stream_name()=="Fisheye 2" )
                {
                    count++;
                    auto video_stream_prof = profile.as<rs2::video_stream_profile>();
                    auto i = video_stream_prof.get_intrinsics();
                   //printf("right: %f\t%f\t%f\t%f\n",i.ppx,i.ppy,i.fx,i.fy);
                   //printf("right: %f\t%f\t%f\t%f\n",i.coeffs[0],i.coeffs[1],i.coeffs[2],i.coeffs[3]);
                    K_r(0, 0) = i.ppx;
                    K_r(1, 1) = i.ppy;
                    //K_r(0, 2) = i.fx;
                    //K_r(1, 2) = i.fy;
                    D_r(0)=i.coeffs[0];
                    D_r(1)=i.coeffs[1];
                    D_r(2)=i.coeffs[2];
                    D_r(3)=i.coeffs[3];
                }
        }

}
fprintf(camera_params,"left camera\n ppx: %f \n ppy: %f \n fx: %f \n fy: %f \n D1: %f \n D2: %f \n D3: %f \n D4: %f \n",K_l(0, 0),K_l(1, 1), K_l(2, 0),K_l(2, 1),D_l(0),D_l(1),D_l(2),D_l(3));
fprintf(camera_params,"right camera\n ppx: %f \n ppy: %f \n fx: %f \n fy: %f \n D1: %f \n D2: %f \n D3: %f \n D4: %f \n",K_r(0, 0),K_r(1, 1), K_r(2, 0),K_r(2, 1),D_r(0),D_r(1),D_r(2),D_r(3));
fclose(camera_params);
count=0;
cout<<"main_rs ready >>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<endl;
rs_ready+=1;

while(!start)
usleep(10);

cout<<" rs_ started\n";
    // Main loop
    while (start )
    {

  //  cout<<"rs.............................................\n";
//ex_app=*exit_app;
rs2::frameset frames;
cv::Mat two_images(cv::Size(1696, 800), CV_8UC1);
  
 
        if (!pipe.poll_for_frames(&frames))
        {
            // Redraw etc
            
            continue;
        }
count++; 
        //std::cout << "Num frames: " << frames.size() << " ";
        for (auto f : frames)
        {
            rs2::stream_profile profile = f.get_profile();
            

            unsigned long fnum = f.get_frame_number();
            double ts = f.get_timestamp();
            dt[profile.stream_type()] = (ts - last_ts[profile.stream_type()] ) / 1000.0;
            last_ts[profile.stream_type()] = ts;
            
           if(profile.stream_name()=="Fisheye 1" )
                {
                rs_ready++;
                rs_time=ts;
                //cout<<"realsense:"<<args->exit_app<<"\t"<<f.get_frame_number()<<endl;
                fprintf(img_time,"%f\t%d\t%lu\t%lu\n",ts,count,fnum,std::chrono::system_clock::now());
               // auto vf1 = frames.first(RS2_STREAM_FISHEYE,RS2_FORMAT_Y8);
                rs2::video_frame video=f.as<rs2::video_frame>();

               
                sprintf(img_no,"../Data/Realsense/left/left_%06llu.PNG",f.get_frame_number());

                Mat image(Size(video.get_width(), video.get_height()), CV_8UC1, (void*)video.get_data(), Mat::AUTO_STEP);
                Mat und_image;
                //cv::fisheye::undistortImage(image, und_image, K_l, D_l);
                imwrite(img_no,image);
                //image.copyTo(two_images(cv::Rect(  0, 0, 848, 800)));
               // Mat undistort_img;
                //fisheye::undistortImage(image, undistort_img, camera_matrix, distortion, camera_matrix);
                // Update the window with new data
                
                
                // Record per-frame metadata for UVC streams
                std::stringstream csv_file;
                csv_file << "rs-save-to-disk-output-" << f.get_profile().stream_name()
                     << "-metadata.csv";
                metadata_to_csv(f, csv_file.str());
                }
           else if(profile.stream_name()=="Fisheye 2" )
                {
                //auto vf2 = frames.first(RS2_STREAM_FISHEYE,RS2_FORMAT_Y8);
                rs2::video_frame video=f.as<rs2::video_frame>();
                // Write images to disk
               
                sprintf(img_no,"../Data/Realsense/right/right_%06llu.PNG",f.get_frame_number());
                
                Mat image(Size(video.get_width(), video.get_height()), CV_8UC1, (void*)video.get_data(), Mat::AUTO_STEP);
                imwrite(img_no,image);
                //image.copyTo(two_images(cv::Rect( 848, 0, 848, 800)));
                // Record per-frame metadata for UVC streams
                std::stringstream csv_file;
                csv_file << "rs-save-to-disk-output-" << f.get_profile().stream_name()
                     << "-metadata.csv";
                metadata_to_csv(f, csv_file.str()); 
                }
            else if(profile.stream_name()=="Pose" )
                {
                    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
					fprintf(POSE_data," %f \t %f \t %f \t %f \t %f \t %f \t %f \t %lu\n",ts,pose_data.translation.x, pose_data.translation.y,pose_data.translation.z,pose_data.rotation.x, pose_data.rotation.y,pose_data.rotation.z,std::chrono::system_clock::now());
					rs_sol[0]=pose_data.translation.x;
					rs_sol[1]=pose_data.translation.y;
					rs_sol[2]=pose_data.translation.z;
					rs_sol[3]=pose_data.velocity.x;
					rs_sol[4]=pose_data.velocity.y;
					rs_sol[6]=pose_data.velocity.z;
                    // Print the x, y, z values of the translation, relative to initial position
                  //  std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<                    pose_data.translation.y << " " << pose_data.translation.z << " (meters)";
                }else if(profile.stream_name()=="Gyro" )
                {
					auto fg = frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        			rs2::motion_frame gyro = fg.as<rs2::motion_frame>();
        		    rs2_vector gv = gyro.get_motion_data();
        			bias.update(gv.x, gv.y, gv.z);

            		double ratePitch = gv.x - bias.x;
           		    double rateYaw = gv.y - bias.y;
            		double rateRoll = gv.z - bias.z;
            		fprintf(gyro_data," %f \t %f \t %f \t %f \t %lu\n",ts,ratePitch, rateYaw,rateRoll,std::chrono::system_clock::now());
                }else if (profile.stream_name()=="Accel")
                {
					auto fa = frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        			rs2::motion_frame accel = fa.as<rs2::motion_frame>();
					rs2_vector av = accel.get_motion_data();
					fprintf(accel_data," %f \t %f \t %f \t %f \t %lu\n",f.get_timestamp(),av.x, av.y,av.z,std::chrono::system_clock::now());
                }

        }


     //   imshow(window_name, two_images);
       //         waitKey(1);
        
        

      
       
    }
usleep(1000);
cout<<"closing RS\n";











return 0;
}
