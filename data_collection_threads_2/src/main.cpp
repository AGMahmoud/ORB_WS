#include <iostream>
#include <stdio.h>
#include <string>
#include <thread>
#include <signal.h>
#include "main_zed.hpp"
#include "main_rs.hpp"
#include "main_uwb.cpp"
#include <ctime>
#include <time.h>
//#include "main_zed.cpp"
//#include "main_rs.cpp"
using namespace std;

#include <signal.h>
bool start_zed;

int zed_ready;
int rs_ready=0;
double zed_pose[6];
double rs_pose[3];
unsigned long long zed_time=0;
double rs_time=0.0;

void nix_exit_handler(int s) {
    start_zed = false;
    //start_rs  = false;
}


// Set the function to handle the CTRL-C
void SetCtrlHandler() {

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = nix_exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGTSTP, &sigIntHandler, NULL);

}
void main_serial();
int main ()
{
string uwb_data;
	start_zed=false;
	//start_rs=false;
	zed_ready=0;
	//rs_ready=false;
	char x;
	SetCtrlHandler();
unsigned long zed_prev_count=0;
unsigned long rs_prev_count=0;
unsigned long long zed_prev_time=0;
double rs_prev_time=0.0;
double zed_fps=0.0;
double rs_fps=0.0;


	thread thread1(main_zed,ref(start_zed),ref(zed_ready),zed_pose,ref(zed_time));

	thread thread2(main_rs,ref(start_zed),ref(rs_ready),rs_pose, ref(rs_time));

	thread thread3(main_uwb,ref(start_zed),ref(uwb_data));
	thread3.detach();
	
	while(zed_ready==0||rs_ready==0)
	cout <<"zed_ready " <<zed_ready<<"\t start_zed "<<start_zed<<"\trs_ready "<<rs_ready<<endl;
	start_zed=true;

	while(start_zed)
	{
	zed_fps=1000000000.0*(zed_ready-zed_prev_count)/(zed_time-zed_prev_time);
	rs_fps=1000.0*(rs_ready-rs_prev_count)/(rs_time-rs_prev_time);
	system("clear");
	printf("ZED_Solution:\t X: %.6f \t Y: %.6f \t Z: %.6f \t ZED_FPS: %.4f\n",zed_pose[0],zed_pose[1],zed_pose[2],zed_fps);
	printf("RS_Solution:\t X: %.6f \t Y: %.6f \t Z: %.6f \t RS_FPS: %.4f\n",rs_pose[0],rs_pose[1],rs_pose[2],rs_fps);

	cout<<uwb_data;
	//cout<<"zed solution:\t X :"<<zed_pose[0]<<"\t Y: "<<zed_pose[1]<<"\t Z: "<<zed_pose[2]<<"\tframe count: "<<zed_ready<<"\t fps: "<< 1000000000.0*(zed_ready-zed_prev_count)/(zed_time-zed_prev_time)<<endl;
	//cout<<"RS solution:\t X :"<<rs_pose[0]<<"\t Y: "<<rs_pose[1]<<"\t Z: "<<rs_pose[2]<<"\tframe count: "<<rs_ready<<"\t fps :"<<1000.0*(rs_ready-rs_prev_count)/(rs_time-rs_prev_time)<<endl;
	zed_prev_time=zed_time;
	rs_prev_time=rs_time;
	zed_prev_count=zed_ready;
	rs_prev_count=rs_ready;
	usleep(1000000);

	
	/*rs_fps=1000*(rs_ready-rs_prev_count)/(rs_time-rs_prev_time);
	zed_fps=1000000000*(zed_ready-zed_prev_count)/(zed_time-zed_prev_time);
	zed_prev_time=zed_time;
	rs_prev_time=rs_time;
	cout<<"zed solution:\t X :"<<zed_pose[0]<<"\t Y: "<<zed_pose[1]<<"\t Z: "<<zed_pose[2]<<"\tframe rate: "<<zed_fps<<endl;
	cout<<"RS solution:\t X :"<<rs_pose[0]<<"\t Y: "<<rs_pose[1]<<"\t Z: "<<rs_pose[2]<<"\tframe rate: "<<rs_fps<<endl;
	cout<<"UWB solution\t"<<uwb_data<<endl;
	zed_prev_count=zed_ready;
	rs_prev_count=rs_ready;*/
		}
	cout <<"zed_ready " <<zed_ready<<"\t start_zed "<<start_zed<<"\trs_ready "<<rs_ready<<endl;


	//usleep(5000);

						
							thread1.join();
							cout<<"zed_joined\n";
							thread2.join();
						cout<<"rs_joined\n";
usleep(1000);
	
printf("closing main\n");

return 0;
}



//pthread_getschedparam(thread1.native_handle(), &policy, &sch);
	//cout<<"priority \t : \t"<<sch.sched_priority<<endl;
   /* sch.sched_priority = 20;
    if (pthread_setschedparam(t1.native_handle(), SCHED_FIFO, &sch)) {
        std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }*/
