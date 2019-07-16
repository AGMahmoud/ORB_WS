#include <iostream>
#include <stdio.h>
#include <string>
#include <thread>
#include <signal.h>
#include <pthread.h>
#include "main_zed.hpp"
#include "main_rs.hpp"
#include "main_uwb.cpp"
//#include "main_zed.cpp"
//#include "main_rs.cpp"
using namespace std;

#include <signal.h>
/*
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
struct arg_struct args;

*/

float args_local[9];
pthread_t thread_zed, thread_rs,thread_uwb;
//int* ref_e=&exit_app;
void nix_exit_handler(int s) {
printf("exiting ...................................\n");
    args_local[0] = 99.0;
    
}


// Set the function to handle the CTRL-C
void SetCtrlHandler() {

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = nix_exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

}
int main ()
{
	float *args=args_local;
	args[0]= 0;
	args[1]= 0;
	args[2]= 0;
	char x;
	SetCtrlHandler();
	
	int  iret_zed, iret_rs, iret_uwb;
	iret_zed = pthread_create( &thread_zed, NULL, main_zed, (void *) args);
     if(iret_zed)
     {
         fprintf(stderr,"Error - pthread_create() return code: %d\n",iret_zed);
         exit(EXIT_FAILURE);
     }

     iret_rs = pthread_create( &thread_rs, NULL, main_rs, (void *) args);
     if(iret_rs)
     {
         fprintf(stderr,"Error - pthread_create() return code: %d\n",iret_rs);
         exit(EXIT_FAILURE);
     }
    /* iret_uwb = pthread_create( &thread_uwb, NULL, main_rs, (void *) &exit_app);
     if(iret_uwb)
     {
         fprintf(stderr,"Error - pthread_create() return code: %d\n",iret_uwb);
         exit(EXIT_FAILURE);
     }*/
         while(args[0]==0.0)
         {
         cout<<"not yet 1";
         if(args[1]==1.0&&args[1]==1.0)
         {
         cout<<"not yet 2 \n";
         args[0]+=1.0;
         }
         }
     while(args[0]!=99)
     {
     
     usleep(100000);
     cout<<"main:\tzed: "<<args[1]<<"\trs: "<<args[2]<<"\texit:"<<args[0]<<endl;
     cout<<"zed = "<<args[1]<<"\t X :"<<args[3]<<"\t Y :"<<args[4]<<"\t Z :"<<args[5]<<endl<<"rs  = "<<args[2]<<"\t X :"<<args[6]<<"\t Y :"<<args[7]<<"\t Z :"<<args[8]<<endl;
	}
	//cout<<"main\n";
     //system("clear");
     //cout<<args.exit_app<<endl;

     
    
	pthread_join( thread_zed, NULL);
     pthread_join( thread_rs, NULL); 
     //pthread_detach( thread_uwb); 
cout<<args[0]<<endl;

printf("closing.......................\n");
return 0;

}
