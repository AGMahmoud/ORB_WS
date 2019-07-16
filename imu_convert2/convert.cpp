#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <time.h>


//using std;
union data
{
    float data;
    char x[8];

};  

int main()
{
data first_F;
double timestamp,tmp,scale;
FILE *IMU_scale, *IMU_time, *data_out;
IMU_scale=fopen("./scale_5.txt","r");
IMU_time=fopen("./scale_data.txt","r");
data_out=fopen("./scale_out.txt","w");
int c=0,c_ts=0;
while(c<1694)
{
std::cout<<"writting data no "<<c+1<<"\t with ts "<<c_ts+1<<std::endl;
fscanf(IMU_scale,"%lf\n",&scale);
fscanf(IMU_time,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",&timestamp,&tmp,&tmp,&tmp,&tmp,&tmp,&tmp);
fprintf(data_out,"%lf\t%lf\n",timestamp,scale);
for(int i=0;i<4;i++)
{
fscanf(IMU_time,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",&timestamp,&tmp,&tmp,&tmp,&tmp,&tmp,&tmp);
c_ts++;
}
c++;
c_ts++;


}
printf("%lf\n",timestamp);
return 0;
}

