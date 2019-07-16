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
union data
{
    float data;
    char x[8];

};  

int main()
{
data first_F;

FILE *IMU_read, *IMU_write;
IMU_read=fopen("./IMU.txt","r");
IMU_write=fopen("./IMU_output.txt","w");
int c;
while(c<1000)
{
for(int i=4; i>0; i--)
{

fscanf(IMU_read,"%c",&first_F.x[i-1]);

}
printf("%.9f \t",first_F.data);
c++;
}
return 0;
}

