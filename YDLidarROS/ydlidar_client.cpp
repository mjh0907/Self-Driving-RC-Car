/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>

#define SERVER_PORT htons(50007)
#define RAD2DEG(x) ((x)*180./M_PI)
int clientSock = 0;
void setup()
{
    char buffer[1000];
    int n;
    int serverSock=socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in serverAddr; 
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = SERVER_PORT;
    serverAddr.sin_addr.s_addr = INADDR_ANY;

/* bind (this socket, local address, address length)
   bind server socket (serverSock) to server address (serverAddr).  
   Necessary so that server can use a specific port */ 
    bind(serverSock, (struct sockaddr*)&serverAddr, sizeof(struct sockaddr));

// wait for a client
/* listen (this socket, request queue length) */
    listen(serverSock,1);

    bzero(buffer, 1000);

    sockaddr_in clientAddr;
    socklen_t sin_size=sizeof(struct sockaddr_in);
    clientSock=accept(serverSock,(struct sockaddr*)&clientAddr, &sin_size);
}
#define maxCount 2000
float message[maxCount];
bool firstTime = true;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //printf("send to socket");
    int count = scan->scan_time / scan->time_increment;
    printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    int rcount = 0;
    //printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	size_t idx = (180+degree) * 4 + 0.3;
        if(true) {// && degree > -5 && degree< 5) {
            //printf("[YDLIDAR INFO]: idx: angle-distance : [%i, %f, %f, %i]\n", idx, degree, scan->ranges[i], i);
        }
        //message[i*2] = degree;
        //message[i*2+1] = scan->ranges[i];
        message[idx] = degree;
        message[idx+1] = scan->ranges[i];
	rcount = idx+2; //- (idx+2)%2;
	if (idx >= 1438) {
	    if (!firstTime) {
		//write(clientSock, &message, 2*count*sizeof(float));
		write(clientSock, &message, rcount*sizeof(float));
	    }
	}
    }
    
    firstTime = false;
}

int main(int argc, char **argv)
{
    setup();
    ros::init(argc, argv, "ydlidar_client");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::spin();
    close(clientSock);
    return 0;
}
