#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <iostream>
using namespace std;

class BaseCommunicator
{
    private:
        int sockfd, portNo, n;
        struct sockaddr_in serv_addr;
        struct hostent *server;
        char buffer[256];

    public:
        BaseCommunicator();
        ~BaseCommunicator();
        int sendDistance(float distance); // CMD01
        int sendVelocity(float speed); // CMD02
        int sendAcceleration(float accel); // CMD03
        int sendStripeCount(int stripes); // CMD04
        int sendRpmFl(float rpmfl); // CMD05
        int sendRpmFr(float rpmfr); // CMD06
        int sendRpmBl(float rpmBl); // CMD07
        int sendRpmBr(float rpmBr); // CMD08
        int sendData(string message);
        // int sendPosition(float position);
        // int sendPodTemperature(float temp);
        // int sendGroundProximity(float prox);
        // int sendRailProximity(float prox);
        // int batteryTemperature(float temp);
        // int sendBatteryCurrent(float current);
        // int sendBatteryVoltage(float volt);
        // int sendAccum1(int status);
        // int sendAccum2(int status);
        // int sendPump1(int status);
        // int sendPump2(int status);
        // void receiverThread();
};
