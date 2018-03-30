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

namespace hyped {
namespace communications {

class BaseCommunicator
{
    private:
        int sockfd, portNo, n;
        struct sockaddr_in serv_addr;
        struct hostent *server;
        char buffer[256];
        // char* ipAddress;

    public:
        BaseCommunicator();
        BaseCommunicator(char* ip);
        bool setUp();
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
        // void receiverThread();
};

}}
