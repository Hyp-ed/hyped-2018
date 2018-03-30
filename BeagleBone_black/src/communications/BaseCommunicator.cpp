#include "BaseCommunicator.hpp"
#include <sstream>
#include <thread>
using namespace std;

namespace hyped {
namespace communications {

int sockfd, portNo, n;
struct sockaddr_in serv_addr;
struct hostent *server;
char buffer[256];
char* ipAddress = (char *) "localhost";

BaseCommunicator::BaseCommunicator()
{

}

BaseCommunicator::BaseCommunicator(char* ip)
{
    ipAddress = ip;
}

bool BaseCommunicator::setUp()
{
    portNo = 5695;
    sockfd = socket(AF_INET, SOCK_STREAM, 0); // socket(int domain, int type, int protocol)

    if (sockfd < 0)
    {
        printf("ERROR: CANNOT OPEN SOCKET.\n");
        return false;
    }

    server = gethostbyname(ipAddress);

    if (server == NULL)
    {
        fprintf(stderr, "ERROR: INCORRECT BASE-STATION IP, OR BASE-STATION S/W NOT RUNNING.\n");
        // exit(0);
        return true;
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET; // server byte order
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portNo);

    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("ERROR: CANNOT ESTABLISH CONNECTION TO BASE-STATION.\n");
        return false;
    }

    // n = read(sockfd, buffer, 255);
    return true;
}

BaseCommunicator :: ~BaseCommunicator()
{
	// DESTRUCTOR: upon deletion of pointer to object (instance of this class), socket to base will be closed.
	close(sockfd);
}

int BaseCommunicator :: sendDistance(float distance)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << distance;

    return sendData("CMD01" + ss.str() + "\n");
}

int BaseCommunicator :: sendVelocity(float speed)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << speed;

    return sendData("CMD02" + ss.str() + "\n");
}

int BaseCommunicator :: sendAcceleration(float accel)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << accel;

    return sendData("CMD03" + ss.str() + "\n");
}

int BaseCommunicator :: sendStripeCount(int stripes)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << stripes;

    return sendData("CMD04" + ss.str() + "\n");
}

int BaseCommunicator :: sendRpmFl(float rpmfl)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << rpmfl;

    return sendData("CMD05" + ss.str() + "\n");
}

int BaseCommunicator :: sendRpmFr(float rpmfr)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << rpmfr;

    return sendData("CMD06" + ss.str() + "\n");
}

int BaseCommunicator :: sendRpmBl(float rpmbl)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << rpmbl;

    return sendData("CMD07" + ss.str() + "\n");
}

int BaseCommunicator :: sendRpmBr(float rpmbr)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << rpmbr;

    return sendData("CMD08" + ss.str() + "\n");
}

int BaseCommunicator :: sendData(string message)
{
	// Incoming strings should be terminated by "...\n".
	bzero(buffer,256);
	const char *data = message.c_str();
	n = write(sockfd, data, strlen(data));
	if (n < 0) printf("ERROR: CANNOT WRITE TO SOCKET.");
	n = read(sockfd, buffer, 255);
	if (n < 0) printf("ERROR: CANNOT READ FROM SOCKET.");

    return atoi(buffer);
}

/*
// Thread must be declared and joined within calling code.
void BaseCommunicator :: receiverThread()
{
	while (true)
	{
		n = read(sockfd, buffer, 255);
		int command = atoi(buffer);

		switch (command)
		{
			case 1:
                printf("ECHO message (1) received.\n");
				// Do nothing, 1 represents echo message
				break;
			case 2:
				printf("READY TO LAUNCH\n");
		}
	}
}
*/
}} // namespace hyped::communcations
