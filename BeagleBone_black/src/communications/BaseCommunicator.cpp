#include "BaseCommunicator.hpp"
#include <sstream>
#include <thread>
using namespace std;

int sockfd, portNo, n;
struct sockaddr_in serv_addr;
struct hostent *server;
char buffer[256];

BaseCommunicator :: BaseCommunicator()
{
    portNo = 5695;
    sockfd = socket(AF_INET, SOCK_STREAM, 0); // socket(int domain, int type, int protocol)

    if (sockfd < 0)
    {
        printf("ERROR: CANNOT OPEN SOCKET.\n");
    }

    server = gethostbyname("localHost");

    if (server == NULL)
	{
		fprintf(stderr, "ERROR: INCORRECT BASE-STATION IP, OR BASE-STATION S/W NOT RUNNING.\n");
		exit(0);
	}

	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET; // server byte order
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(portNo);

	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("ERROR: CANNOT ESTABLISH CONNECTION TO BASE-STATION.");
    }

    n = read(sockfd, buffer, 255);

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

/*
int BaseCommunicator :: sendPosition(float position)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << position;

    return sendData("CMD03" + ss.str() + "\n");
}

int BaseCommunicator :: sendPodTemperature(float temp)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << temp;

    return sendData("CMD04" + ss.str() + "\n");
}

int BaseCommunicator :: sendGroundProximity(float prox)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << prox;

    return sendData("CMD07" + ss.str() + "\n");
}

int BaseCommunicator :: sendRailProximity(float prox)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << prox;

    return sendData("CMD08" + ss.str() + "\n");
}

int BaseCommunicator :: batteryTemperature(float temp)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << temp;

    return sendData("CMD09" + ss.str() + "\n");
}

int BaseCommunicator :: sendBatteryCurrent(float current)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << current;

    return sendData("CMD10" + ss.str() + "\n");
}

int BaseCommunicator :: sendBatteryVoltage(float volt)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << volt;

    return sendData("CMD11" + ss.str() + "\n");
}

int BaseCommunicator :: sendAccum1(int status)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << status;

    return sendData("CMD12" + ss.str() + "\n");
}

int BaseCommunicator :: sendAccum2(int status)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << status;

    return sendData("CMD13" + ss.str() + "\n");
}

int BaseCommunicator :: sendPump1(int status)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << status;

    return sendData("CMD14" + ss.str() + "\n");
}

int BaseCommunicator :: sendPump2(int status)
{
    stringstream ss (stringstream::in | stringstream::out);
    ss << status;

    return sendData("CMD15" + ss.str() + "\n");
}
*/

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
