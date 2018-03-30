#include "communications/main.hpp"

namespace hyped {
namespace communications {

Main::Main(uint8_t id) : Thread(id)
{
    baseCommunicator = new BaseCommunicator(); // To use IP address: BaseCommunicator((char *) "127.0.0.1");
    baseCommunicator->setUp();
    // TODO: start receiverThread here
}

void Main::run()
{
    while (1) {
        nav = data.getNavigationData();
        mtr = data.getMotorData();
        baseCommunicator->sendDistance(nav.distance);
        baseCommunicator->sendVelocity(nav.velocity);
        baseCommunicator->sendAcceleration(nav.acceleration);
        baseCommunicator->sendStripeCount(nav.stripe_count);
        baseCommunicator->sendRpmFl(mtr.rpm_FL);
        baseCommunicator->sendRpmFr(mtr.rpm_FR);
        baseCommunicator->sendRpmBl(mtr.rpm_BL);
        baseCommunicator->sendRpmBr(mtr.rpm_BR);
    }
}

}}
