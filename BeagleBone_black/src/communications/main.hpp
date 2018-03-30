#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "communications/BaseCommunicator.hpp"

namespace hyped {

using utils::concurrent::Thread;

namespace communications {

class Main : public Thread {
    public:
        explicit Main(uint8_t id);
        void run() override;

    private:
        BaseCommunicator* baseCommunicator;
        data::Data& data = data::Data::getInstance();
        data::Navigation nav;
        data::Motors mtr;
};

}}
