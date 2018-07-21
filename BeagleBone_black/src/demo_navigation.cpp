
/*
 * Author : Branislav Pilnan
 * Organisation: HYPED
 * Date: 18/07/2018
 * Description:
 * This is the main executable for BeagleBone pod node
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */


#include "data/data.hpp"
#include "navigation/main.hpp"
#include "navigation/navigation.hpp"
#include "sensors/main.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"


using hyped::data::Data;
using hyped::data::ModuleStatus;
using hyped::data::Sensors;
using hyped::data::State;
using hyped::data::StateMachine;
using hyped::navigation::Navigation;
using hyped::utils::concurrent::Thread;
using hyped::utils::Logger;
using hyped::utils::System;


int main(int argc, char* argv[])
{
  System::parseArgs(argc, argv);
  System& sys = System::getSystem();
  Logger log_system(sys.verbose, sys.debug);
  Logger log_nav(sys.verbose_nav, sys.debug_nav);
  Logger log_sensor(sys.verbose_sensor, sys.debug_sensor);

  log_system.INFO("MAIN", "Starting BBB with %d modules", 2);
  log_system.DBG("MAIN", "DBG0");
  log_system.DBG1("MAIN", "DBG1");
  log_system.DBG2("MAIN", "DBG2");
  log_system.DBG3("MAIN", "DBG3");

  Thread* sensors                     = new hyped::sensors::Main(2, log_sensor);
  hyped::navigation::Main* navigation = new hyped::navigation::Main(3, log_nav);

  log_system.INFO("MAIN", "all modules created");
  sensors->start();
  navigation->start();
  log_system.INFO("MAIN", "all module threads started");
  Thread::sleep(1000);
  log_system.INFO("MAIN", "After 1 sec sleep");

  Data& data = Data::getInstance();
  Sensors sens;
  StateMachine sm;
  while (sys.running_) {
    log_system.INFO("TST", "\n\n\n");

    const Navigation::FullOutput navs = navigation->getAllNavData();
    sens = data.getSensorsData();
    sm = data.getStateMachineData();

    switch (sm.current_state) {
      case State::kIdle :
        if (sens.module_status == ModuleStatus::kInit && *navs.status == ModuleStatus::kInit) {
          sm.current_state = State::kCalibrating;
          data.setStateMachineData(sm);
          log_system.INFO("TST", "Transitioned to Calibrating");
        }
      case State::kCalibrating :
        if (*navs.status == ModuleStatus::kReady) {
          sm.current_state = State::kAccelerating;
          data.setStateMachineData(sm);
          System::getSystem().navigation_motors_sync_.wait();
          log_system.INFO("TST", "Transitioned to Accelerating");
        }
      default :
        break;
    }

    auto imus = sens.imu.value;
    log_system.INFO("TST_SEN", "IMUs online: %d,%d,%d,%d", !!imus[0].operational,
                                                        !!imus[1].operational,
                                                        !!imus[2].operational,
                                                        !!imus[3].operational);
    for (auto& imu_data : sens.imu.value) {
      auto& acc = imu_data.acc;
      auto& gyr = imu_data.gyr;
      log_system.INFO("TST_SEN",
          "Accelerometer  (%8.4f %8.4f %8.4f)  Gyroscope  (%8.4f %8.4f %8.4f)",
          acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);
    }

#ifdef PROXI
    auto proxis_front = sens.proxi_front.value;
    log_system.INFO("TST_SEN", "Proxi-front online: %3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
        !!proxis_front[0].operational, !!proxis_front[1].operational,
        !!proxis_front[2].operational, !!proxis_front[3].operational,
        !!proxis_front[4].operational, !!proxis_front[5].operational,
        !!proxis_front[6].operational, !!proxis_front[7].operational);
    log_system.INFO("TST_SEN", "Proxi-front data:   %3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
        proxis_front[0].val, proxis_front[1].val, proxis_front[2].val, proxis_front[3].val,
        proxis_front[4].val, proxis_front[5].val, proxis_front[6].val, proxis_front[7].val);

    auto proxis_back = sens.proxi_back.value;
    log_system.INFO("TST_SEN", "Proxi-back online: %3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
        !!proxis_back[0].operational, !!proxis_back[1].operational,
        !!proxis_back[2].operational, !!proxis_back[3].operational,
        !!proxis_back[4].operational, !!proxis_back[5].operational,
        !!proxis_back[6].operational, !!proxis_back[7].operational);
    log_system.INFO("TST_SEN", "Proxi-back data:   %3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
        proxis_back[0].val, proxis_back[1].val, proxis_back[2].val, proxis_back[3].val,
        proxis_back[4].val, proxis_back[5].val, proxis_back[6].val, proxis_back[7].val);
#endif
    
    auto scs = sens.keyence_stripe_counter;
    log_system.INFO("TST_SEN", "stripe count 1 = %2d, stripe count 2 = %2d\n",
        scs[0].count.value, scs[1].count.value);


    log_system.INFO("TST_NAV", "status = %d, is_calibrating = %d",
        *navs.status, *navs.is_calibrating);
    log_system.INFO("TST_NAV",
        "After %7d grav samples, g=[(%8.4f %8.4f %8.4f), (%8.4f %8.4f %8.4f), "
        "(%8.4f %8.4f %8.4f), (%8.4f %8.4f %8.4f)]",
        *navs.num_gravity_samples,
        (*navs.g)[0][0], (*navs.g)[0][1], (*navs.g)[0][2],
        (*navs.g)[1][0], (*navs.g)[1][1], (*navs.g)[1][2],
        (*navs.g)[2][0], (*navs.g)[2][1], (*navs.g)[2][2],
        (*navs.g)[3][0], (*navs.g)[3][1], (*navs.g)[3][2]);
    log_system.INFO("TST_NAV",
        "After %7d gyro samples: gyro_offsets=[(%8.4f %8.4f %8.4f), (%8.4f %8.4f %8.4f), "
        "(%8.4f %8.4f %8.4f), (%8.4f %8.4f %8.4f)]",
        *navs.num_gyro_samples,
        (*navs.gyro_offsets)[0][0], (*navs.gyro_offsets)[0][1], (*navs.gyro_offsets)[0][2],
        (*navs.gyro_offsets)[1][0], (*navs.gyro_offsets)[1][1], (*navs.gyro_offsets)[1][2],
        (*navs.gyro_offsets)[2][0], (*navs.gyro_offsets)[2][1], (*navs.gyro_offsets)[2][2],
        (*navs.gyro_offsets)[3][0], (*navs.gyro_offsets)[3][1], (*navs.gyro_offsets)[3][2]);
    log_system.INFO("TST_NAV",
        "Outputs: a=(%8.4f %8.4f %8.4f), v=(%8.4f %8.4f %8.4f), d=(%10.4f %10.4f %10.4f)",
        (*navs.acceleration)[0], (*navs.acceleration)[1], (*navs.acceleration)[2],
            (*navs.velocity)[0],     (*navs.velocity)[1],     (*navs.velocity)[2],
        (*navs.displacement)[0], (*navs.displacement)[1], (*navs.displacement)[2]);
    log_system.INFO("TST_NAV", "         orientation=(%7.4f, %7.4f, %7.4f, %7.4f)",
        (*navs.orientation)[0], (*navs.orientation)[1],
        (*navs.orientation)[2], (*navs.orientation)[3]);
    log_system.INFO("TST_NAV", "stripe count = %2d, braking d = %8.2f, emergency braking d = %8.2f",
        *navs.stripe_count, navs.braking_dist, navs.em_braking_dist);
    Thread::sleep(500);
  }

  sensors->join();
  navigation->join();

  delete sensors;
  delete navigation;
  return 0;
}
