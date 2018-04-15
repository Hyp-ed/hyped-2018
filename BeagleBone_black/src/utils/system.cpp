/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 2. April 2018
 * Description:
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

#include "utils/system.hpp"

#include <stdlib.h>
// #include <unistd.h>
#include <stdio.h>
#include <getopt.h>

#define DEFAULT_VERBOSE false
#define DEFAULT_DEBUG   -1

namespace hyped {
namespace utils {

namespace {
void printUsage()
{
  printf("./hyped [args]\n");
}
}

System::~System()
{
  if (log_) delete log_;
}

System::System(int argc, char* argv[])
    : verbose(DEFAULT_VERBOSE)
    , verbose_motor(DEFAULT_VERBOSE)
    , verbose_nav(DEFAULT_VERBOSE)
    , verbose_sensor(DEFAULT_VERBOSE)
    , verbose_state(DEFAULT_VERBOSE)
    , verbose_cmn(DEFAULT_VERBOSE)
    , debug(DEFAULT_DEBUG)
    , debug_motor(DEFAULT_DEBUG)
    , debug_nav(DEFAULT_DEBUG)
    , debug_sensor(DEFAULT_DEBUG)
    , debug_state(DEFAULT_DEBUG)
    , debug_cmn(DEFAULT_DEBUG)
{
  int c;
  int option_index = 0;
  while (1) {
    static option long_options[] = {
      {"verbose", no_argument, 0, 'v'},
      {"verbose_motor", no_argument, 0, 'a'},
      {"verbose_nav", no_argument, 0, 'A'},
      {"verbose_sensor", no_argument, 0, 'b'},
      {"verbose_state", no_argument, 0, 'B'},
      {"verbose_cmn", no_argument, 0, 'c'},
      {"debug", optional_argument, 0, 'd'},
      {"debug_motor", optional_argument, 0, 'e'},
      {"debug_nav", optional_argument, 0, 'E'},
      {"debug_sensor", optional_argument, 0, 'f'},
      {"debug_state", optional_argument, 0, 'F'},
      {"debug_cmn", optional_argument, 0, 'g'},
      {"help", no_argument, 0, 'h'},
      {0, 0, 0, 0}
    };
    c = getopt_long(argc, argv, "vd::h", long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {
      case 'v':
        verbose = true;
        break;
      case 'h':
        printUsage();
        exit(0);
        break;
      case 'd':
        if (optarg) {
          debug = atoi(optarg);
        } else {
          debug = 0;
        }
        break;
      case 'a':   // verbose_motor
        verbose_motor = true;
        break;
      case 'A':   // verbose_nav
        verbose_nav = true;
        break;
      case 'b':   // verbose_sensor
        verbose_sensor = true;
        break;
      case 'B':   // verbose_state
        verbose_state = true;
        break;
      case 'c':   // verbose_cmn
        verbose_cmn = true;
      case 'e':   // debug_motor
        if (optarg) debug_motor = atoi(optarg);
        else        debug_motor = 0;
        break;
      case 'E':   // debug_nav
        if (optarg) debug_nav = atoi(optarg);
        else        debug_nav = 0;
        break;
      case 'f':   // debug_sensor
        if (optarg) debug_sensor = atoi(optarg);
        else        debug_sensor = 0;
        break;
      case 'F':   // debug_state
        if (optarg) debug_state = atoi(optarg);
        else        debug_state = 0;
        break;
      case 'g':   // debug_cmn
        if (optarg) debug_cmn = atoi(optarg);
        else        debug_cmn = 0;
        break;
      default:
        printUsage();
        exit(1);
        break;
    }

    // propagate verbose and debug to modules if not set module-specific
    if (verbose_motor == DEFAULT_VERBOSE) verbose_motor = verbose;
    if (verbose_nav   == DEFAULT_VERBOSE) verbose_nav = verbose;
    if (verbose_sensor == DEFAULT_VERBOSE) verbose_sensor = verbose;
    if (verbose_state == DEFAULT_VERBOSE) verbose_state = verbose;
    if (verbose_cmn == DEFAULT_VERBOSE) verbose_cmn = verbose;

    if (debug_motor == DEFAULT_DEBUG) debug_motor = debug;
    if (debug_nav   == DEFAULT_DEBUG) debug_nav = debug;
    if (debug_sensor == DEFAULT_DEBUG) debug_sensor = debug;
    if (debug_state == DEFAULT_DEBUG) debug_state = debug;
    if (debug_cmn == DEFAULT_DEBUG) debug_cmn = debug;
  }

  log_ = new Logger(verbose, debug);
  system_ = this;
}

System* System::system_ = 0;

void System::parseArgs(int argc, char* argv[])
{
  if (system_) delete system_;
  system_ = new System(argc, argv);
}

System& System::getSystem()
{
  if (system_) return *system_;
  Logger log;
  log.ERR("SYSTEM", "somebody tried to get System"
          " before initialisation, aborting\n");
  exit(1);
}

Logger& System::getLogger()
{
  System& sys = getSystem();
  return *sys.log_;
}

}}  // namespace hyped::utils
