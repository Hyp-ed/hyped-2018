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

#include "utils/logger.hpp"

#include <stdio.h>
#include <stdarg.h>

#include <chrono>
#include <iomanip>
#include <ctime>

#include "utils/concurrent/lock.hpp"

namespace hyped {
namespace utils {

using concurrent::Lock;
using concurrent::ScopedLock;

namespace {
Lock logger_lock;

void myPrint(FILE* file, const char* format, va_list args)
{
  vfprintf(file, format, args);
}

static auto start_time = std::chrono::high_resolution_clock::now();

void logHead(FILE* file, const char* title)
{
  using namespace std::chrono;
  auto now_time = high_resolution_clock::now();
  duration<double, std::milli> time_span = now_time - start_time;
  // std::time_t t = std::time(nullptr);
  // tm* tt = localtime(&t);
  fprintf(file, "%f ", time_span.count());
  fprintf(file, title);
}

}

Logger::Logger(bool verbose, int8_t debug)
    : verbose_(verbose)
    , debug_(debug)
{ /* EMPTY */ }

void Logger::INFO(const char* format, ...)
{
  if (verbose_) {
    ScopedLock L(&logger_lock);
    logHead(stdout, "INFO");
    va_list args;
    va_start(args, format);
    myPrint(stdout, format, args);
    va_end(args);
  }
}

void Logger::DBG(const char* format, ...)
{
  if (debug_ >= 0) {
    va_list args;
    va_start(args, format);
    myPrint(stderr, format, args);
    va_end(args);
  }
}

void Logger::DBG0(const char* format, ...)
{
  if (debug_ >= 0) {
    va_list args;
    va_start(args, format);
    myPrint(stderr, format, args);
    va_end(args);
  }
}
void Logger::DBG1(const char* format, ...)
{
  if (debug_ >= 1) {
    va_list args;
    va_start(args, format);
    myPrint(stderr, format, args);
    va_end(args);
  }
}
void Logger::DBG2(const char* format, ...)
{
  if (debug_ >= 2) {
    va_list args;
    va_start(args, format);
    myPrint(stderr, format, args);
    va_end(args);
  }
}
void Logger::DBG3(const char* format, ...)
{
  if (debug_ >= 3) {
    va_list args;
    va_start(args, format);
    myPrint(stderr, format, args);
    va_end(args);
  }
}
}}  // namespace hyped::utils

