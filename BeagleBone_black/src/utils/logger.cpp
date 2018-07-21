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
  fprintf(file, "\n");
}

// static auto start_time = std::chrono::high_resolution_clock::now();

void logHead(FILE* file, const char* title, const char* module)
{
  using namespace std::chrono;
  std::time_t t = std::time(nullptr);
  tm* tt = localtime(&t);
  fprintf(file, "%02d:%02d:%02d",
      tt->tm_hour, tt->tm_min, tt->tm_sec);

  static const bool print_micro = true;
  if (print_micro) {
    auto now_time = high_resolution_clock::now().time_since_epoch();
    duration<int, std::milli> time_span = duration_cast<std::chrono::milliseconds>
          (now_time);
    fprintf(file, ".%03d ", static_cast<uint16_t>(time_span.count()) % 1000);
  } else {
    fprintf(file, " ");
  }
  fprintf(file, "%s[%s]: ", title, module);
}

}

Logger::Logger(bool verbose, int8_t debug)
    : verbose_(verbose),
      debug_(debug)
{ /* EMPTY */ }

void Logger::ERR(const char* module, const char* format, ...)
{
  static FILE* file = stdout;
  ScopedLock L(&logger_lock);
  logHead(file, "ERR", module);
  va_list args;
  va_start(args, format);
  myPrint(file, format, args);
  va_end(args);
}

void Logger::INFO(const char* module, const char* format, ...)
{
  static FILE* file = stdout;
  if (verbose_) {
    ScopedLock L(&logger_lock);
    logHead(file, "INFO", module);
    va_list args;
    va_start(args, format);
    myPrint(file, format, args);
    va_end(args);
  }
}

void Logger::DBG(const char* module, const char* format, ...)
{
  static FILE* file = stderr;
  if (debug_ >= 0) {
    ScopedLock L(&logger_lock);
    logHead(file, "DBG0", module);
    va_list args;
    va_start(args, format);
    myPrint(file, format, args);
    va_end(args);
  }
}

void Logger::DBG1(const char* module, const char* format, ...)
{
  static FILE* file = stderr;
  if (debug_ >= 1) {
    ScopedLock L(&logger_lock);
    logHead(file, "DBG1", module);
    va_list args;
    va_start(args, format);
    myPrint(file, format, args);
    va_end(args);
  }
}
void Logger::DBG2(const char* module, const char* format, ...)
{
  static FILE* file = stderr;
  if (debug_ >= 2) {
    ScopedLock L(&logger_lock);
    logHead(file, "DBG2", module);
    va_list args;
    va_start(args, format);
    myPrint(file, format, args);
    va_end(args);
  }
}
void Logger::DBG3(const char* module, const char* format, ...)
{
  static FILE* file = stderr;
  if (debug_ >= 3) {
    ScopedLock L(&logger_lock);
    logHead(file, "DBG3", module);
    va_list args;
    va_start(args, format);
    myPrint(file, format, args);
    va_end(args);
  }
}
}}  // namespace hyped::utils

