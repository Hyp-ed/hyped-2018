
/*
 * Authors: Yash Mittal and Ragnor Comerford
 * Organisation: HYPED
 * Date: 11. February 2018
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

#pragma once

#include "state_machine/event.hpp"
#include "state_machine/hyped-machine.hpp"

namespace hyped {
namespace state_machine {

class HypedMachine;

class State {
 public:
  virtual void react(HypedMachine &machine, Event event) = 0;
  virtual void entry() = 0;
};

class Idle : public State {
 public:
  virtual void react(HypedMachine &machine, Event event) override;
  virtual void entry() override;
};

class Accelerating : public State {
 public:
  virtual void react(HypedMachine &machine, Event event) override;
  virtual void entry() override;
};

class Decelerating : public State {
 public:
  virtual void react(HypedMachine &machine, Event event) override;
  virtual void entry() override;
};

class EmergencyBraking : public State {
 public:
  virtual void react(HypedMachine &machine, Event event) override;
  virtual void entry() override;
};

class RunComplete : public State {
 public:
  virtual void react(HypedMachine &machine, Event event) override;
  virtual void entry() override;
};

class FailureStopped : public State {
 public:
  virtual void react(HypedMachine &machine, Event event) override;
  virtual void entry() override;
};

class Exiting : public State {
 public:
  virtual void react(HypedMachine &machine, Event event) override;
  virtual void entry() override;
};

class Finished : public State {
 public:
  virtual void react(HypedMachine &machine, Event event) override;
  virtual void entry() override;
};

}  // namespace state_machine
}  // namespace hyped
