/*
 * ur_communication.h
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef UR_COMMUNICATION_H_
#define UR_COMMUNICATION_H_

#include <ur_modern_driver/robot_state.h>
#include <ur_modern_driver/do_output.h>
#include <ur_modern_driver/utils.h>
#include <ur_modern_driver/socket_com.h>

class UrCommunication
{
public:
	bool connected_;
	RobotState robot_state_;

	UrCommunication(ur_::Semaphore &msg_sem, std::string host);
	~UrCommunication();
	bool start();
	void halt();

private:
	int pri_sockfd_, sec_sockfd_;
	bool keepalive_;

	std::string host_;
	int sec_port_;
	int pri_port_;

	std::thread comThread_;
	int flag_;
	void run();
	void initSec();
	void initPri();

  ur_::Semaphore *msg_sem_ptr;
};

#endif /* UR_COMMUNICATION_H_ */
