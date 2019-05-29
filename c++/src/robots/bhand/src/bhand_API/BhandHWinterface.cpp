#include <bhand/bhand_API/BhandHWInterface.h>

#include <iostream>

BhandHWInterface::BhandHWInterface()
{
	joint_limit_ticks.push_back(std::pair<int, int>(0, 35939));  // spread
  joint_limit_ticks.push_back(std::pair<int, int>(0, 195100)); // finger 1
  joint_limit_ticks.push_back(std::pair<int, int>(0, 195250)); // finger 2
  joint_limit_ticks.push_back(std::pair<int, int>(0, 194900)); // finger 3

  joint_limit.push_back(std::pair<double, double>(0.0, 3.14)); // spread
  joint_limit.push_back(std::pair<double, double>(0.0, 2.44)); // finger 1
  joint_limit.push_back(std::pair<double, double>(0.0, 2.44)); // finger 2
  joint_limit.push_back(std::pair<double, double>(0.0, 2.44)); // finger 3

	sg_dead_zone[0] = 6;
	sg_dead_zone[1] = 6;
	sg_dead_zone[2] = 6;
	sg_dead_zone[3] = 6;

	mode = IDLE;
}

BhandHWInterface::~BhandHWInterface()
{

}

// Initializes the Barrett Hand.
void BhandHWInterface::initialize(const std::string &handType, bool rt_control_enable)
{
	int err; // Return value (error) of all BHand calls
	std::string buf; // buffer for the error message

	// Set hardware description before initialization
	// int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-262");
	// int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-280");
	this->hand_type = handType;
	int hwIndex = BHandHardware::getBHandHardwareIndex(hand_type.c_str());
	if (hwIndex < 0)
  {
		throw std::runtime_error("\n\nThe API has not been compiled to include target hand.\n");
	}

	setHardwareDesc(hwIndex);
	//bool use280Config = (strcmp(getHardwareDesc()->getModelNumber(), "BH8-280") == 0);
	//printf("\nuse280Config = %d\n", use280Config);
	if ((err = handInitWithMenu(((BHand *)this)))){
		//this->printErrorMessage(err);
		// return;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
	//if (err = InitSoftware(com_port, THREAD_PRIORITY_TIME_CRITICAL))
	//	Error();

	//printf("Initialization...");
	if ((err = InitHand("123S"))){
		//this->printErrorMessage(err);
		//return;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
	//printf(" Done\n");

	initialized = true;

	if (rt_control_enable) enableRTControl();

	initSgOffsets();
}

void BhandHWInterface::initSgOffsets()
{
	int N_msr = 30;

	for (int i=0; i<4; i++) sg_offset[i] = 0;

	for (int i=0; i<N_msr; i++)
	{
		RTUpdate();
		sg_offset[0] += RTGetStrain('S');
		sg_offset[1] += RTGetStrain('0' + 1);
		sg_offset[2] += RTGetStrain('0' + 2);
		sg_offset[3] += RTGetStrain('0' + 3);
	}

	for (int i=0; i<4; i++) sg_offset[i] = (sg_offset[i] + 0.5) / N_msr;
}

void BhandHWInterface::setMode(const BhandHWInterface::Mode &m)
{
  //if (mode == m) return;

	stop();

  switch (m)
  {
    case BhandHWInterface::IDLE:
      set_param("123S", "MODE", 0);
    case BhandHWInterface::JOINT_VEL_CONTROL:
      set_param("123S", "TSTOP", 0);
      set_param("123S", "HSG", 10000);
      set_param("123S", "MODE", 4);
  }

  mode = m;
}

void BhandHWInterface::stop()
{
	set_param("123S", "TSTOP", 0);
	set_param("123S", "HSG", 10000);
	set_param("123S", "MODE", 4);
	for (int i=0;i<4;i++) setJointVelocity(0.0, i);
}

int BhandHWInterface::command(const char *send, char *receive)
{
	return Command(send, receive);
}

// Stops the Barrett Hand and shuts down the motor.
void BhandHWInterface::terminate()
{
	RTAbort();
	// StopMotor("123S");
	set_param("123S", "MODE", 0);
	terminate();
	initialized = RT_control_enabled = false;
}

void BhandHWInterface::printErrorMessage(std::string &buf, int err_id) const
{
	const int buff_size = 100;
	char temp_buff[buff_size];
	snprintf(temp_buff, buff_size, "ERROR: %d\n%s\n", err_id, ErrorMessage(err_id));

	buf = temp_buff;
}

void BhandHWInterface::printErrorMessage(int err_id) const
{
	std::string buf;
	printErrorMessage(buf, err_id);
	std::cerr << buf;
}

void BhandHWInterface::enableRTControl()
{
	int err; // Return value (error) of all BHand calls

	char motor[] = "123S";

	control_velocity_flag                = true;   // LCV   Loop Control Velocity Flag
	control_propgain_flag                = false;  // LCPG  Loop Control Proportional Gain Flag
	control_torque_flag                  = false;  // LCT   Loop Control Torque Flag
	feedback_velocity_flag               = true;   // LFV   Loop Feedback Velocity Flag
	feedback_strain_flag                 = true;   // LFS   Loop Feedback Stain Flag
	feedback_position_flag               = true;   // LFAP  Loop Feedback Absolute Position Flag
	feedback_deltapos_flag               = false;  // LFDP  Loop Feedback Delta Position Flag
	feedback_breakaway_position_flag     = false;  // LFBP  Loop Feedback Breakaway Position Flag
	feedback_analog_input_flag           = false;  // LFAIN Loop Feedback Analog Input Flag
	feedback_delta_position_discard_flag = false;  // LFDPD Loop Feedback Delta Position Discard Flag
	feedback_temperature                 = true;   // LFT   Loop Feedback Temperature Flag

	control_position_flag                = false;

	control_velocity_coefficient        = 3;  // LCVC  Loop Control Velocity Coefficient
	feedback_velocity_coefficient       = 1;  // LFVC  Loop Feedback Velocity Coefficient
	feedback_delta_position_coefficient = 1;  // LFDPC Loop Feedback Delta Position Coefficient

	if ((err = RTSetFlags(motor,
                control_velocity_flag, control_velocity_coefficient, control_propgain_flag,
                control_torque_flag, feedback_velocity_flag, feedback_velocity_coefficient,
                feedback_strain_flag, feedback_position_flag, feedback_deltapos_flag,
                feedback_delta_position_coefficient, feedback_breakaway_position_flag,
                feedback_analog_input_flag, feedback_delta_position_discard_flag,
                feedback_temperature ))){
					std::string buf;
					printErrorMessage(buf, err);
					throw std::runtime_error(buf);
					// this->printErrorMessage(err);
	}

	//RTStart( "123S" , BHMotorTSTOPProtect);
	RTStart( "123S" , BHMotorTorqueLimitProtect);
	RTUpdate();

	RT_control_enabled = true;
}

bool BhandHWInterface::isInitialized() const
{
	return initialized;
}

bool BhandHWInterface::isRTControlEnabled() const
{
	return RT_control_enabled;
}

void BhandHWInterface::set_param(const char *motor, const char *propertyName, int value)
{
	int err = Set(motor, propertyName, value);

	if (err){
		if (err == -1){
		  printErrorMessage(err);
		  return;
		}
		std::string buf;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
}

void BhandHWInterface::get_param(const char *motor, const char *propertyName, int *result)
{
	int err = Get(motor, propertyName, result);

	if (err){
		if (err == -1){
		  printErrorMessage(err);
		  return;
		}
		std::string buf;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
}

double BhandHWInterface::ticks2rad(int i, int ticks) const
{
  return joint_limit[i].first + (ticks - joint_limit_ticks[i].first)*(joint_limit[i].second - joint_limit[i].first)/(double)(joint_limit_ticks[i].second - joint_limit_ticks[i].first);
}

int BhandHWInterface::radPerSec2ticksPerMs(int i, double rad_per_sec) const
{
	double ticks_range = static_cast<double>(joint_limit_ticks[i].second - joint_limit_ticks[i].first);
  double rad_range = static_cast<double>(joint_limit[i].second - joint_limit[i].first);
  return (rad_per_sec * ticks_range / rad_range) / 1000.0;
}

int BhandHWInterface::rad2ticks(int i, double rads) const
{
  return 0.5 + joint_limit_ticks[i].first + (rads - joint_limit[i].first)*(joint_limit_ticks[i].second - joint_limit_ticks[i].first)/(joint_limit[i].second - joint_limit[i].first);
}

double BhandHWInterface::sg2Nm(int sg_value) const
{
  double max_tip_torque = 40 * 0.06;  // 4kg of max tip force with 6cm distal link size
  double min_tip_torque = -40 * 0.06;  // -4kg of min tip force with 6cm distal link size
  double min_sg = -90;
  double max_sg = 90;

  double sg_perc = (static_cast<double>(sg_value) - min_sg) / (max_sg - min_sg);
  return sg_perc * (max_tip_torque - min_tip_torque) + min_tip_torque ;

	// double p1 = 2.754e-10;
  // double p2 = -1.708e-06;
  // double p3 = 0.003764;
  // double p4 = -2.85;
	//
	// return p1*std::pow(sg_value,3) + p2*std::pow(sg_value,2) + p3*sg_value + p4;
}

double BhandHWInterface::getJointPosition(int i)
{
  // Check if the feedback position flag is set
  // if (!feedback_position_flag)
  //   throw std::runtime_error("Error: BarrettHand: getFingerPosition: The loop feedback absolute position (LFAP) flag must be set to receive absolute position feedback.");

	int pos_ticks;

  if (i == 0) pos_ticks = RTGetPosition('4'); // spread joint
	else pos_ticks = RTGetPosition(i + '0');

	return ticks2rad(i, pos_ticks);
}

void BhandHWInterface::setJointVelocity(double vel, int i)
{
  int err;
  // Check if the feedback position flag is set
  // if (!control_velocity_flag)
	// 	throw std::runtime_error("Error: BarrettHand: setFingerVelocity: The loop control velocity (LCV) flag must be set to send velocity references to the hand.");

	int tick_vel = radPerSec2ticksPerMs(i,vel);

  if (i == 0) err = RTSetVelocity('4', tick_vel);
  else  err = RTSetVelocity(i + '0', tick_vel);

  //if (err) errorHandler(err);
}

double BhandHWInterface::getJointTorque(int i)
{
  // Check if the feedback position flag is set
  // if (!feedback_strain_flag)
  //   throw std::runtime_error("Error: BarrettHand: getFingerForce: The loop feedback strain (LFS) flag must be set to receive strain gauge feedback.");

	int sg_value;
  if (i == 0) sg_value = RTGetStrain('S');
  else sg_value = RTGetStrain(i + '0');
	sg_value -= sg_offset[i];

	int sg_sign = 2*(sg_value >= 0) - 1; // get sign
	int sg2 = sg_value - sg_sign*sg_dead_zone[i]; // subtract/add acoording to sign
	if (sg2*sg_sign < 0) sg2 = 0; // if sign was inverted set to zero

	// std::cerr << "sg_value = " << sg2 << "\n";

	return sg2Nm(sg2);
}
