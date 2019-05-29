#ifndef BHAND_HARDWARE_INTERFACE_H
#define BHAND_HARDWARE_INTERFACE_H

#include "BHand.h"
#include "BHandAppHelper.h"
#include <bhand/utils.h>

class BhandHWInterface: public BHand
{
public:
  enum Mode
  {
    IDLE,
    JOINT_VEL_CONTROL
  };

  BhandHWInterface();
  ~BhandHWInterface();

  void initialize(const std::string &handType="BH8-280", bool rt_control_enable=true);

  void terminate();

  bool isInitialized() const;
  bool isRTControlEnabled() const;

  void enableRTControl();

  void printErrorMessage(int err_id) const;
  void printErrorMessage(std::string &buf, int err_id) const;

  int command(const char *send, char *receive = 0);
  void set_param(const char *motor, const char *propertyName, int value);
  void get_param(const char *motor, const char *propertyName, int *result);

  double getJointPosition(int i);
  double getJointTorque(int i);
  void setJointVelocity(double vel, int i);

  void setMode(const BhandHWInterface::Mode &m);

  void stop();

private:

  void initSgOffsets();

  int sg_offset[4];
  int sg_dead_zone[4];

  bool initialized;
  bool RT_control_enabled;

  std::string hand_type;

	bool control_velocity_flag ;	            // LCV: Loop Control Velocity Flag
	bool control_propgain_flag;	                // LCPG: Loop Control Proportional Gain Flag
	bool control_torque_flag;	                // LCT: Loop Control Torque Flag
	bool control_position_flag;					// LCP: Loop Control Position Flag
	bool feedback_velocity_flag;	            // LFV: Loop Feedback Velocity Flag
	bool feedback_strain_flag;	                // LFS: Loop Feedback Stain Flag
	bool feedback_position_flag;	            // LFAP: Loop Feedback Absolute Position Flag
	bool feedback_deltapos_flag;	            // LFDP: Loop Feedback Delta Position Flag
	bool feedback_breakaway_position_flag;	    // LFBP: Loop Feedback Breakaway Position Flag
	bool feedback_analog_input_flag;	        // LFAIN: Loop Feedback Analog Input Flag
	bool feedback_delta_position_discard_flag;	// LFDPD: Loop Feedback Delta Position Discard Flag
	bool feedback_temperature;	                // LFT: Loop Feedback Temperature Flag

	int control_velocity_coefficient;           // LCVC: Loop Control Velocity Coefficient
	int feedback_velocity_coefficient;          // LFVC: Loop Feedback Velocity Coefficient
	int feedback_delta_position_coefficient;    // LFDPC: Loop Feedback Delta Position Coefficient

  std::vector<std::pair<double, double>> joint_limit;
  std::vector<std::pair<int, int>> joint_limit_ticks;

  Mode mode;

  double ticks2rad(int i, int ticks) const;
  int rad2ticks(int i, double rads) const;
  int radPerSec2ticksPerMs(int i, double rad_per_sec) const;
  double sg2Nm(int sg_value) const;
};

#endif // BHAND_HARDWARE_INTERFACE_H
