#ifndef ROBOTIQ_3F_GRIPPER_MODBUS_TCP_CLIENT_H
#define ROBOTIQ_3F_GRIPPER_MODBUS_TCP_CLIENT_H

#include <mutex>

#include <robotiq_3f_gripper_control/robotiq_3f_gripper_client_base.h>
#include <robotiq_3f_gripper_control/modbus.h>


namespace robotiq_3f_gripper_control
{


class Robotiq3FGripperModbusTCPClient : public Robotiq3FGripperClientBase
{
public:

    Robotiq3FGripperModbusTCPClient(const std::string& host, const int& port);
    ~Robotiq3FGripperModbusTCPClient();

  /**
   * \brief Write the given set of control flags to the memory of the gripper
   *
   * @param[in] output The set of output-register values to write to the gripper
   */
  void writeOutputs(const GripperOutput& output);

  /**
   * \brief Reads set of input-register values from the gripper.
   * \return The gripper input registers as read from the controller IOMap
   */
  GripperInput readInputs();

  /**
   * \brief Reads set of output-register values from the gripper.
   * \return The gripper output registers as read from the controller IOMap
   */
  GripperOutput readOutputs() const;

private:
  modbus manager_;
  std::mutex mutex_;
};

}

#endif
