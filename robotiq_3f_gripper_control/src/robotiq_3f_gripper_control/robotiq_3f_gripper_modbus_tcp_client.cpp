#include <robotiq_3f_gripper_control/robotiq_3f_gripper_modbus_tcp_client.h>


namespace robotiq_3f_gripper_control
{

Robotiq3FGripperModbusTCPClient::Robotiq3FGripperModbusTCPClient(const std::string& host, const int& port): manager_(host, port)
{
    ROS_INFO_STREAM("MODBUS - Connecting to " << host << ":" << port);
    if (!manager_.modbus_connect()){
        ROS_ERROR("Failed to connect to modbus server");
    }
    ROS_INFO("MODBUS - Connected");

}


Robotiq3FGripperModbusTCPClient::~Robotiq3FGripperModbusTCPClient()
{
    manager_.modbus_close();
}


void Robotiq3FGripperModbusTCPClient::writeOutputs(const Robotiq3FGripperModbusTCPClient::GripperOutput& output){

    // Lock mutex
    std::lock_guard<std::mutex> lock(mutex_);
    
    // ROS_INFO_STREAM("writeOutputs: " << std::this_thread::get_id()  << " with obj " << this);
    uint8_t map[16] = {0}; // array containing all 15 output registers
    // ROS_INFO_STREAM("Activaiton bit " << (int)(output.rACT));
    // Pack the Action Request register byte
    map[0] = (output.rACT & 0x1) | (output.rMOD << 0x1) & 0x6 | ((output.rGTO << 0x3) & 0x8) | ((output.rATR << 0x4) & 0x10);

    // Pack the Gripper Options register byte
    map[1] =  ((output.rICF << 0x2) & 0x4) | ((output.rICS << 0x3) & 0x8);

    // map[2] is empty

    // Requested Position, Speed and Force (Finger A).
    map[3]  = output.rPRA;
    map[4]  = output.rSPA;
    map[5]  = output.rFRA;

    // Finger B
    map[6]  = output.rPRB;
    map[7]  = output.rSPB;
    map[8]  = output.rFRB;

    // Finger C
    map[9]  = output.rPRC;
    map[10] = output.rSPC;
    map[11] = output.rFRC;

    // Scissor Mode
    map[12] = output.rPRS;
    map[13] = output.rSPS;
    map[14] = output.rFRS;

    uint16_t buffer[8];
    for (unsigned i = 0; i < 8; ++i)
    {
        // Each register is 2 bytes, so we need to pack two bytes into each register
        buffer[i] = (map[i * 2] << 8) | map[i * 2 + 1];
    }


    int res = manager_.modbus_write_registers(0, 8 , buffer);
    if (res != 0){
        ROS_ERROR_STREAM("Failed to write to output registers; code: " << res);
    }
    // ROS_INFO("MODBUS - Wrote to register %d; res is %d", i, res);
}


Robotiq3FGripperModbusTCPClient::GripperInput Robotiq3FGripperModbusTCPClient::readInputs()
{
    // ROS print thread ID
    // ROS_INFO_STREAM("readInputs: " << std::this_thread::get_id() << " with obj " << this);

    // Lock mutex
    std::lock_guard<std::mutex> lock(mutex_);

    uint16_t buffer[8];
    int res = manager_.modbus_read_input_registers(0, 8, buffer);
    if (res != 0){
        ROS_ERROR_STREAM("Failed to read input registers; code: " << res);
    }

    uint8_t map[16];
    for (unsigned i = 0; i < 8; ++i)
    {        
        // Each register is 2 bytes, so we need to unpack two bytes from each register
        map[i * 2] = (buffer[0] & 0xFF00) >> 8;
        map[i * 2 + 1] = buffer[0] & 0x00FF;
    }

    // numRegs = int(ceil(numBytes/2.0))

    // #To do!: Implement try/except
    // #Get status from the device
    // with self.lock:
    //     response = self.client.read_input_registers(0, numRegs)

    // #Instantiate output as an empty list
    // output = []

    // #Fill the output with the bytes in the appropriate order
    // for i in range(0, numRegs):
    //     output.append((response.getRegister(i) & 0xFF00) >> 8)
    //     output.append( response.getRegister(i) & 0x00FF)


    // message.gACT = (status[0] >> 0) & 0x01;
    //     message.gMOD = (status[0] >> 1) & 0x03;
    //     message.gGTO = (status[0] >> 3) & 0x01;
    //     message.gIMC = (status[0] >> 4) & 0x03;
    //     message.gSTA = (status[0] >> 6) & 0x03;


    // Decode Input Registers
    Robotiq3FGripperModbusTCPClient::GripperInput input;

    // Gripper Status
    input.gACT = (map[0] >> 0)    & 0x01;
    input.gMOD = (map[0] >> 1) & 0x03;
    input.gGTO = (map[0] >> 3) & 0x01;
    input.gIMC = (map[0] >> 4) & 0x03;
    input.gSTA = (map[0] >> 6) & 0x03;


    //     message.gDTA = (status[1] >> 0) & 0x03;
    //     message.gDTB = (status[1] >> 2) & 0x03;
    //     message.gDTC = (status[1] >> 4) & 0x03;
    //     message.gDTS = (status[1] >> 6) & 0x03;

    // Object Status
    input.gDTA = (map[1] >> 0) & 0x03;
    input.gDTB = (map[1] >> 2) & 0x03;
    input.gDTC = (map[1] >> 4) & 0x03;
    input.gDTS = (map[1] >> 6) & 0x03;


    //     message.gFLT = status[2]

    // Fault Status
    input.gFLT = map[2] & 0xF;


    // Requested Position, Speed and Force (Finger A).
    input.gPRA = map[3];
    input.gPOA = map[4];
    input.gCUA = map[5];

    // Finger B
    input.gPRB = map[6];
    input.gPOB = map[7];
    input.gCUB = map[8];

    // Finger C
    input.gPRC = map[9];
    input.gPOC = map[10];
    input.gCUC = map[11];

    // Scissor Mode
    input.gPRS = map[12];
    input.gPOS = map[13];
    input.gCUS = map[14];

    return input;
}

// I don't think this is used anywhere
Robotiq3FGripperModbusTCPClient::GripperOutput Robotiq3FGripperModbusTCPClient::readOutputs() const
{
    ROS_INFO("MODBUS - Reading outputs");
    return Robotiq3FGripperModbusTCPClient::GripperOutput();
}

}
