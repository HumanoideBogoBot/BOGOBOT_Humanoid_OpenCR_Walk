#include <DynamixelSDK.h>
// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

#define BAUDRATE                        2000000
#define DEVICENAME                      "OpenCR_DXL_Port"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define ESC_ASCII_VALUE                 0x1b0x1b
void readPoseAll(double PresentPose[]) { // Lee la posición actual de todos los 18 servos Dynamixel y lo escribe en el arreglo PresentPose[]
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler("OpenCR_DXL_Port");
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_value;
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
    
  for (int dxl_id = 1; dxl_id <= 18; ++dxl_id) {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_value, &dxl_error);
    //delay(5);// Este delay es MUY importante para darle tiempo a los servomotores de enviar sus Status Packages de regreso a la OpenCR
    PresentPose[dxl_id] = float(dxl_value);
  }
  portHandler->closePort();
}
bool movRobot(double PresentPose[], double Pose[], double dxl_goal_position[], float tf, double tmp[]){
    float startMillis;
    float currentMillis;
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
    dynamixel::GroupSyncWrite groupSyncWriteVel(portHandler, packetHandler, 32, LEN_MX_GOAL_POSITION);
    double dxl_goal_position2[20];
    uint8_t param_goal_position[2];
    uint8_t param_goal_vel[2];
    bool dxl_addparam_result = false;
    int dxl_comm_result = COMM_TX_FAIL;
    //portHandler->openPort();
    //portHandler->setBaudRate(BAUDRATE);
    IK_Feet(Pose, dxl_goal_position);
    IK_arms(Pose, dxl_goal_position2);
    startMillis = millis()/1000.0;
    float t_ac = 0.0;
    float dt = 0.0;
    double q;
    while(t_ac <= tf){
      currentMillis = millis()/1000.0;
      dt = currentMillis - startMillis;
      t_ac = t_ac + dt;
      float q;
      for(int i = 1; i <= 6; i++){
        q = interpolation(t_ac ,tf,PresentPose[i],dxl_goal_position[i]);
        param_goal_position[0] = DXL_LOBYTE(q);
        param_goal_position[1] = DXL_HIBYTE(q);
        float vel = abs((dxl_goal_position[i]-PresentPose[i])/tf);
        param_goal_vel[0] = DXL_LOBYTE(vel);
        param_goal_vel[1] = DXL_HIBYTE(vel);
        dxl_addparam_result = groupSyncWriteVel.addParam(i, param_goal_position);
        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);//
      }
      for(int i = 7; i <= 12; i++){
        q = interpolation(t_ac ,tf,PresentPose[i],dxl_goal_position[i]);
        //float val = millis()/1000.0;
        //Serial.println(q);
        param_goal_position[0] = DXL_LOBYTE(q);
        param_goal_position[1] = DXL_HIBYTE(q);
        float vel = abs((dxl_goal_position[i]-PresentPose[i])/tf);
        param_goal_vel[0] = DXL_LOBYTE(vel);
        param_goal_vel[1] = DXL_HIBYTE(vel);
        dxl_addparam_result = groupSyncWriteVel.addParam(i, param_goal_position);
        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);//
      }
      for(int i = 13; i <= 15; i++){
        q = interpolation(t_ac ,tf,PresentPose[i],dxl_goal_position2[i]);
        //float val = millis()/1000.0;
        //Serial.println(q);
        param_goal_position[0] = DXL_LOBYTE(q);
        param_goal_position[1] = DXL_HIBYTE(q);
        float vel = abs((dxl_goal_position[i]-PresentPose[i])/tf);
        param_goal_vel[0] = DXL_LOBYTE(vel);
        param_goal_vel[1] = DXL_HIBYTE(vel);
        dxl_addparam_result = groupSyncWriteVel.addParam(i, param_goal_position);
        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);//
      }
      for(int i = 16; i <= 18; i++){
        q = interpolation(t_ac ,tf,PresentPose[i],dxl_goal_position2[i]);
        //float val = millis()/1000.0;
        //Serial.println(q);
        param_goal_position[0] = DXL_LOBYTE(q);
        param_goal_position[1] = DXL_HIBYTE(q);
        float vel = abs((dxl_goal_position[i]-PresentPose[i])/tf);
        param_goal_vel[0] = DXL_LOBYTE(vel);
        param_goal_vel[1] = DXL_HIBYTE(vel);
        dxl_addparam_result = groupSyncWriteVel.addParam(i, param_goal_position);
        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);//
      }
      dxl_comm_result = groupSyncWrite.txPacket();
      dxl_comm_result = groupSyncWriteVel.txPacket();
      groupSyncWrite.clearParam();
  
      startMillis = currentMillis;  
    }

    for (int i = 1; i <= 12; i++) {
      tmp[i] = dxl_goal_position[i];  
    }

    for (int i = 13; i <= 18; i++) {
      tmp[i] = dxl_goal_position2[i];
    }

    //portHandler->closePort();
    return true;
  
}
