/**
 * @file   tachymeter_geocom.cpp - main method.h
                    mySerial.cpp - build serial structure to communicate with ttyUSB0 
                    mySerial.h   - header 
                order_geocom.h   - geoCom interface commands as hardcoded strings
 * @Author Finn Linzer, 4. Master Semester HafenCity University (finn.linzer@gmail.com)
 * @date   August, 2017
 * @brief  Settup a geocom tachymeter to start tracking a prism
           and send acquired info as geometry_msgs::PointStamped.
           This could be use to set x,y,z positions for frames, etc.

 * Tachymeter Data are measured continously by a modern Tachymeter. The data are distributed over a ros NodeHandle (ex. /tachy_points). These Data are already converted to a X,Y,Z Coordinate + Timestamp. This data is brought to you via a geometry_msgs::PointStamped object.

To have the correct pose in ros, we merge these information to a full functional TF2-Frame. The correct advertisement of this information is the main goal of the following code.

The startup and setup process of a tachymeter, to track a prism, is still not very confortable but possible
While tracking the prism should not be moved to fast.
To provide the system with a best case PointStamped is the main goal of this code.
 */

/**
 * @param [in] None.
 *
 * @retval TRUE   Successfully provide PointStamped in an ongoing process
 * @retval FALSE  Oops, did something.
 *
 * Example Usage:
 * @code
 *    rosrun tachymeter tachymeter_node
      Check whether the aim of the tachymeter is targeting the prism already
 * @endcode
 */

#include "mySerial.h"
#include "order_geocom.h"
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#define PI                               3.1415926535897932384626433832795

// Tachymeter Prototypes to start device, split messages
// and to calculate kartesian coordinates from polar mesurement
void startDevice(bool&); 
std::vector<double> calculateKartesianCoordiantes(double, double, double);
std::vector<double> splitTachyMessage(std::string);
std::string send2TachyAndReceive(std::string);

int main(int argc, char** argv)
{
  bool startTheDevice = false; 
  std::string tachymeter_position_topic_;
  std::string position_local_frame_;
  //Tachy response goes here;
  std::string msg_tachy = ""; 

//------------------------------ START TRACKING CODE ----------------------------------------------------------------------------------------
  ros::init(argc, argv, "measure_with_tachy");
  ros::NodeHandle n;

  ros::NodeHandle pnh_("~");
  pnh_.param<std::string>("tachymeter_position_topic", tachymeter_position_topic_, "tachy_points");
  pnh_.param<std::string>("position_local_frame", position_local_frame_, "tachy_frame");
  

  ros::Publisher point_pub = n.advertise<geometry_msgs::PointStamped>(tachymeter_position_topic_, 10);

  geometry_msgs::PointStamped tachy_meas;
  tachy_meas.header.frame_id = position_local_frame_;
	
  std::vector<double> valuesFromTachy; 
	

  //dummy values
  double dHz = -1; 
  double dVz = -1;
  double ddist = -1;
  double dLastHzAng = -1;
  double dhz_angle_diff = -1;
  int Result = -1;


  std::cout << "FineAdjustment " <<  std::endl;
  msg_tachy = send2TachyAndReceive(AUT_FineAdjust_hardcoded);
  std::cout <<  msg_tachy << std::endl;

  std::cout << "Set LOCK " << std::endl;
  msg_tachy = send2TachyAndReceive(AUS_SetUserLockState_hardcoded);
  std::cout <<  msg_tachy << std::endl;


  std::cout << "Start Tracking " << std::endl;
  msg_tachy = send2TachyAndReceive(AUT_LockIn_hardcoded);
  std::cout << msg_tachy << std::endl;

  std::cout << "MeasureQuick! " << std::endl; 
  msg_tachy = send2TachyAndReceive(TMC_QuickDist_hardcoded);
  std::cout << msg_tachy << std::endl; 

  //TODO(linzer): The following goes into a function later, delete this message if done
  valuesFromTachy= splitTachyMessage(msg_tachy);

  while (ros::ok()) {
    dLastHzAng=valuesFromTachy.at(1);
    msg_tachy = send2TachyAndReceive(TMC_QuickDist_hardcoded);
    std::cout << msg_tachy << std::endl;
    valuesFromTachy = splitTachyMessage(msg_tachy);

    //Result sais wheather measured QuickDist-Data could be used
    //Result==0 ---> measurement ok
    //Result==1285 &&  1284 are only somewhat valid, 
    //lack of distance measurment. Therefore need to evalute what to do.  
    //TODO(linzer): Need differenciation
    Result=valuesFromTachy.at(0);
    std::cout << "Respone of Tachy to tracking request: " << Result << std::endl;
    switch (Result) {
      case 1284: { 
        dhz_angle_diff=fabs(dLastHzAng - valuesFromTachy.at(1)); 
        std::cout << "Measured angle difference: " << dhz_angle_diff << std::endl; 
        //TODO(linzer): Check whether measurement is in between time itervall
        dHz = valuesFromTachy.at(1);
        dVz = valuesFromTachy.at(2);
        ddist = valuesFromTachy.at(3);
        std::cout << "dHz: " << dHz << "	dVz: " << dVz << "		ddist: " << ddist << std::endl;

        std::vector<double> newCoordinates = calculateKartesianCoordiantes(dHz, dVz, ddist);
        //TODO(linzer): Evaluate coorect transformation situation
        tachy_meas.point.x = -1*newCoordinates.at(0);
        tachy_meas.point.y = newCoordinates.at(1);
        tachy_meas.point.z = newCoordinates.at(2);
        std::cout << "dX: " << tachy_meas.point.x << "	dY: " << tachy_meas.point.y << "		dZ: " << tachy_meas.point.z << std::endl;

	tachy_meas.header.stamp = ros::Time::now();
        point_pub.publish(tachy_meas);
        ros::spinOnce();
      } //end case 


       case 0: { 
        dhz_angle_diff=fabs(dLastHzAng - valuesFromTachy.at(1)); 
        std::cout << "Measured angle difference: " << dhz_angle_diff << std::endl; 
        //TODO(linzer): Check whether measurement is in between time itervall
        dHz = valuesFromTachy.at(1);
        dVz = valuesFromTachy.at(2);
        ddist = valuesFromTachy.at(3);
        std::cout << "dHz: " << dHz << "	dVz: " << dVz << "		ddist: " << ddist << std::endl;

        std::vector<double> newCoordinates = calculateKartesianCoordiantes(dHz, dVz, ddist);
        //TODO(linzer): Evaluate coorect transformation situation
        tachy_meas.point.x = -1*newCoordinates.at(0);
        tachy_meas.point.y = newCoordinates.at(1);
        tachy_meas.point.z = newCoordinates.at(2);
        std::cout << "dX: " << tachy_meas.point.x << "	dY: " << tachy_meas.point.y << "		dZ: " << tachy_meas.point.z << std::endl;

	tachy_meas.header.stamp = ros::Time::now();
        point_pub.publish(tachy_meas);
        ros::spinOnce();
      }
    } //end Result Switch
  }//end ros.ok loop



//------------------------------ ENDE TRACKING CODE ------------------------------------------------------------------------------------------



  std::cout << "Program end! Bail out!" << std::endl;
  return 0;
}


std::vector<double> calculateKartesianCoordiantes( double hz, double vz, double distance){
  double radValue_hz = hz; 
  double radValue_vz = vz; 
  double x = distance * sin(radValue_vz) * cos(radValue_hz);
  double y = distance * sin(radValue_vz) * sin(radValue_hz);
  double z = distance * cos(radValue_vz);

  std::vector<double> newCoordinate;
  newCoordinate.push_back(x);
  newCoordinate.push_back(y);
  newCoordinate.push_back(z);

  return newCoordinate;
};

std::vector<double> splitTachyMessage(std::string msg_tachy){
  int pos = msg_tachy.find(":");
  std::string msg_tachy_cut = msg_tachy.substr(pos+1, msg_tachy.length());
  std::istringstream ss(msg_tachy_cut);
  std::string token;
  int c_i = 0;
  std::vector<double> valuesFromTachy;
  while (std::getline(ss, token, ',')) {
    valuesFromTachy.push_back(std::atof(token.c_str()));
  }
  return valuesFromTachy;
};

void startDevice(bool& startTheDevice){
  std::cout << "1 - Starten des Tachymeters" << std::endl << "0 - Programm beenden, Gerät bleibt ausgeschaltet" << std::endl;
  std::cin >> startTheDevice;
  if (startTheDevice) {
    std::cout << "Instrument start..." << std::endl;
    std::cout << "Turn to starting position..." << std::endl;
    std::cout << "Wait until instrument respond!.." << std::endl;
    std::string answerFromTachy = send2TachyAndReceive(AUT_MakePositioning_hardcoded);
  } else {
    std::cout << "Shutdown..." << std::endl;
    std::string answerFromTachy = send2TachyAndReceive(COM_SwitchOffTPS);
    exit(0);
  }
};

// Send any geocom ASCII-Request as an order
// Some orders charge special treatment 
std::string send2TachyAndReceive(std::string order){                      
  mySerial serial(deviceFile,deviceBaudrate);
  serial.Send(order);
  std::string answerFromTachy = "notValid";
  // TODO: Killswitch
  //Instrument shutdown command. If COM_SwitchOffTPS do not wait for a response
  if (order==COM_SwitchOffTPS) { 
    return answerFromTachy;
  } else if (order!=AUT_MakePositioning_hardcoded) { 
    while(answerFromTachy == "notValid" || answerFromTachy.empty()) {
    std::ifstream read(deviceFile.c_str());
    std::getline(read,answerFromTachy);
    }
  } else {
    // Start instrument AND turn to starting position
    if (order==AUT_MakePositioning_hardcoded) { 
      //wait for 3 seconds and check for a non empty response (valid check)
      sleep(3); 
      std::cout << "..." << std::endl;
      std::ifstream read(deviceFile.c_str());
      std::getline(read,answerFromTachy);
      std::cout << "Answer from Tachy: " << answerFromTachy << std::endl;
      if (!answerFromTachy.empty()) { 
        std::cout << "Started! You may proceed." << std::endl;
        return "started";
      }
      // Recursive call, to check whether instrument
      // is started allready   
      send2TachyAndReceive(AUT_MakePositioning_hardcoded); 
    }      
  }
  return answerFromTachy;
}


