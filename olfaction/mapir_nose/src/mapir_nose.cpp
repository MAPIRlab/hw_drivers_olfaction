// 1 TAB = 8 SPACES // LINE LENGTH = 100 CHARACTERS //

/*	+-----------------------------------+-----------------------------------+
	|                                                                       |
	|                          YOUR PROJECT'S NAME                          |
	|                  https://project.url/if.there.is.any                  |
	|                                                                       |
	| Copyright (c) 2015 - 2016, Individual contributors, see AUTHORS file. |
	| Machine Perception and Intelligent Robotics (MAPIR),  		|
	| University of Malaga. <http://mapir.isa.uma.es>                       |
	|                                                                       |
	| This program is free software: you can redistribute it and/or modify  |
	| it under the terms of the GNU General Public License as published by  |
	| the Free Software Foundation, either version 3 of the License, or     |
	| (at your option) any later version.                                   |
	|                                                                       |
	| This program is distributed in the hope that it will be useful,       |
	| but WITHOUT ANY WARRANTY; without even the implied warranty of        |
	| MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
	| GNU General Public License for more details.                          |
	|                                                                       |
	| You should have received a copy of the GNU General Public License     |
	| along with this program. If not, see <http://www.gnu.org/licenses/>.  |
	|                                                                       |
	+-----------------------------------------------------------------------+ */


#include "ros/ros.h"
#include <olfaction_msgs/gas_sensor.h>
#include <olfaction_msgs/gas_sensor_array.h>

#include "mavlink/enose/mavlink.h"
#include "serial_port.h"
#include "LinuxTimer.hpp"
#include <map>			// Container for key-value pairs
#include <string>


// At this point I lost track of which ones are really needed. TODO: Clean
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>


using namespace std;



//##################################################################################################

class GasInfoCompacter
{
public:			
	void appendSensorData(mavlink_message_t& msg)
	{
		// DECODE DATA: msg -> decoded_msg
		mavlink_gas_sensor_measurement_t decoded_msg;
		mavlink_msg_gas_sensor_measurement_decode(&msg,&decoded_msg);	
				
				
		// GENERATE KEY TO STORE DATA: unique ID for each sensors on the e-nose: combine all the information we have about it to create a unique string
		std::string key;
		key  = to_string(msg.sysid) + "-";
		key += to_string(msg.compid) + "-";
		key += to_string(decoded_msg.technology) + "-";
		key += to_string(decoded_msg.manufacturer) + "-";
		key += to_string(decoded_msg.mpn);
		
		
		// ADD SYSID TO MPN FOR EASIER IDENTIFICATION
		decoded_msg.mpn += (msg.sysid<<3);
				
				
		// STORE DATA USING KEY, AND RESET READ COUNTER
		data[key] = decoded_msg;
		read_counter[key] = 0; // this tracks time between updates
	}
			
			
	olfaction_msgs::gas_sensor_array getSensorDataAsGasSensorArray()
	{
		olfaction_msgs::gas_sensor sensor;
		olfaction_msgs::gas_sensor_array sensor_array;
	
		for (map<std::string,mavlink_gas_sensor_measurement_t>::iterator it = data.begin(); it != data.end(); ++it)
		{
			// GET KEY
			string key = it->first;
			
			
			// READ SENSOR DATA
			mavlink_gas_sensor_measurement_t sensor_data;
			sensor_data = data[key];
			read_counter[key]++;
			
			
			// CHECK IF SENSOR HAS NOT BEEN UPDATED FOR A LONG TIME->REMOVE
			if(read_counter[key]>20)
			{
				data.erase(key);
			}
			
			
			// COMPOSE SENSOR MESSAGE	
			sensor.technology = sensor_data.technology;
			sensor.manufacturer = sensor_data.manufacturer;
			sensor.mpn = sensor_data.mpn;
			sensor.raw = sensor_data.raw_data;
			sensor.raw_air = sensor_data.units; 
			sensor.calib_A = sensor_data.calibraiton_a;
			sensor.calib_B = sensor_data.calibraiton_b;
			sensor.raw_units = sensor_data.units;
			sensor.header.frame_id = "base_link";
						
			
			// ADD TO ARRAY
			sensor_array.sensors.push_back(sensor);		
		}
		
		return sensor_array;
	}


private:
	map<std::string,mavlink_gas_sensor_measurement_t> data;
	map<std::string, int> read_counter;

};


//##################################################################################################




mavlink_message_t msg;
mavlink_status_t status;
GasInfoCompacter compacter;



bool processMsg(uint8_t c)
{
	if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
	{
		switch(msg.msgid)
		{
		case MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT:
			//mavlink_msg_gas_sensor_measurement_decode(&msg,&gas);
			compacter.appendSensorData(msg);
			break;
		default:
			break;
		}

		return true;
	}
	else
	{
		return false;
	}
}



//##################################################################################################


int main(int argc, char **argv)
{
	ROS_INFO("STARTING MAPIR NOSE!");

	ros::init(argc, argv, "mapir_nose");
	ros::NodeHandle n("~");


	string device;
	n.param<string>("serial_device", device, "/dev/ttyUSB0");
	ROS_INFO("Param serieal_device: Got serial device: %s", device.c_str());
	
	string topic;	
	n.param<string>("topic", topic, "/olfaction/mapir_enose");
	ROS_INFO("Param topic: I will publish on topic: %s", topic.c_str());
	
	
	// PUBLISH ON THESE TOPICS
	ros::Publisher sensor_array_pub = n.advertise<olfaction_msgs::gas_sensor_array>(topic.c_str(), 2);
	
	double refresh_period_s;
	n.param<double>("refresh_period_s", refresh_period_s, 1.0);
	ROS_INFO("Param refresh_period_s: I will publish e-nose data every %f seconds", refresh_period_s);

	
	SerialPort file(device);
	LinuxTimer timer;
	

	if (file.isOpen())
	{
		// OPEN SERIAL PORT	
		if(file.configure(SerialPort::BaudRate::BR_115200,SerialPort::DataBits::BITS_8,SerialPort::Parity::DISABLED,true,1))
			ROS_INFO("Serial port Ready");


		// MAIN LOOP
		while (ros::ok())
		{
			// READ DATA FROM SERIAL INTERFACE
			char byte;
			file.get(byte);
			processMsg(byte); // Extract data from serial stream one byte at a time
			

			// PUBLISH DATA
			if(timer.getElapsedSeconds() >= refresh_period_s)
			{
				timer.reset();
				sensor_array_pub.publish(compacter.getSensorDataAsGasSensorArray());
			}


			ros::spinOnce();
		}//while (ros::ok())


		file.close();
		ROS_INFO("Serial port closed");
	}//if(file.isOpen())
	else
	{
		ROS_INFO("CAN NOT OPEN %s", device.c_str());
	}

	return 0;
}


