#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>

#include <ros/ros.h>
#include <serial/serial.h>
#include <olfaction_msgs/anemometer.h>
#include <visualization_msgs/Marker.h>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <angles/angles.h>
#include <tf/transform_listener.h>

using namespace std;

//Globals
ros::Publisher windmarker_pub;
ros::Publisher wind_pub;
std::string frame_id;
std::vector<float> windS;
std::vector<float> windD;
ros::Time displayTime;



struct WindMeasurement
{
	char id;
    int direction;
    float speed;
    char units;
    int status;
};


float get_average_wind_direction(std::vector<float> const &v)
{
    //Average of wind direction, avoiding the problems of +/- pi angles.
    float x =0.0, y = 0.0;
    for(std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
    {
        x += cos(*i);
        y += sin(*i);
    }
    float average_angle = atan2(y, x);   


    return average_angle;
}

float get_average_vector(std::vector<float> const &v)
{
    size_t length = v.size();
    float sum = 0.0;
    for(std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
        sum += *i;

    return sum/length;
}

WindMeasurement parseWindMeasurement(std::string &data)
{
    // WindSonic mode (default is Polar continuous)
    // format = <STX>Q, dir, speed, units, status, <ETX>\r
    // Q      -> Sensor identifier
    // dir    -> 3 digits (0-359)
    // speed  -> %3.2f
    // units  -> M (m/s), K (km/h)
    // status -> 00 = ok, else is an Error Code
    
    
	WindMeasurement wind;
	
	
	// SEPARATE MESSAGE INTO SUB-STRINGS
	std::vector<std::string> strings;
    boost::split(strings, data, boost::is_any_of(","));
 
    
    // CHECK FOR MESSAGE INTEGRITY
    if (strings.size()!=6)
    {
    	ROS_ERROR("[windsonic_node] Message could not be parsed:");
    	for (int i=0; i<strings.size(); i++) { ROS_ERROR("%s",strings[i].c_str()); }
		ROS_ERROR("-----");
		wind.id='x';
		return wind;
    }
	
    
    // PARSE
    
    // 0. Q
    strings[0].erase(0,1);						// Delete 0x02 (STX)
    sscanf(strings[0].c_str(), "%c", &wind.id);	//ID (usually='Q')

	// 1. DIR 3 digits (0-359), empty if no wind
	if (strings[1].length() < 3)
	{
		wind.direction = -1;
	} else {
		sscanf(strings[1].c_str(), "%3u", &wind.direction);        
	}
	
	// 2. SPEED %3.2f
	if (strings[2].length() > 6)
	{
		wind.speed = -1.0;
	} else {
		sscanf(strings[2].c_str(), "%f", &wind.speed);        
	}
	
	// 3. UNITS
	if (strings[3].length() != 1)
	{
		wind.units = '?';
	} else {
		sscanf(strings[3].c_str(), "%c", &wind.units);
		if (wind.units != 'M' && wind.units != 'K')
		{
			wind.units = '?';
		}	
	}      
	
	// 4. STATUS
	if (strings[4].length() < 1)
	{
		wind.status = -1;
	} else {
		sscanf(strings[4].c_str(), "%u", &wind.status);
	}
	
	return wind;
}




void read_anemometer(std::string &data)
{
	// GET WIND MEASUREMENT
	WindMeasurement wind = parseWindMeasurement(data);
	

	if(wind.id=='x'){
		//if there was an error parsing, ignore the message instead of closing the node
		return;
	}
	
	// PROCESS DIRECTION -> If none, no wind detected
	if (wind.direction < 0)
	{
		wind.direction = 0;
		wind.speed = 0.0;
	}
	
	// PROCESS SPEED -> Check if valid, and convert to m/s
	if (wind.speed < 0)
	{
		wind.direction = 0;
		wind.speed = 0.0;
		ROS_ERROR("[windsonic] Reported invalid wind speed");
	}
	else
	{
		if (wind.units == 'M' )
		{
			ROS_DEBUG("[windsonic] Wind speed in M/S");		
		}
		else if (wind.units == 'K' )
		{
			ROS_DEBUG("[windsonic] Wind speed in Km/h");
			wind.speed *= 1000.0/3600.0;	
		}
		else
		{
			ROS_ERROR("[windsonic] Wind speed reported in invalid units");
			wind.direction = 0;
			wind.speed = 0.0;
		}
	}
	
	
	// PROCESS STAUTS
	if (wind.status != 0)
	{
		ROS_WARN("[windsonic] Satus = %d, wind-reading might be unrealiable", wind.status);
	}
	
	
	// VERBOSE
	ROS_INFO("[windsonic] Speed: %f\tDirection: %d\tStatus: %d", wind.speed, wind.direction, wind.status);
	
	
	// CREATE OLFACTION_MSGS ANEMOMETER MESSAGE
	olfaction_msgs::anemometer wind_msg;
	
    wind_msg.header.stamp = ros::Time::now();
    wind_msg.header.frame_id = frame_id.c_str();
    wind_msg.sensor_label = "windSonic";
    wind_msg.wind_speed = wind.speed;
    wind_msg.wind_direction = angles::from_degrees(-wind.direction)+M_PI-M_PI/4;  //deg to rad

    wind_pub.publish(wind_msg);

	windS.push_back(wind_msg.wind_speed);
	windD.push_back(angles::normalize_angle(wind_msg.wind_direction));
	
	if( (ros::Time::now() - displayTime).toSec() >= 1 ){
		float average_wind_direction = get_average_wind_direction(windD);
		float average_wind_speed = get_average_vector(windS);

		visualization_msgs::Marker wind_point_inv;
		wind_point_inv.header.frame_id = frame_id.c_str();
		wind_point_inv.action = visualization_msgs::Marker::ADD;
		wind_point_inv.ns = "measured_wind_inverted";
		wind_point_inv.type = visualization_msgs::Marker::ARROW;

		wind_point_inv.header.stamp = ros::Time::now();
					wind_point_inv.points.clear();
					wind_point_inv.id = 1;  //unique identifier for each arrow
					wind_point_inv.pose.position.x = 0;
					wind_point_inv.pose.position.y = 0;
					wind_point_inv.pose.position.z = 0.0;
					wind_point_inv.pose.orientation = tf::createQuaternionMsgFromYaw(average_wind_direction);
					wind_point_inv.scale.x = average_wind_speed;	  //arrow lenght
					wind_point_inv.scale.y = 0.1;	  //arrow width
					wind_point_inv.scale.z = 0.1;	  //arrow height
					wind_point_inv.color.r = 0.0;
					wind_point_inv.color.g = 1.0;
					wind_point_inv.color.b = 0.0;
					wind_point_inv.color.a = 1.0;
		windmarker_pub.publish(wind_point_inv);
		displayTime=ros::Time::now();
		windD.clear();
		windD.clear();
	}
    
}

// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "windsonic_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    serial::Serial my_serial;

    //Get node parameters
    std::string port;
    int baudrate;
    std::string topic;
    
    pn.param<std::string>("port", port, "/dev/ttyUSB1");
    pn.param("baud", baudrate, 9600);
    pn.param<std::string>("frame_id", frame_id, "windsonic_link");
    pn.param<std::string>("topic", topic, "/wind");

    ROS_INFO("Initializing module at port:%s:%u on frame reference:%s",port.c_str(),baudrate,frame_id.c_str());

	wind_pub = n.advertise<olfaction_msgs::anemometer>(topic, 10);
	windmarker_pub = n.advertise<visualization_msgs::Marker>("Windvector_display", 100);

    //Open serial port
    try
    {
        my_serial.setPort(port);
        my_serial.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        my_serial.setTimeout(to);
        my_serial.open();
    }
    catch(serial::IOException &e)
    {
        ROS_ERROR_STREAM("WindSonic -- Failed to open serial port!");
        ROS_BREAK();
    }

    if (my_serial.isOpen())
        ROS_INFO_STREAM("Serial port initialized.");
    else
        return -1;

    //Read Loop
    ros::Rate loop_rate(5);
    while(ros::ok())
    {        
        ros::spinOnce();

        if(my_serial.available())
        {
            std::string result;
            result = my_serial.readline(65536,"\r");
            
            read_anemometer(result);
        }
        loop_rate.sleep();
    }

    my_serial.close();
    return(0);
}



