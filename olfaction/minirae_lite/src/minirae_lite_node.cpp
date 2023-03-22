#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>

#include <ros/ros.h>
#include <serial/serial.h>
#include <olfaction_msgs/gas_sensor_array.h>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

using namespace std;

//Globals

// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "minirae_lite_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    serial::Serial my_serial;

    //Get node parameters
    std::string port;
    int baudrate;
    std::string frame_id;
    std::string topic_name;
    pn.param<std::string>("port", port, "/dev/ttyUSB0");
    pn.param("baud", baudrate, 9600);
    pn.param<std::string>("frame_id", frame_id, "minirae_lite_link");
    pn.param<std::string>("topic_name", topic_name, "/PID_reading");

    ROS_INFO("Initializing module at port:%s:%u on frame reference:%s",port.c_str(),baudrate,frame_id.c_str());
    ros::Publisher pub = n.advertise<olfaction_msgs::gas_sensor>(topic_name, 10);

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
        ROS_ERROR_STREAM("MiniRAE_lite -- Failed to open serial port!");
        ROS_BREAK();
    }

    if (my_serial.isOpen())
        ROS_INFO_STREAM("Serial port initialized.");
    else
        return -1;

    //Loop
    ros::Rate loop_rate(20);
    while(ros::ok())
    {        
        ros::spinOnce();

        my_serial.flush();
        //Request the PID a new  reading
        my_serial.write("R");
        if(my_serial.available())
        {
            olfaction_msgs::gas_sensor sensor_msg;
            std::string result;
            //result = my_serial.read(my_serial.available());
            result = my_serial.readline(500,"\r");
            //ROS_INFO_STREAM("Read string:" << result);
            //string to ppm
            float ppm = atof(result.c_str())/1000;

            //ROS_INFO_STREAM("Read ppm:" << ppm);

            //Create msg and publish
            sensor_msg.header.stamp = ros::Time::now();
            sensor_msg.header.frame_id = frame_id.c_str();
            
            sensor_msg.technology = sensor_msg.TECH_PID;
            sensor_msg.manufacturer = sensor_msg.MANU_RAE;
            sensor_msg.mpn = sensor_msg.MPN_MINIRAELITE;
            sensor_msg.raw_units = sensor_msg.UNITS_PPM;
            sensor_msg.raw_air = 0.0;
            sensor_msg.raw = ppm;

            pub.publish(sensor_msg);
        }


        loop_rate.sleep();
    }

    my_serial.close();
    return(0);
}



