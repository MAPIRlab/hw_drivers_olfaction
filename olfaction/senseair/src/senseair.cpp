#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <olfaction_msgs/gas_sensor_array.h>

ros::Publisher measurement_pub;
std::string frame_id;

int main(int argc, char** argv){
    ros::init(argc, argv, "senseair");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    serial::Serial my_serial;

    //Get node parameters
    std::string port;
    int baudrate;
    std::string topic;
    
    pn.param<std::string>("port", port, "/dev/ttyUSB0");
    pn.param("baud", baudrate, 9600);
    pn.param<std::string>("frame_id", frame_id, "/s8_link");
    pn.param<std::string>("topic", topic, "/co2_reading");

    ROS_INFO("Initializing module at port:%s:%u on frame reference:%s",port.c_str(),baudrate,frame_id.c_str());

	measurement_pub = n.advertise<olfaction_msgs::gas_sensor>(topic, 10); 

    //Open serial port
    try
    {
        my_serial.setFlowcontrol(serial::flowcontrol_none);
        my_serial.setParity(serial::parity_none);
        my_serial.setStopbits(serial::stopbits_one);
        my_serial.setPort(port);
        my_serial.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        my_serial.setTimeout(to);
        my_serial.open();
    }
    catch(serial::IOException &e)
    {
        ROS_ERROR_STREAM("Senseair s8 -- Failed to open serial port!");
        ROS_BREAK();
    }

    if (my_serial.isOpen())
        ROS_INFO_STREAM("Serial port initialized.");
    else
        return -1;

    uint8_t readCommand[] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};
    
    //Loop
    ros::Rate loop_rate(0.5);
    while(ros::ok())
    {        
        ros::spinOnce();

        my_serial.flush();
        //Request the sensor a new  reading
        my_serial.write(readCommand,8);

        if(my_serial.available())
        {
            olfaction_msgs::gas_sensor sensor_msg;
            std::string result;
            //result = my_serial.read(my_serial.available());
            result = my_serial.read(7);

            /* 
            std::cout<< "Message("<<result.length()<<"): ";
            for (int i=0; i<result.length(); i++){
                printf("%02X ",(unsigned char)(result[i]));
            }
            std::cout<<"\n";
            */

            //string to ppm
            unsigned char firstByte = result[3];
            unsigned char secondByte = result[4];
            unsigned int ppm = (firstByte << 8) + secondByte;

            ROS_INFO_STREAM("Read ppm: " << ppm);

            //Create msg and publish
            sensor_msg.header.stamp = ros::Time::now();
            sensor_msg.header.frame_id = frame_id.c_str();
            
            sensor_msg.raw_units = sensor_msg.UNITS_PPM;
            sensor_msg.raw_air = 0.0;
            sensor_msg.raw = ppm;

            measurement_pub.publish(sensor_msg);
        }


        loop_rate.sleep();
    }

    my_serial.close();
    return(0);
}