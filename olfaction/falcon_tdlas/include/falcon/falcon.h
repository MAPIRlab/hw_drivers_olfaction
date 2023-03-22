#pragma once

#include <ros/ros.h>
#include <serial/serial.h>
#include <olfaction_msgs/gas_sensor.h>
#include <falcon/OutMessage.h> 
#include <falcon/SensorReading.h>

class Falcon
{
    public:
    Falcon();
    ~Falcon()=default;

    void run();

    private:
    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_publisher;

    struct Settings
    {
        float frequency;
        std::string port;
        std::string topic;
        bool verbose;
    };
    Settings m_settings;

    bool initSerial(serial::Serial& serial);
    bool doHandShake(serial::Serial& serial);
    InMessage getReading(serial::Serial& serial);

    void publish(const InMessage& msg);
};
