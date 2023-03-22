#include <falcon/falcon.h>

int main( int argc, char** argv)
{
    ros::init(argc, argv, "falcon_tdlas");
    Falcon falcon;

    falcon.run();
    return 0;
}

Falcon::Falcon()
{
    ros::NodeHandle private_nh("~");
    m_settings.frequency =  private_nh.param<float>("frequency", 2);
    m_settings.port =  private_nh.param<std::string>("port", "/dev/ttyUSB0");
    m_settings.topic =  private_nh.param<std::string>("topic", "/falcon/reading");
    m_settings.verbose = private_nh.param<bool>("verbose", false);

    m_publisher = m_nodeHandle.advertise<olfaction_msgs::gas_sensor>(m_settings.topic, 10);
}

void Falcon::run()
{
    serial::Serial serial;
    if(!initSerial(serial))
        return;
    
    ros::Rate rate(m_settings.frequency);
    
    while( !doHandShake(serial) && ros::ok())
        rate.sleep();

    
    std::string bytesBuffer;
    while(ros::ok())
    {
        ros::spinOnce();

        InMessage reading = getReading(serial);
        
        if(reading.valid)
            publish(reading);
            
        rate.sleep();
    }

    serial.close();
}

bool Falcon::initSerial(serial::Serial& serial)
{
    serial.setBaudrate(19200);
    serial.setStopbits(serial::stopbits_one);
    serial.setParity(serial::parity_none);
    serial.setBytesize(serial::eightbits);
    serial.setPort(m_settings.port);
    serial.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);

    try{
        serial.open();
    }
    catch (std::exception& e)
    {
        ROS_ERROR("[FALCON_TDLAS] %s", e.what());
    }

    if(!serial.isOpen())
    {
        ROS_ERROR("[FALCON_TDLAS] Failed to open serial port\n");
        return false;
    }
    return true;
}

bool Falcon::doHandShake(serial::Serial& serial)
{
    
    //version message
    {
        const std::array<uint8_t,3> VER = {0x56, 0x45, 0x52};
        OutMessage message;

        message <<
            MSGCodes::BEGIN_MESSAGE << MSGCodes::ETC << MSGCodes::SEPARATOR 
            << VER << MSGCodes::QUERY
            << MSGCodes::END_MESSAGE;
        
        message << message.BCC();

        serial.write(message.data);

        if(!serial.waitReadable())
        {
            ROS_INFO("[FALCON_TDLAS] Timeout - No response during handshake");
            return false;
        }

        auto read = serial.read(256);
        if(read.size()==0 ||read.at(0) != MSGCodes::ACK)
        {
            ROS_ERROR("[FALCON_TDLAS] Error during handshake! Received message did not contain an ACK: %s\n", read.c_str());
            return false;
        }
    }

    {
        const std::array<uint8_t,3> ALL = {0x41, 0x4C, 0x4C};

        serial.write(&MSGCodes::ACK, 1);
        OutMessage message;
        message << 
            MSGCodes::BEGIN_MESSAGE << MSGCodes::CMN <<MSGCodes::SEPARATOR
            << ALL << MSGCodes::QUERY << MSGCodes::END_MESSAGE;
        
        message << message.BCC();
        message << std::vector<uint8_t>(message.data); //duplicate the message without the ACK (for some reason)
        serial.write(message.data);

        if(!serial.waitReadable())
        {
            ROS_INFO("[FALCON_TDLAS] Timeout - No response during handshake");
            return false;
        }

        auto read = serial.read(512);

        if(read.size()==0 || read.at(0) != MSGCodes::ACK)
        {
            ROS_ERROR("[FALCON_TDLAS] Error during handshake! Received message did not contain an ACK: %s\n", read.c_str());
            return false;
        }

        serial.write(&MSGCodes::ACK, 1);
    }

    return true;
}

InMessage Falcon::getReading(serial::Serial& serial)
{
    OutMessage message;
    message << MSGCodes::ACK << MSGCodes::BEGIN_MESSAGE 
        << MSGCodes::ETC << MSGCodes::SEPARATOR << MSGCodes::FWD
        << MSGCodes::QUERY << MSGCodes::END_MESSAGE;

    message << message.BCC();

    serial.write(message.data);

    if(!serial.waitReadable())
    {
        ROS_INFO("[FALCON_TDLAS] Timeout - No response to reading request");
        return {};
    }

    auto read = serial.read(256);
    if(read.size()==0 || read.at(0) != MSGCodes::ACK)
    {
        ROS_ERROR("[FALCON_TDLAS] Error getting a sensor reading! Received message did not contain an ACK: %s\n", read.c_str());
        return {};
    }

    InMessage reading;
    try{
        reading = InMessage(read);
    }
    catch(std::exception& e)
    {
        ROS_ERROR("[FALCON_TDLAS] Exception: %s received while trying to parse sensor reading: %s\n", e.what(), read.c_str());
    }
    
    if(m_settings.verbose)
    {
        std::cout<<"[FALCON_TDLAS] ";
        std::cout<<(reading.valid? "valid" : "ERROR")<<"\n"
            <<"PPMxM: "<< reading.average_PPMxM <<"\n"
            << "\n";
            for(auto it = reading.readings.begin(); it != reading.readings.end(); it++)
            {
                std::cout<< "\tPPMxM: " <<it->PPMxM <<"\n"
                            << "\tReflectivity: " <<it->reflectionStrength<<"\n"
                            << "\tAbsorptivity: " <<it->absorptionStrength<<"\n"
                            << "\tTimestamp: " <<it->timestamp<<"\n";
            }
    }

    return reading;
}


uint8_t OutMessage::BCC()
{
    uint8_t running = 0;

    uint8_t last_read = 0;
    int i = 0;
    bool started = false;
    while(i<data.size() && last_read != MSGCodes::END_MESSAGE)
    {
        if(last_read == MSGCodes::BEGIN_MESSAGE)
            started = true;

        if(started)
            running ^= data[i];
        
        last_read = data[i];
        i++;
    }
    return running;
}

void Falcon::publish(const InMessage& reading)
{
    olfaction_msgs::gas_sensor msg;
    msg.technology = msg.TECH_TDLAS;
    msg.raw = reading.average_PPMxM;
    msg.raw_units = msg.UNITS_PPMxM;

    m_publisher.publish(msg);
}
