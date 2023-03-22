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


#ifndef __SERIAL_PORT__
#define __SERIAL_PORT__


#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <limits.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>


/***********************************************************************************************//**
 * @brief
 **************************************************************************************************/
class SerialPort
{
public:
				enum class BaudRate
				{
					BR_50		= B50,
					BR_75		= B75,
					BR_110		= B110,
					BR_134		= B134,
					BR_200		= B200,
					BR_300		= B300,
					BR_600		= B600,
					BR_1200		= B1200,
					BR_2400		= B2400,
					BR_4800		= B4800,
					BR_9600		= B9600,
					BR_19200	= B19200,
					BR_38400	= B38400,
					BR_57600	= B57600,
					BR_115200	= B115200,
					BR_230400	= B230400,
				};

				enum class DataBits
				{
					BITS_5	= CS5,
					BITS_6	= CS6,
					BITS_7	= CS7,
					BITS_8	= CS8
				};

				enum class Parity
				{
					DISABLED,
					ODD,
					EVEN
				};

				//------------------------------------------------------------------
public:

				SerialPort(const std::string& device)
				{
					SerialPort();	// Call default constructor
					open(device);
				}

				SerialPort(void) : 
					fd_(-1) 
				{}

	virtual			~SerialPort(void)
				{
					if (isOpen())	{close();}
				}


	bool			open(const std::string& device)
				{
					termios port_settings;

					try
					{
						// CHECK IF ALREADY OPEN
						//Todo


						// CHECK IF DEVICE NAME IS NOT CORRECT
						if (not device.size())	{throw("Serial port name may not be empty");}


						// OPEN DEVICE
						// The O_NOCTTY flag tells UNIX that this program doesn't want to be the "controlling terminal" for that port.
						// The O_NDELAY flag tells UNIX that this program doesn't care what state the DCD signal line is in - whether the other end of the port is up and running
						fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
						if (not isOpen())	{throw("Could not open serial port");}


						// CLEAR CONFIGURATION FLAGS
						fcntl(fd_, F_SETFL, 0);


						// ASSEMBLE NEW CONFIGURAITON
						bzero(&port_settings,sizeof(port_settings));	// Init to 0
						port_settings.c_cflag |= B115200 | CS8 | CREAD | CLOCAL;	// Baudrate | 8bit | Enable receiver | Ignore modem control lines

						port_settings.c_cc[VTIME]    = 10;   // Multiple of 0.1s. 0 = Unlimited //
						port_settings.c_cc[VMIN]     = 1;   // blocking read until 1 chars received //


						// APPLY NEW CONFIGURATION
						if (tcflush(fd_, TCIFLUSH) < 0)			{throw("Serial port not flushed");}
						if (tcsetattr(fd_,TCSANOW, &port_settings) < 0)	{throw("Serial port configuration not written");}
					}
					catch(const char* msg)
					{
						std::cerr << "EXCEPTION: " <<msg << std::endl;
						close();
					}

					return isOpen();
				}
				
				
	bool			isOpen(void) const
				{
					return fd_ >= 0 ? true : false;
				}
				
				
	void			close(void)
				{
					if (isOpen())	{::close(fd_);}
				}


	bool			configure(BaudRate baud_rate,
					  DataBits character_size,
					  Parity parity,
					  bool enable_stop_bit,
					  unsigned int number_stop_bits)
				{
					bool success = false;
					termios port_settings;

					try
					{
						// CHECK IF DEVICE IS OPEN AND SIZE IS CORRECT
						if (not isOpen())	{throw("Serial port not open");}


						// GET CURRENT SETTINGS
						if (tcgetattr(fd_, &port_settings) < 0 )
							throw("Cannot get the current settings");


						// SET BAUD RATE
						if ((cfsetispeed(&port_settings,(int)baud_rate) < 0 ) || ( cfsetospeed(&port_settings,(int)baud_rate) < 0 ))
							throw("Cannot change Baud Rate");


						// SET CHARACTER SIZE
						port_settings.c_cflag &= ~CSIZE ;
						port_settings.c_cflag |= (int)character_size;


						// SET PARITY
						switch (parity)
						{
						case Parity::EVEN:
							port_settings.c_cflag |= PARENB;
							port_settings.c_cflag &= ~PARODD;
							port_settings.c_iflag |= INPCK;
							break ;
						case Parity::ODD:
							port_settings.c_cflag |= (PARENB | PARODD);
							port_settings.c_iflag |= INPCK;
							break ;
						case Parity::DISABLED:
							port_settings.c_cflag &= ~(PARENB);
							port_settings.c_iflag |= IGNPAR;
							break ;
						default:
							break;
						}


						// ENABLE/DISABLE STOP BIT
						if (enable_stop_bit)	{port_settings.c_cflag |= CSTOPB;}
						else			{port_settings.c_cflag &= ~(CSTOPB);}


						// STOP BITS
						switch (number_stop_bits)
						{
						case 1:	port_settings.c_cflag &= ~(CSTOPB);	break ;
						case 2:	port_settings.c_cflag |= CSTOPB;	break ;
						default:
							throw("Invalid number of stop bits");
							break;
						}


						// UPDATE SETTINGS
						if (tcflush(fd_, TCIFLUSH) < 0)			{throw("Serial port not flushed");}
						if (tcsetattr(fd_,TCSANOW, &port_settings) < 0)	{throw("Serial port configuration not written");}
					}
					catch(const char* msg)
					{
						std::cerr << "EXCEPTION: " <<msg << std::endl;
						success = false;
					}

					return success;
				}


	std::size_t		read(void* buffer, std::size_t size)
				{
					std::size_t already_readed = 0;

					try
					{
						// CHECK IF DEVICE IS OPEN AND SIZE IS CORRECT
						if (not isOpen())	{throw("Serial port not open");}
						if (size == 0)		{return 0;}


						while (already_readed < size)
						{
							std::size_t left_to_read = size - already_readed;
							int new_readed = 0;


							// TODO: Check if device still present, if not close this and return already_readed


							// READ NEW DATA
							new_readed = ::read(fd_, ((char*)buffer)+already_readed, left_to_read);

							if (new_readed >= 0)	{already_readed += new_readed;}
							else			{} // TODO: error
						}
					}
					catch(const char* msg)
					{
						std::cerr << "EXCEPTION: " <<msg << std::endl;
					}

					return already_readed;
				}


	void			get(char& byte)
				{
					read(&byte,1);
				}


private:
	int			fd_;
};


/* ---------------------------------------------------------------------------------------------- */
#endif // __SERIAL_PORT__
