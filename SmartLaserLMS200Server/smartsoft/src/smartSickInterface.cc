//--------------------------------------------------------------------------
//
//  Copyright (C) 2003 Boris Kluge, Andreas Steck
//
//        schlegel@hs-ulm.de
//
//        Prof. Dr. Christian Schlegel
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License 
//  as published by the Free Software Foundation; either version 2.1 
//  of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along 
//  with this library; if not, write to the Free Software Foundation, Inc., 
//  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
//  This work is based on previous work by Christian Schlegel, FAW,
//  and on the work by the folks from PlayerStage.
//
//--------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Supported: Sick LMS 200, PLS
//
// CREDITS:
//
// The laser code was taken from the Playerstage Project,
// which is distributed under GPL, and you can find at 
// http://playerstage.sourceforge.net/
//
// The PlayerStage Project:
//   Copyright (C) 2000  
//      Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
//                   
//----------------------------------------------------------------------------

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string.h>

// asteck
#include <linux/serial.h>
#include <sys/ioctl.h>

#define HAVE_HI_SPEED_SERIAL

#include "smartSickInterface.hh"

using namespace Smart;

SickInterface::SickInterface()
: verbose(false), 
  sick_type(LMS),
  length_unit(1), 
  resolution(50), 
  width(180), 
  bitrate(38400),
  term_bitrate(38400),
  _fd(0)
{ 
  // setting to 0 workd fine to set the baud rate to 500kbaud
  serial_high_speed_mode = 0; 
}

////////////////////////////////////////////////////////////////////////////////
// Open the serial device
// Returns 0 on success
//
int SickInterface::open_device(const char *dev_name)
{
  if(verbose) std::cerr << "open_device(\"" << dev_name << "\")" << std::endl;
  _fd = ::open(dev_name, O_RDWR | O_SYNC , S_IRUSR | S_IWUSR );
  if(_fd < 0)
  {
    std::cerr << "ERROR: unable to open serial port" << std::endl;
    return 1;
  }

  // set the serial port speed to 9600 to match the laser
  // later we can ramp the speed up to the SICK's 38K
  //
  struct termios term;
  if( tcgetattr( _fd, &term ) < 0 )
  {
    std::cerr << "ERROR: Unable to get serial port attributes" << std::endl;
    return -1;
  }
  
  if(sick_type==LMS)
  {
    term.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
    term.c_oflag &= ~OPOST;
    term.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
    term.c_cflag &= ~(CSIZE|PARENB);
    term.c_cflag |= CS8;
  }
  else if(sick_type==PLS)
  {
    term.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
    term.c_oflag &= ~OPOST;
    term.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
    term.c_cflag &= ~(CSIZE);
    term.c_cflag |= (CS8|PARENB);
  }
  else
  {
    std::cerr << "WARNING: unknown SICK type." << std::endl;
  }
  
  cfsetispeed( &term, B9600 );
  cfsetospeed( &term, B9600 );
  
  if( tcsetattr( _fd, TCSAFLUSH, &term ) < 0 )
  {
    std::cerr << "ERROR: Unable to set serial port attributes" << std::endl;
    return -1;
  }

  // Make sure queue is empty
  //
  tcflush(_fd, TCIOFLUSH);
    
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Close the terminal
// Returns 0 on success
//
int SickInterface::close_device()
{
  if(_fd>0) 
  {
    ::close(_fd);
    return 0;
  }
  return -1;
}



////////////////////////////////////////////////////////////////////////////////
// Serial connection speed probing.
// Returns:  =0 on success and sets the sick_speed and sick_type arguments,
//          !=0 on failure
//
int SickInterface::probe_sick_speed(unsigned int &sick_speed, std::string &sick_name)
{
  //static int rates[] = { 1200, 1800, 2400, 4800, 9600, 19200, 38400, 0 };
  static int rates[] = { 9600, 38400, 500000 };
  sick_speed = 0;
  sick_name = "";
  int *r = rates;
  while((*r)>0)
  {
    std::cout << "probing " << *r << "bps..." << std::endl;
    if(set_term_speed(*r)==0)
    {
      int result;
      SensorStatus status;
      result = get_sensor_status(status);
      if(result==0) 
      {
        sick_speed = *r;
        std::cerr << "Probing serial speed: success (" << *r << ")" << std::endl;
        
        if(sick_type==LMS)
        {
          get_sick_type(sick_name);
        }
        else
        {
          sick_name = "PLS ";
          sick_name += std::string(status.software_version, 7);
        }
        return 0;
      }
    }
    ++r;
  }
  //std::cerr << "Probing serial speed: failure" << std::endl;
  return -1;
}

/******************
this is taken from Player and adapted to the smartSickInterface

*/
int SickInterface::set_term_speed(int speed)
{
  struct termios term;

#ifdef HAVE_HI_SPEED_SERIAL
  struct serial_struct serial;
  if(this->serial_high_speed_mode == 0)
  {

          // we should check and reset the AYSNC_SPD_CUST flag
          // since if it's set and we request 38400, we're likely
          // to get another baud rate instead (based on custom_divisor)
          // this way even if the previous player doesn't reset the
          // port correctly, we'll end up with the right speed we want
          if (ioctl(this->_fd, TIOCGSERIAL, &serial) < 0)
          {
            //RETURN_ERROR(1, "error on TIOCGSERIAL in beginning");
            printf("ioctl() failed while trying to get serial port info");
          }
          else
          {
            serial.flags &= ~ASYNC_SPD_CUST;
            serial.custom_divisor = 0;
            if (ioctl(this->_fd, TIOCSSERIAL, &serial) < 0)
            {
              //RETURN_ERROR(1, "error on TIOCSSERIAL in beginning");
              printf("ioctl() failed while trying to set serial port info");
            }
          }
  }
#endif
  switch(speed)
  {
    case 9600:
      //PLAYER_MSG0(2, "terminal speed to 9600");
      if( tcgetattr( this->_fd, &term ) < 0 )
        if(verbose) printf("unable to get device attributes");

      cfmakeraw( &term );
      cfsetispeed( &term, B9600 );
      cfsetospeed( &term, B9600 );

      if( tcsetattr( this->_fd, TCSAFLUSH, &term ) < 0 )
        if(verbose) printf("unable to set device attributes");
      break;

    case 38400:
      //PLAYER_MSG0(2, "terminal speed to 38400");
      if( tcgetattr( this->_fd, &term ) < 0 )
        if(verbose) printf("unable to get device attributes");

      cfmakeraw( &term );
      cfsetispeed( &term, B38400 );
      cfsetospeed( &term, B38400 );

      if( tcsetattr( this->_fd, TCSAFLUSH, &term ) < 0 )
        if(verbose) printf("unable to set device attributes");
      break;

    case 500000:
     if(this->serial_high_speed_mode == 0)
     {
        #ifdef HAVE_HI_SPEED_SERIAL
              //if (ioctl(this->_fd, TIOCGSERIAL, &this->old_serial) < 0) {
              if (ioctl(this->_fd, TIOCGSERIAL, &serial) < 0) {
                if(verbose) printf("error on TIOCGSERIAL ioctl");
              }

              //serial = this->old_serial;

              serial.flags |= ASYNC_SPD_CUST;
              serial.custom_divisor = 240/5; // for FTDI USB/serial converter divisor is 240/5

              if (ioctl(this->_fd, TIOCSSERIAL, &serial) < 0) {
                if(verbose) printf("error on TIOCSSERIAL ioctl");
              }

        #else
              printf("sicklms200: Trying to change to 500kbps in serial_high_speed_mode = 0, but no support compiled in, defaulting to 38.4kbps.\n");
        #endif

              // even if we are doing 500kbps, we have to set the speed to 38400...
              // the FTDI will know we want 500000 instead.

              if( tcgetattr( this->_fd, &term ) < 0 )
                if(verbose) printf("unable to get device attributes");

              cfmakeraw( &term );
              cfsetispeed( &term, B38400 );
              cfsetospeed( &term, B38400 );

              if( tcsetattr( this->_fd, TCSAFLUSH, &term ) < 0 )
                printf("unable to set device attributes");
        }
        else if(this->serial_high_speed_mode == 1)
        {
/* TODO we don't need this !!!

                printf("ChangeTermSpeed() -- serial_high_speed_mode == 1\n");
                tcflush(this->_fd, TCIFLUSH);
                close(this->_fd);
                usleep(1000000);
                this->_fd = ::open(this->device_name, O_RDWR | O_NOCTTY | O_SYNC);
                if(this->_fd < 0)
                        printf("error opening");

                if( tcgetattr( this->_fd, &term ) < 0 )
                         printf("unable to get device attributes");

                term.c_cflag = this->serial_high_speed_baudremap | CS8 | CLOCAL | CREAD;
                      cfmakeraw( &term );
                      cfsetispeed( &term, this->serial_high_speed_baudremap );
                      cfsetospeed( &term, this->serial_high_speed_baudremap );
                tcflush(this->_fd, TCIFLUSH);
                tcsetattr(this->_fd, TCSANOW, &term);
                tcflush(this->_fd, TCIFLUSH);
*/
        }
        break;

        default:
              printf("unknown speed %d", speed);
  }
  return 0;
}






////////////////////////////////////////////////////////////////////////////////
// Get the laser type
//
int SickInterface::get_sick_type(std::string &sick_type)
{
  unsigned char packet[1];
  packet[0] = 0x3A;

  if(verbose) std::cerr << "sending get type request to laser" << std::endl;
  if(_send(packet, 1) < 1) 
  {
    std::cerr << "ERROR: failed to send packet" << std::endl;
    return -1;
  }

  // Wait for laser to return data
  // This could take a while...
  //
  if(verbose) std::cerr << "waiting for reply" << std::endl;
  if(_receive_ack())
  {
    //std::cerr << "ERROR: ACK expected, but not received" << std::endl;
    return -1;
  }
  int len = _receive(false, 3000);
  if (len < 1) 
  {
    if(verbose) std::cerr << "ERROR: no reply from laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] == NACK)
  {
    if(verbose) std::cerr << "ERROR: request denied by laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] != 0xBA)
  {
    if(verbose) std::cerr << "ERROR: unexpected packet type" << std::endl;
    return -1;
  }

  sick_type.resize(len - 2);
  for(int i=0; i<len - 2; ++i)
  {
    sick_type[i] = buffer[PACKET_DATA_START + i];
  }
  return 0;
}



////////////////////////////////////////////////////////////////////////////////
// Get the sensor status
//
int SickInterface::get_sensor_status(SensorStatus &status)
{
  unsigned char packet[1];
  packet[0] = 0x31;

  if(verbose) std::cerr << "sending get sensor status request to laser" << std::endl;
  if(_send(packet, 1) < 1) 
  {
    std::cerr << "ERROR: failed to send packet" << std::endl;
    return -1;
  }

  // Wait for laser to return data
  // This could take a while...
  //
  if(verbose) std::cerr << "waiting for reply" << std::endl;
  if(_receive_ack())
  {
    if(verbose) std::cerr << "ERROR: ACK expected, but not received" << std::endl;
    return -1;
  }
  int len = _receive(false, 3000);
  if (len < 1) 
  {
    if(verbose) std::cerr << "ERROR: no reply from laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] == NACK)
  {
    if(verbose) std::cerr << "ERROR: request denied by laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] != 0xB1)
  {
    if(verbose) std::cerr << "ERROR: unexpected packet type" << std::endl;
    return -1;
  }

  for(unsigned int i=0; i<7; ++i)
  {
    status.software_version[i] = buffer[PACKET_DATA_START + i];
  }

  status.sensor_ok = (buffer[PACKET_DATA_START + 8] == 0);

  return 0;
}



////////////////////////////////////////////////////////////////////////////////
// Set the laser configuration
//
int SickInterface::set_laser_config()
{
  // 2. set the configuration of the scanner (from parameter file)
  //    -length unit
  //    -angular resolution
  //    -reflector reporting
  //    -serial speed

  unsigned int num_retry;
  
  if(sick_type==LMS)
  {
    num_retry = 0;
    while(true)
    {
      if(set_laser_config_mode()==0) break;
      if(++num_retry>=MAX_RETRIES) return -1;
    }

    num_retry = 0;
    while(true)
    {
      if(set_laser_length_unit_and_intensity()==0) break;
      if(++num_retry>=MAX_RETRIES) return -1;
    }

    num_retry = 0;
    while(true)
    {
      if(set_laser_resolution()==0) break;
      if(++num_retry>=MAX_RETRIES) return -1;
    }
  }
  else
  {
    if(verbose)
    {
      std::cerr << "using fixed values for PLS:" << std::endl;
      std::cerr << "  1bit intensity, 1cm length unit, 0.5deg resolution, 180deg width" << std::endl;
    }
    length_unit = 10;
    resolution = 50;
    width = 180;
  }

  num_retry = 0;
  while(true)
  {
    if(set_laser_speed()==0) break;
    if(++num_retry>=MAX_RETRIES) return -1;
  }

  num_retry = 0;
  while(true)
  {
    if(set_term_speed(term_bitrate)==0) break;
    if(++num_retry>=MAX_RETRIES) 
    {
      if(term_bitrate==500000)
      {
        std::cerr << "WARNING: failed to set terminal speed to 500000, maybe you want to connect your scanner to a different serial interface now." << std::endl;
        break;
      }
      else
      {
        return -1;
      }
    }
  }

  return 0;
}



////////////////////////////////////////////////////////////////////////////////
// Put the laser into configuration mode
//
int SickInterface::set_laser_config_mode()
{
  unsigned char packet[10];
  packet[0] = 0x20; // mode change command
  packet[1] = 0x00; // configuration mode
  packet[2] = 0x53; // S - the password 
  packet[3] = 0x49; // I
  packet[4] = 0x43; // C
  packet[5] = 0x4B; // K
  packet[6] = 0x5F; // _
  packet[7] = 0x4C; // L
  packet[8] = 0x4D; // M
  packet[9] = 0x53; // S

  if(verbose) std::cerr << "sending configuration mode request to laser" << std::endl;
  if (_send(packet, 10) < 10) 
  {
    std::cerr << "ERROR: failed to send packet" << std::endl;
    return -1;
  }

  // Wait for laser to return ack
  // This could take a while...
  //
  if(verbose) std::cerr << "waiting for acknowledge" << std::endl;
  if(_receive_ack())
  {
    if(verbose) std::cerr << "ERROR: ACK expected, but not received" << std::endl;
    return -1;
  }
  if(verbose) if(verbose) std::cerr << "configuration mode request ok" << std::endl;
  return 0;
}



////////////////////////////////////////////////////////////////////////////////
// Set the laser configuration
// Returns 0 on success
//
int SickInterface::set_laser_length_unit_and_intensity()
{
  unsigned char packet[1];
  packet[0] = 0x74;

  if(verbose) std::cerr << "sending get configuration request to laser" << std::endl;
  if (_send(packet, 1) < 1)
  {
    std::cerr << "ERROR: failed to send packet" << std::endl;
    return -1;
  }

  // Wait for laser to return data
  // This could take a while...
  //
  if(verbose) std::cerr << "waiting for reply" << std::endl;
  if(_receive_ack())
  {
    if(verbose) std::cerr << "ERROR: ACK expected, but not received" << std::endl;
    return -1;
  }
  int len = _receive(false, 3000);
  if (len < 1)
  {
    if(verbose) std::cerr << "ERROR: no reply from laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] == NACK)
  {
    if(verbose) std::cerr << "ERROR: request denied by laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] != 0xF4)
  {
    if(verbose) std::cerr << "ERROR: unexpected packet type" << std::endl;
    return -1;
  }

  if(verbose) std::cerr << "get configuration request ok" << std::endl;

  // Modify the configuration and send it back
  //
  buffer[PACKET_CMD] = 0x77;

  buffer[PACKET_INTENSITY] = 0x01; // return intensity (values 0..7) in top 3 data bits
  
  if(length_unit==1)        buffer[PACKET_LENGTH_UNIT] = 0x01;
  else if(length_unit==10)  buffer[PACKET_LENGTH_UNIT] = 0x00;
  else if(length_unit==100) buffer[PACKET_LENGTH_UNIT] = 0x02;
  else
  {
    std::cerr << "WARNING: unknown length unit (" << length_unit << "mm)" << std::endl;
    buffer[PACKET_LENGTH_UNIT] = 0x01;
  }
  
  if(verbose) std::cerr << "sending set configuration request to laser" << std::endl;
  len = len - HEADER_LENGTH - 1;
  if (_send(buffer + HEADER_LENGTH, len) < len)
  {
    std::cerr << "ERROR: failed to send packet" << std::endl;
    return -1;
  }

  // Wait for the change to "take"
  //
  if(verbose) std::cerr << "waiting for acknowledge" << std::endl;
  if(_receive_ack())
  {
    if(verbose) std::cerr << "ERROR: ACK expected, but not received" << std::endl;
    return -1;
  }
  if(verbose) std::cerr << "waiting for reply (may take up to 12sec)" << std::endl;
  len = _receive(false, 15000);
  if (len < 1)
  {
    if(verbose) std::cerr << "ERROR: no reply from laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] == NACK)
  {
    if(verbose) std::cerr << "ERROR: request denied by laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] != 0xF7)
  {
    if(verbose) std::cerr << "ERROR: unexpected packet type" << std::endl;
    return -1;
  }

  if(verbose) std::cerr << "set configuration request ok" << std::endl;
  
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Change the resolution of the laser
// Valid widths are: 100, 180 (degrees)
// Valid resolitions are: 25, 50, 100 (1/100 degree)
//
int SickInterface::set_laser_resolution()
{
  if((resolution!=25) && (resolution!=50) && (resolution!=100))
  {
    std::cerr << "WARNING: unknown resolution (" << 0.01*resolution << "deg), using default (0.5deg)" << std::endl;
    resolution = 50;
  }
  if((width!=100) && (width!=180))
  {
    std::cerr << "WARNING: unknown scan width (" << width << "deg)";
    if(resolution!=25)
    {
      std::cerr << ", using default (180deg)" << std::endl;
      width = 180;
    }
    else
    {
      std::cerr << ", using default (100deg) for 0.25deg resolution" << std::endl;
      width = 100;
    }
  }
  if((resolution==25) && (width==180))
  {
    std::cerr << "WARNING: 0.25deg resolution requires 100deg width, will use reduced width"  << std::endl;
    width = 100;
  }

  unsigned char packet[5];
  packet[0] = 0x3B;
  packet[1] = ( width       & 0xFF);
  packet[2] = ((width >> 8) & 0xFF);
  packet[3] = ( resolution       & 0xFF);
  packet[4] = ((resolution >> 8) & 0xFF);

  if(verbose) std::cerr << "sending set variant request to laser" << std::endl;
  if (_send(packet, 5) < 5)
  {
    std::cerr << "ERROR: failed to send packet" << std::endl;
    return -1;
  }

  // Wait for laser to return data
  // This could take a while...
  //
  if(verbose) std::cerr << "waiting for reply" << std::endl;
  if(_receive_ack())
  {
    if(verbose) std::cerr << "ERROR: ACK expected, but not received" << std::endl;
    return -1;
  }
  int len = _receive(false, 3000);
  if (len < 1)
  {
    if(verbose) std::cerr << "ERROR: no reply from laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] == NACK)
  {
    if(verbose) std::cerr << "ERROR: request denied by laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] != 0xBB)
  {
    if(verbose) std::cerr << "ERROR: unexpected packet type" << std::endl;
    return -1;
  }

  // See if the request was accepted
  //
  if (buffer[PACKET_DATA_START] == 0)
  {
    if(verbose) std::cerr << "WARNING: variant request ignored" << std::endl;
  }


  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Set the laser data rate
// Valid values are 9600 and 38400
// Returns 0 on success
//
int SickInterface::set_laser_speed()
{
  unsigned char packet[2];
  packet[0] = 0x20;
  if(bitrate==9600) packet[1] = 0x42; 
  else if(bitrate==19200) packet[1] = 0x41;
  else if(bitrate==38400) packet[1] = 0x40;
  else if(bitrate==500000) packet[1] = 0x48;
  else
  {
    std::cerr << "ERROR: unknown serial speed (" << bitrate << "bps)" << std::endl;
    return -1;
  }

  if(verbose) std::cerr << "sending baud rate request to laser" << std::endl;
  if(_send(packet, 2) < 2)
  {
    std::cerr << "ERROR: failed to send packet" << std::endl;
    return -1;
  }
  
  if(sick_type==LMS)
  {
    // Wait for laser to return ack
    // This could take a while...
    //
    if(verbose) std::cerr << "waiting for acknowledge" << std::endl;
    if(_receive_ack())
    {
      if(verbose) std::cerr << "ERROR: ACK expected, but not received" << std::endl;
      return -1;
    }
    const int len = _receive(false, 3000);
    if (len < 1)
    {
      if(verbose) std::cerr << "ERROR: no reply from laser" << std::endl;
      return -1;
    }
    else if (buffer[PACKET_CMD] == NACK)
    {
      if(verbose) std::cerr << "ERROR: request denied by laser" << std::endl;
      return -1;
    }
    else if (buffer[PACKET_CMD] != 0xA0)
    {
      if(verbose) std::cerr << "ERROR: unexpected packet type" << std::endl;
      return -1;
    }
    else if(buffer[PACKET_DATA_START] != 0x00)
    {
      if(verbose) std::cerr << "ERROR: scanner reports problem (" << int(buffer[PACKET_DATA_START]) << ") with changing baud rate " << std::endl;
      return -1;
    }
    if(verbose) std::cerr << "baud rate request ok" << std::endl;
  }

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Request continuous laser data
//
int SickInterface::request_laser_data()
{
  unsigned char packet[2];
  packet[0] = 0x20; // mode change command
  packet[1] = 0x24; // all points, continuously

  if(verbose) std::cerr << "sending scan data request to laser" << std::endl;
  if (_send(packet, 2) < 2)
  {
    std::cerr << "ERROR: failed to send packet" << std::endl;
    return -1;
  }

  // Wait for laser to return ack
  // This should be fairly prompt
  //
  if(verbose) std::cerr << "waiting for acknowledge" << std::endl;
  if(_receive_ack())
  {
    if(verbose) std::cerr << "ERROR: ACK expected, but not received" << std::endl;
    return -1;
  }
  int len = _receive(false, 3000);
  if (len < 1)
  {
    if(verbose) std::cerr << "ERROR: no reply from laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] == NACK)
  {
    if(verbose) std::cerr << "ERROR: request denied by laser" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_CMD] != 0xA0)
  {
    if(verbose) std::cerr << "ERROR: unexpected packet type" << std::endl;
    return -1;
  }
  else if (buffer[PACKET_DATA_START] != 0x00)
  {
    if(verbose) std::cerr << "ERROR: scanner reports problem (" << int(buffer[PACKET_DATA_START]) << ") with changing mode" << std::endl;
    return -1;
  }
  if(verbose) std::cerr << "scan data request ok" << std::endl;
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Read range data from laser
//
int SickInterface::read_laser_data()
{
  // Read a packet from the laser
  //
  int len = _receive();
  if (len < 1)
  {
    if(verbose) std::cerr << "empty packet" << std::endl;
    return -1;
  }

  // Process raw packets
  //
  if (buffer[PACKET_CMD] == 0xB0)
  {
    if(verbose) std::cerr << "received full scan" << std::endl;
    _scan_data_start = PACKET_DATA_START;
    if(verbose) 
    {
      std::cerr << std::hex;
      for(unsigned int i = 0; i < 6; ++i)
      {
        std::cerr << " 0x" << std::setw(2) << std::setfill('0') << int(buffer[_scan_data_start + i]);
      }
      std::cerr << std::dec << std::endl;
    }
  }
  else if (buffer[PACKET_CMD] == 0xB7) // reduced width
  {
    if(verbose) std::cerr << "received partial scan" << std::endl;
    // bytes 0+1 = index of first point
    // bytes 2+3 = index of last point
    // bytes 4 ... as in standard case
    _scan_data_start = PACKET_DATA_START + 4;
  }
  else
  {
    std::cerr << "ERROR: unexpected packet type" << std::endl;
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Send a packet to the scanner.
//
int SickInterface::_send(unsigned char *data, unsigned int len)
{
  if(HEADER_LENGTH + len + 2 > BUFFER_SIZE)
  {
    std::cerr << "ERROR: maximum packet length exceeded." << std::endl;
    return -1;
  }

  // create header...
  buffer[0] = STX;
  buffer[1] = 0;
  buffer[2] = ( len       & 0xff);
  buffer[3] = ((len >> 8) & 0xff);

  // ...body...
  memcpy(buffer + 4, data, len);

  // ...and crc.
  const unsigned short crc = _calculate_crc(buffer, 4 + len);
  buffer[4 + len + 0] = ( crc       & 0xff);
  buffer[4 + len + 1] = ((crc >> 8) & 0xff);

  // Make sure both input and output queues are empty
  ::tcflush(_fd, TCIOFLUSH);
    
  // Write the data to the port
  const int num_sent = ::write(_fd, buffer, 4 + len + 2);

  if(verbose)
  {
    std::cerr << "sent" << std::hex;
    for(unsigned int i=0; i< 4 + len + 2; ++i)
    {
      std::cerr << " 0x" << std::setw(2) << std::setfill('0') << int(buffer[i]);
    }
    std::cerr << std::dec << std::endl;
  }

  // Make sure the queue is drained
  // Synchronous IO doesnt always work
  ::tcdrain(_fd);

  // Return the actual number of bytes sent, including header and footer
  return (num_sent>0)? num_sent - 6 : num_sent;
}


////////////////////////////////////////////////////////////////////////////////
// Read a packet from the laser
// Set ack to true to ignore all packets except ack and nack
// Set timeout to -1 to make this blocking, otherwise it will return in timeout ms.
// Returns the packet length (-1 if timeout occurs)
//
// Layout of a packet:
//      byte    contents
//        0     0x02 startbyte STX
//        1     address, (LMS address|0x80) for response
//        2     length n (low byte)
//        3     length n (hi byte)
//        4     command
//        5     (data)
//       ...
//        2+n   (data)
//        3+n   status
//        4+n   crc (low byte)
//        5+n   crc (hi byte)
// The length n contains command, data, and status,
// not the header (4 bytes) and not the crc (2 bytes)
//
int SickInterface::_receive(bool ack, int timeout)
{
  // get current time as reference for timeout
  timeval start_time;
  gettimeofday(&start_time, 0);

  // clear the first 5 bytes of the buffer, they
  // will be used as a ring buffer until we find a valid header and command
  //
  for(unsigned int i=0; i<5; ++i) buffer[i] = 0;

if(verbose) std::cerr << "receiving:" << std::hex;

  // read until we get a valid header and command byte, or we time out
  //
  while (true)
  {
    const int status = _wait_for_data(_fd, timeout, start_time);
    if(status>0)
    {
      // read one byte into the last field of the header
      // (header is used as a cyclic buffer)
      //
      ::read(_fd, buffer + 4, 1);

if(verbose) std::cerr << " 0x" << std::setw(2) << std::setfill('0') << int(buffer[4]);

      // check if we got ACK or NACK
      if((buffer[4] == ACK) || (buffer[4] == NACK))
      {
        // ok, get timestamp and return.
        gettimeofday(&_receive_time, 0);
if(verbose) std::cerr << std::dec << std::endl;
        usleep(60000);
        return 1;
      }
      if(!ack) // we're interested in any packet (not only ACK/NACK)
      {
        // check if we have a valid header
        if ((buffer[0] == STX) && (buffer[1] & 0x80)) // valid header
        {
          // we're interested in any packet (not only ACK/NACK),
          // therefore this header is ok.
          break;
        }
        // else no break: go on, listen for the next packet
      }
      // shift buffer to the left by one position
      for(unsigned int i=0; i<4; ++i) buffer[i] = buffer[i+1];
    }
    else // nothing to read, timed out
    {
if(verbose) std::cerr << std::dec << std::endl;
      //std::cerr << "ERROR: timeout" << std::endl;
      return -1;
    }
  }
  gettimeofday(&_receive_time, 0);

if(verbose) std::cerr << " (header found) ";

  // Determine data length
  // Includes status, but not CRC
  //
  unsigned int len = ((unsigned int) buffer[2] | (((unsigned int) buffer[3] << 8)));

  // Check for buffer overflows
  //
  if (len > BUFFER_SIZE - HEADER_LENGTH)
  {
if(verbose) std::cerr << std::dec << std::endl;
    std::cerr << "ERROR: buffer overflow (len > maxlen)" << std::endl;
    return -1;
  }

  // Read in the data,
  // we already received the cmd byte:
  //
  unsigned int i = 1;
  while (i < len)
  {
    const int status = _wait_for_data(_fd, timeout, start_time);
    if(status>0)
    {
const unsigned int old_i = i;
      i += ::read(_fd, buffer + HEADER_LENGTH + i, len - i);

if(verbose)
{
  for(unsigned int c = old_i; c < i; ++c)
  {
    std::cerr << " 0x" << std::setw(2) << std::setfill('0') << int(buffer[HEADER_LENGTH + c]);
  }
}

    }
    else // nothing to read, timed out
    {
if(verbose) std::cerr << std::dec << std::endl;
      std::cerr << "ERROR: timeout" << std::endl;
      return -1;
    }
  }

  // Read in crc
  //
  unsigned char crc_buffer[2];
  i = 0;
  while (i < 2)
  {
    const int status = _wait_for_data(_fd, timeout, start_time);
    if(status>0)
    {
const unsigned int old_i = i;
      i += ::read(_fd, crc_buffer + i, 2 - i);

if(verbose)
{
  for(unsigned int c = old_i; c < i; ++c)
  {
    std::cerr << " 0x" << std::setw(2) << std::setfill('0') << int(buffer[HEADER_LENGTH + c]);
  }
}

    }
    else // nothing to read, timed out
    {
if(verbose) std::cerr << std::dec << std::endl;
      std::cerr << "ERROR: timeout" << std::endl;
      return -1;
    }
  }
  const unsigned short received_crc = ((unsigned short)crc_buffer[0]) | (((unsigned short)crc_buffer[1]) << 8);

if(verbose) std::cerr << std::dec << std::endl; // packet read

  // check the CRC
  //
  const unsigned short calculated_crc = _calculate_crc(buffer, 4 + len);
  if(received_crc!=calculated_crc)
  {
    std::cerr << "ERROR: CRC error, ignoring packet" << std::endl;
    return -1;
  }
  return len;
}

           
////////////////////////////////////////////////////////////////////////////////
// Wait for ACK
//
int SickInterface::_receive_ack()
{
  if(verbose) std::cerr << "waiting for ACK..." << std::endl;
  const int result = _receive(true, 90); // SICK: "max response time for ACK/NACK = 60ms"
  if((result==1) && (buffer[PACKET_CMD]==ACK)) 
  {
    if(verbose) std::cerr << "ACK received." << std::endl;
    return 0;
  }
  if(verbose) std::cerr << "Failed to receive ACK." << std::endl;
  return -1;
}

////////////////////////////////////////////////////////////////////////////////
// Wait for incoming data on filedescriptor fd.
//
// Arguments:
//   timeout: wait up to timeout msec if timeout>=0,
//            wait forever if timeout<0
//
//   start_time: start time for timeout countdown 
//
// Returns:
//   >0 : did not time out, data available for reading
//   =0 : timed out
//   <0 : error (bad filedescriptor)
//
int SickInterface::_wait_for_data(int fd, int timeout, const timeval &start_time) const
{
//  if(timeout<0) return 1; // let read() block

  fd_set read_fdset;
  
  FD_ZERO(&read_fdset);
  FD_SET(_fd, &read_fdset);
  
  if(timeout>=0)
  {
    timeval remaining_time = _remaining_time(start_time, timeout);
    return select(_fd + 1, &read_fdset, 0, 0, &remaining_time);
  }
  else // no timeout for select
  {
    return select(_fd + 1, &read_fdset, 0, 0, 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Create a CRC for the given packet
//
unsigned short SickInterface::_calculate_crc(unsigned char* data, unsigned int len) const
{
  unsigned short uCrc16;
  unsigned char abData[2];
  
  uCrc16 = 0;
  abData[0] = 0;
  
  while(len--)
  {
    abData[1] = abData[0];
    abData[0] = *data++;
    
    if( uCrc16 & 0x8000 )
    {
      uCrc16 = (uCrc16 & 0x7fff) << 1;
      uCrc16 ^= CRC16_GEN_POL;
    }
    else
    {    
      uCrc16 <<= 1;
    }
    uCrc16 ^= ((unsigned short)(abData[0])) | (((unsigned short)(abData[1])) << 8);
  }
  return (uCrc16); 
}

timeval SickInterface::_remaining_time(const timeval &start_time, int timeout) const
{
  timeval now, remaining;
  gettimeofday(&now, 0);

  const int diff_msec = (now.tv_sec - start_time.tv_sec)*1000 + (now.tv_usec - start_time.tv_usec)/1000;

  const int remaining_msec = timeout - diff_msec;
  if(remaining_msec<=0)
  {
    remaining.tv_sec  = 0;
    remaining.tv_usec = 0;
  }
  else
  {
    remaining.tv_sec  = remaining_msec / 1000;
    remaining.tv_usec = (remaining_msec % 1000) * 1000;
  }

  return remaining;
}
