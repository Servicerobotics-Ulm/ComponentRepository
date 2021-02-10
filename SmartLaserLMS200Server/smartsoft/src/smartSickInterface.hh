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

#ifndef FAW_SICK_INTERFACE_HH
#define FAW_SICK_INTERFACE_HH

#include <sys/time.h>
#include <unistd.h>

#include <string>

namespace Smart {

/**
  The SickInterface class encapsulates the 
  communication with a SICK laser scanner.
  By not embedding the communication directly into the LaserThread,
  reuse of this more-or-less SmartSoft independant part is facilitated.
 */
class SickInterface
{
public:

  SickInterface();

  //
  // USER PARAMETERS
  // 
  
  // be quiet or not?
  bool verbose;

  // supported scanner types
  enum SickType 
  {
    LMS, 
    PLS
  };
  // the type of used scanner
  SickType sick_type;


  // length unit in [mm]
  // possible values are 1, 10, and 100.
  // corresponding maximum distances are 8m, 80m, and 150m
  unsigned int length_unit;

  // angular resolution [0.01 deg]
  // possible values are 25, 50, 100
  // the 0.25 deg resolution requires the reduced scan width of 100 (see below)
  unsigned int resolution;

  // angular width [deg] of the scan
  // possible values are 180 and 100
  unsigned int width;

  // the bitrate of the connection to the scanner.
  // possible values are 9600, 19200, 38400, and 500000
  unsigned int bitrate;

  // the bitrate which shall be commanded to the serial interface card.
  // this is usefull if your card uses an internal multiplier (like mine).
  unsigned int term_bitrate;

  //
  // SENSOR STATUS
  //
  
  struct SensorStatus
  {
    char software_version[7]; // possibly not null terminated!
    bool sensor_ok;
  };

  //
  // COMMUNICATION METHODS
  //
  
  int open_device(const char *devname);
  int close_device();

  int probe_sick_speed(unsigned int &sick_speed, std::string &sick_type);
  int set_term_speed(int speed);

  /** LMS only. */
  int get_sick_type(std::string &sick_type);

  int get_sensor_status(SensorStatus &status);

  int set_laser_config();
  int set_laser_config_mode();

  /** LMS only */
  int set_laser_length_unit_and_intensity();

  /** LMS only */
  int set_laser_resolution();

  int set_laser_speed();

  int request_laser_data();
  int read_laser_data();

  inline const timeval &get_receive_timestamp() const { return _receive_time; }
  inline unsigned int extract_num_points() const;
  inline unsigned int extract_length_unit() const;

  /**
    Get the maximum distance in length units as used by the scanner,
    which does not represent an overflow.
   */
  inline unsigned int get_max_distance() const;

  /**
    Return the integer distance, to be interpreted with respect to
    the length unit of the scan.
    If no distance has been measured, a value of 8187 (0x1FFB) is returned
    (at least on the scanner where I tested that).
    Update: I saw values of 8183 (0x1FF7) reported for overflows, too.
    Will now use 8, 80, and 150 meters (for 1mm, 1cm, and 10cm resolution) 
    as distance overflow limits, as indicated by the SICK manual.
   */
  inline unsigned int extract_distance(unsigned int i) const;
  inline unsigned int extract_intensity(unsigned int i) const;

  inline bool is_overflow(unsigned int dist) const
  { 
    return (dist > get_max_distance());
  }

private:
  // for setting the baud rate to 500kbaud
  int serial_high_speed_mode;

  static const unsigned char STX = 0x02;
  static const unsigned char ACK = 0x06;
  static const unsigned char NACK = 0x15;
  static const unsigned short int CRC16_GEN_POL = 0x8005;
  static const unsigned int MAX_RETRIES = 5;

  // the filedescriptor of the serial device connecting to the scanner
  int _fd;

  // the timestamp of a successfully received packet
  // only valid after a successful call to _receive(...) 
  timeval _receive_time;

  //
  // Internal low level communication with a SICK scanner
  //

  /**
    Send a packet to the scanner.
   */
  int _send(unsigned char *data, unsigned int count);
  
  /**
    Receive a packet from the scanner.
    The received packet excluding CRC is written to the \c buffer,
    the data length plus header length is returned (number of bytes used in buffer).
   */
  int _receive(bool ack = false, int timeout = -1);

  /**
    Return 0 if ACK is received,
    return -1 if NACK is received or time is over.
   */
  int _receive_ack();

  /**
    Wait for incoming data.
   */
  int _wait_for_data(int fd, int timeout, const timeval &start_time) const;

  /**
    Calculates CRC for a telegram.
   */
  unsigned short _calculate_crc(unsigned char *data, unsigned int len) const;

  //
  // Timeout management
  //
  
  timeval _remaining_time(const timeval &start_time, int timeout) const;

  //
  // global buffer of 64k 
  //

  static const unsigned int BUFFER_SIZE = 65536;
  unsigned char buffer[BUFFER_SIZE];
  static const int HEADER_LENGTH = 4;

  enum PacketPositions
  {
    PACKET_ADDR = 1,
    PACKET_LEN_LO = 2,
    PACKET_LEN_HI = 3,
    PACKET_CMD = 4,
    PACKET_DATA_START = 5,
    PACKET_INTENSITY = 10,
    PACKET_LENGTH_UNIT = 11
  };

  unsigned int _scan_data_start;
};

inline unsigned int SickInterface::extract_num_points() const
{
  return (((unsigned int) buffer[_scan_data_start]) | 
         (((unsigned int) buffer[_scan_data_start+1]) << 8)) & 0x3FFF;
}

inline unsigned int SickInterface::extract_length_unit() const
{
  if(sick_type==PLS) return 10;
  if(buffer[_scan_data_start+1] & 0x40) return 1;
  if(buffer[_scan_data_start+1] & 0x80) return 100;
  return 10;
}

inline unsigned int SickInterface::get_max_distance() const
{
  if(extract_length_unit()==100) return 1500;
  return 8000;
}

inline unsigned int SickInterface::extract_distance(unsigned int i) const
{
  const unsigned int addr = _scan_data_start + 2 + 2*i;
  return ((unsigned int)buffer[addr]) | ((unsigned int)(buffer[addr + 1] & 0x1F) << 8);
}

inline unsigned int SickInterface::extract_intensity(unsigned int i) const
{
  const unsigned int addr = _scan_data_start + 3 + 2*i;
  if(sick_type==LMS) return (buffer[addr] & 0xE0) >> 5;
  return (buffer[addr] & 0x20) >> 5;
}

} // namespace Smart

#endif


