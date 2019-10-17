//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    drive.h
\brief   C++ Interface: CobotDrive
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#ifndef COBOT_DRIVE
#define COBOT_DRIVE

#include "math.h"
#include <float.h>
#include <stdint.h>
#include "serial.h"
#include "geometry.h"
#include "timer.h"
#include "terminal_utils.h"
#include "fcs.h"

typedef struct
{
  signed short xspeed;
  signed short yspeed;
  signed short rspeed;
} mspcommand_t;

typedef struct
{
  /// Counts accrued after moving straight one meter
  vector2d encoderCountsPerMeter;
  /// Counts accrued after truning by one radian
  double encoderCountsPerRadian;
  /// Velocity command to move straight at 1m/s
  vector2d transMotionScale;
  /// Velocity command to turn at 1rad/s
  double rotMotionScale;
  /// Indicates whether x,y coorinates are swapped or not
  bool xyFlipped;
} motor_properties_t;

class AccelLimits{
  public:
    double max_accel;  // acceleration limit from 0 to max_vel
    double max_deccel; // acceleration limit from max_vel to 0
    double max_vel;    // maximum velocity along dimension
    
  public:
    void set(double a,double d,double v)
    {max_accel=a; max_deccel=d; max_vel=v;}
    
    // return new limits with all parameters scaled by <f>
    AccelLimits operator*(double f) const
    {AccelLimits r; r.set(max_accel*f,max_deccel*f,max_vel*f); return(r);}
    
    // scale all parameters by <f> in-place
    AccelLimits &operator*=(double f);
    
    // set limits to <al> with all parameters scaled by <f>
    AccelLimits &set(const AccelLimits &al,double f);
};


class CobotDrive{
protected:
  vector2d transSpeed;
  double rotSpeed;
  Serial robotSerial;
  static const int ReceiveBufferSize = 1024;
  //unsigned char receiveBuffer[ReceiveBufferSize];
  uint64_t tStamp;
  AccelLimits transLimits, rotLimits;
  double lastSerialSendT;
  vector2d lastTransSpeed, desiredTransSpeed;
  double curAngle;
  vector2d curLoc;
  double lastRotSpeed, desiredRotSpeed;
  bool enableMotion;
  
  double odometryR, odometryX, odometryY;
  double v0, v1, v2, v3;
  double currentRSpeed, currentXSpeed, currentYSpeed;
  bool newOdometryAvailable;
  double lastCommandT;
  unsigned char status;
  double adc;
  motor_properties_t motorProps;
  
private:
  void SerialReceive();
  void SerialSend();
  int makercpacket(mspcommand_t *command, unsigned char *buf);
  void EncoderUpdate(uint32_t e0, uint32_t e1, uint32_t e2, uint32_t e3, double t);
public:
  CobotDrive(motor_properties_t _motorProps);
  ~CobotDrive();
  void setLimits(AccelLimits _transLimits, AccelLimits _rotLimits);
  void init(const char *serialPort);
  void run();
  void close();
  void setSpeeds(float x, float y, float r);
  bool GetFeedback(double &r, double &x, double &y, 
                   double& _v0, double& _v1, double& _v2, double& _v3,
                   double &vr, double &vx, double &vy,
                   double& _adc, unsigned char& _status, vector2d &curLoc, double &curAngle);
  void GetDriveRawFeedback(double& vx, double& vy, double& vr);
};

#endif //COBOT_DRIVE
