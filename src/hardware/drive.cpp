//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
 * \file    drive.cpp
 * \brief   C++ Implementation: CobotDrive
 * \author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include "drive.h"

#include <cmath>

#include "shared/util/helpers.h"

using std::isfinite;

AccelLimits &AccelLimits::operator*=(double f) {
  max_accel  *= f;
  max_deccel *= f;
  max_vel    *= f;
  return(*this);
}

AccelLimits &AccelLimits::set(const AccelLimits &al,double f) {
  max_accel  = al.max_accel  * f;
  max_deccel = al.max_deccel * f;
  max_vel    = al.max_vel    * f;
  return(*this);
}

CobotDrive::CobotDrive(motor_properties_t _motorProps) {
  //Motor properties
  motorProps = _motorProps;

  //Status variables
  status=0;
  adc = 0.0;
  tStamp = 0;

  //Motion limits
  transLimits.set(1.0,2.0,2.5);
  rotLimits.set(1.0*M_PI,1.0*M_PI,1.5*M_PI);

  //Drive commands
  lastSerialSendT = 0.0;
  lastTransSpeed.zero();
  desiredTransSpeed.zero();
  lastRotSpeed = 0.0;
  desiredRotSpeed = 0.0;
  enableMotion = true;

  //Odometry variables
  odometryR = odometryX = odometryY = 0.0;
  v0 = v1 = v2 = v3 = 0.0;
  currentRSpeed = currentXSpeed = currentYSpeed = 0.0;
  newOdometryAvailable = false;
  curLoc.zero();
}

void CobotDrive::setLimits(AccelLimits _transLimits, AccelLimits _rotLimits) {
  transLimits = _transLimits;
  rotLimits = _rotLimits;
}


CobotDrive::~CobotDrive() {
}

void CobotDrive::close() {
  robotSerial.close();
}

void CobotDrive::setSpeeds(float x, float y, float r) {
  static const double maxTransSpeed = 3.5;
  static const double maxRotSpeed = RAD(360.0);
  if (!isfinite(x) || !isfinite(y) || !isfinite(r) ||
      (sq(x)+sq(y)>sq(maxTransSpeed)) || fabs(r)>maxRotSpeed) {
    char text[1024];
    sprintf(text,"Invalid command: %f %f %f\n",x,y,r);
    TerminalWarning(text);
    x = y = r = 0;
  }
  lastCommandT = GetTimeSec();
  desiredTransSpeed.set(x,y);
  desiredRotSpeed = r;
}

uint32_t CharArrayToInt(unsigned char *n) {
  uint32_t a = 0;
  for(int i=0;i<4;i++) {
    a += (n[3-i]<<(8*i));
  }
  return a;
}

double IntDiff(uint32_t _a, uint32_t _b) {
  int64_t a = _a, b = _b;
  double diff = 0;
  if ( ( (b & 0xC0000000ULL) == 0xC0000000ULL) &&
      ( (a & 0xC0000000ULL) == 0x00000000ULL) ) {
    //overflow
    diff = (0x100000000LL - b) + a;
  }else if ( ( (b & 0xC0000000ULL) == 0x00000000ULL) &&
      ( (a & 0xC0000000ULL) == 0xC0000000ULL) ) {
    //underflow
    diff = -(0x100000000LL - a) - b;
  }else {
    diff = a - b;
  }

  return diff;
}

void CobotDrive::EncoderUpdate(
    uint32_t e0, uint32_t e1, uint32_t e2, uint32_t e3, double t) {
  static const bool debug = false;
  static uint32_t  e0last=0, e1last=0, e2last=0, e3last=0;
  static double tlast=0.0;
  double  e0diff=0, e1diff=0, e2diff=0, e3diff=0;
  double tdiff=0;

  //Override t, use current time instead
  t = GetTimeSec();

  if (tlast<DBL_MIN) {
    //This is the first time the function is being called, can't estimate velocities right now
    tlast=t;
    e0last=e0;
    e1last=e1;
    e2last=e2;
    e3last=e3;
    return;
  }

  e0diff = IntDiff(e0,e0last);
  e1diff = IntDiff(e1,e1last);
  e2diff = IntDiff(e2,e2last);
  e3diff = IntDiff(e3,e3last);
  tdiff = t-tlast;

  if (tdiff<DBL_MIN) {
    //Redundant update in the same time step?
    return;
  }

  double x = (e0diff+e1diff-e2diff-e3diff)/(4.0*motorProps.encoderCountsPerMeter.x);
  double y = (e0diff-e1diff-e2diff+e3diff)/(4.0*motorProps.encoderCountsPerMeter.y);
  if (motorProps.xyFlipped)
    swap(x,y);

  odometryR = -(e0diff+e1diff+e2diff+e3diff)/(4.0*motorProps.encoderCountsPerRadian);
  odometryX = x;
  odometryY = y;

  curLoc += vector2d(x,y).rotate(curAngle);
  curAngle = angle_mod(curAngle+odometryR);

  v0 = (double) e0diff/tdiff;
  v1 = (double) e1diff/tdiff;
  v2 = (double) e2diff/tdiff;
  v3 = (double) e3diff/tdiff;

  currentRSpeed = odometryR/tdiff;
  currentXSpeed = odometryX/tdiff;
  currentYSpeed = odometryY/tdiff;

  newOdometryAvailable = true;

  if (debug) {
    printf("Odometry: %f %f %f %f\n",v0,v1,v2,v3);
  }

  e0last=e0;
  e1last=e1;
  e2last=e2;
  e3last=e3;
  tlast=t;
}

bool CobotDrive::GetFeedback(
    double& r, double& x, double& y,
    double& _v0, double& _v1, double& _v2, double& _v3,
    double& vr, double& vx, double& vy,
    double& _adc, unsigned char& _status, vector2d &curLoc, double &curAngle) {
  if (newOdometryAvailable) {
    r = odometryR;
    x = odometryX;
    y = odometryY;
    _v0 = v0;
    _v1 = v1;
    _v2 = v2;
    _v3 = v3;
    vr = currentRSpeed;
    vx = currentXSpeed;
    vy = currentYSpeed;
    _adc = adc;
    _status = status;
    newOdometryAvailable = false;
    curLoc = this->curLoc;
    curAngle = this->curAngle;
    return true;
  }else {
    return(false);
  }
}

void CobotDrive::GetDriveRawFeedback(double& vx, double& vy, double& vr) {
 vx = transSpeed.x;
 vy = transSpeed.y;
 vr = rotSpeed;
}

void CobotDrive::SerialReceive() {
  static const int EncoderPacketSize = 29;
  static const bool debug = false;
  static const bool debug_serial = false;
  static const int bufferSize = ReceiveBufferSize;
  static double tLast = GetTimeSec();
  unsigned char buffer[bufferSize+1];
  int i=0;
  const double t_start = debug_serial ? GetTimeSec() : 0.0;
  int len = robotSerial.read(buffer,bufferSize);
  if (debug_serial) {
    const double t_duration = GetTimeSec() - t_start;
    if (len < 0) {
      std::string str = StringPrintf(
          "%f (%6.3F): %d ", GetTimeSec() * 1e6, t_duration, len);
      perror(str.c_str());
    } else {
      printf("%f (%6.3F): %d\n", GetTimeSec() * 1e6, t_duration, len);
    }
  }
  

  if (len>0) {
    //serial data received!
    //Ensure that the string terminates well
    buffer[min(bufferSize,len+1)]=0;
    if (debug) {
      printf("Rcv %d bytes (dT=%.3f) \n",len,GetTimeSec()-tLast);
      tLast = GetTimeSec();
    }
    if (buffer[0]=='m' && buffer[1]=='p') {
      if (false && debug) {
        printf("MP:");
        for(i=0;i<29;i++)
          printf(" %02X",buffer[i]);
        printf("\n");
      }
      if (checkfcs((char*)buffer,EncoderPacketSize)) {
        uint32_t e0=0, e1=0, e2=0, e3=0, t=0;
        e0=CharArrayToInt(&buffer[2]);
        e1=CharArrayToInt(&buffer[6]);
        e2=CharArrayToInt(&buffer[10]);
        e3=CharArrayToInt(&buffer[14]);
        t=CharArrayToInt(&buffer[18]);
        adc=0.04647239263803680982*double(CharArrayToInt(&buffer[22]));
        status=buffer[26];
        if (debug) {
          printf("Encoder Packet: %d %d %d %d %d %f 0x%02X\n",
                 e0,e1,e2,e3,t,adc,status);
        }
        EncoderUpdate(e0,e1,e2,e3,t);
        tStamp++;
      }else {
        if (debug) printf("Corrupt Packet\n");
      }
    } else {
      if (debug) printf("MP incomplete.\n");
    }
  }
}

int CobotDrive::makercpacket(mspcommand_t* command, unsigned char* buf) {
  buf[0] = 'M';
  buf[1] = 'P';
  buf[2] = command->xspeed & 0xFF;
  buf[3] = (command->xspeed >> 8) & 0xFF;
  buf[4] = command->yspeed & 0xFF;
  buf[5] = (command->yspeed >> 8) & 0xFF;
  buf[6] = command->rspeed & 0xFF;
  buf[7] = (command->rspeed >> 8) & 0xFF;

  return addfcs((char*)buf, 8);
}

void CobotDrive::SerialSend() {
  static const bool debug = false;
  static const double CommandTimeout = 0.2;
  mspcommand_t mspcommand;

  if (GetTimeSec()-lastCommandT > CommandTimeout) {
    if (debug) printf("No command received recently, engage safety halt\n");
    //No command received recently, engage safety halt
    desiredTransSpeed.zero();
    desiredRotSpeed = 0.0;
  }

  double dt = GetTimeSec() - lastSerialSendT;

  unsigned char commandBuffer[20];

  if (!enableMotion) {
    rotSpeed = 0.0;
    transSpeed.zero();
  }else {
    //Abide by limits
    if (desiredTransSpeed.sqlength()>sq(transLimits.max_vel))
      desiredTransSpeed = desiredTransSpeed.norm(transLimits.max_vel);
    double dvMax = 0.0;
    if (desiredTransSpeed.sqlength()>lastTransSpeed.sqlength())
      dvMax = dt*transLimits.max_accel;
    else
      dvMax = dt*transLimits.max_deccel;
    vector2d dv = desiredTransSpeed - lastTransSpeed;
    if (dv.sqlength()>sq(dvMax)) {
      dv = dv.norm(dvMax);
    }
    transSpeed = lastTransSpeed + dv;

    if (fabs(desiredRotSpeed)>rotLimits.max_vel)
      desiredRotSpeed = sign(desiredRotSpeed)*rotLimits.max_vel;
    double drMax = dt*rotLimits.max_accel;
    double dr = desiredRotSpeed - lastRotSpeed;
    if (fabs(dr)>drMax) {
      dr = sign(dr)*drMax;
    }
    rotSpeed = lastRotSpeed + dr;
  }

  mspcommand.xspeed =
      bound(-motorProps.transMotionScale.y*transSpeed.y, -30000, 30000);
  mspcommand.yspeed =
      bound(motorProps.transMotionScale.x*transSpeed.x, -30000, 30000);
  mspcommand.rspeed = bound(motorProps.rotMotionScale*rotSpeed, -30000, 30000);
  double packetLen = makercpacket(&mspcommand, commandBuffer);
  if (debug) {
    for(int i=0; i<packetLen; i++)
      printf("0x%02X, ",commandBuffer[i]);
    printf("\n");
  }
  robotSerial.write(commandBuffer,packetLen+1);

  lastRotSpeed = rotSpeed;
  lastTransSpeed = transSpeed;
  lastSerialSendT = GetTimeSec();

}

void CobotDrive::run() {
  //Receive encoder feedback
  SerialReceive();
  //Send motor commands
  SerialSend();
}

void CobotDrive::init(const char *serialPort) {
  if(robotSerial.open(serialPort,115200)==false) exit(0);
}
