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
\file    laser_scanner.h
\brief   C++ Interface: LaserScanner
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "math.h"
#include <stdint.h>
#include "fcs.h"
#include "serial.h"
#include "geometry.h"
#include "timer.h"
#include "terminal_utils.h"

#define CMD_G 'G'
#define CMD_V 'V'
#define CMD_L 'L'

class LaserScanner
{
  Serial laserSerial;
  static const unsigned int ReceiveBufferSize = 2048;
  static const unsigned int NumScanRays = 768;
  static const double ScanTimeout;
  unsigned char receiveBuffer[ReceiveBufferSize];
  double scanData[NumScanRays];
  unsigned int bufferIndex;
  unsigned int expectedPacketLength;
  unsigned int packetDataStartIdx;
  bool scanComplete;
  bool dataAvailable;
  double tScanStart;

  private:
  void SerialReceive();
  void SerialSend();

  public:
  LaserScanner();
  ~LaserScanner();
  void init(const char *serialPort);
  void run();
  void close();
  int getScanData(double _scanData[NumScanRays]);
};