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
\file    cobot_drive.cpp
\brief   C++ Implementation: LaserScanner
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "laser_scanner.h"

const double LaserScanner::ScanTimeout(0.12);

LaserScanner::LaserScanner()
{
  bufferIndex = 0;
  expectedPacketLength = 0;
  packetDataStartIdx = 0;
  scanComplete = true;
  dataAvailable = false;
  tScanStart = 0.0;
}

LaserScanner::~LaserScanner()
{
}

void LaserScanner::init(const char *serialPort)
{
  laserSerial.open(serialPort,115200);
  Sleep(0.1);
  char cmd[] = "L1\n";  //Turn on laser
  laserSerial.write(cmd,3);
}

void LaserScanner::SerialReceive()
{

  static const bool debug = false;
  unsigned int i = 0, j = 0, k = 0;
  bool packetComplete = false;

  packetDataStartIdx = 12;
  int start = 0, end = 767;
  int numPoints = (end-start+1);
  expectedPacketLength = packetDataStartIdx +  numPoints*2 + ceil(numPoints/32) + 1;

  int maxSize = ReceiveBufferSize-bufferIndex;
  if(debug) printf("ms:%d el:%d\n",maxSize, expectedPacketLength);
  int numRead = laserSerial.read(&receiveBuffer[bufferIndex],maxSize);

  if(numRead<=0){
    return;
  }else{
    if(debug) printf("%d bytes read\n",numRead);
    bufferIndex += numRead;
  }

  if(bufferIndex>=ReceiveBufferSize){
    char text[256];
    sprintf(text,"Serial Buffer overflow, BytesRead=%d BufferIndex=%d",numRead, bufferIndex);
    TerminalWarning(text);
    bufferIndex = ReceiveBufferSize-1;
  }

  for(i=0;i<bufferIndex-1;i++){
    // if(debug) printf("%c ",receiveBuffer[i]);
    if(receiveBuffer[i]==10 && this->receiveBuffer[i+1]==10){
      packetComplete = true;
      break;
    }
  }
  if(debug) printf("\n");

  if(packetComplete){
    if(debug) printf("Packet complete!\n");
    for(j=packetDataStartIdx,k=0;j<i&&k<768;j++)
    {
      if(this->receiveBuffer[j]==10)
        continue;
      scanData[k]=0.001*double( (unsigned int) (((receiveBuffer[j]-0x30)&0x3F)<<6) + ((receiveBuffer[j+1]-0x30)&0x3F) );
      k++;
      j++;//j is incremented twice since one scan data is two chars
    }
    if(debug){
      printf("Range Center:");
      for(int i=382;i<386;i++)
        printf(" %f",scanData[i]);
      printf("\n");
			printf("%d rays read\n",k);
    }
    scanComplete = true;
		if(k>767)
	    dataAvailable = true;
    bufferIndex = 0;
    memset(receiveBuffer, 0, ReceiveBufferSize * sizeof(unsigned char));
  }else{
    if(bufferIndex>expectedPacketLength){
      TerminalWarning("Corrupt Data Packet!");
      bufferIndex = 0;
      scanComplete = true;
      dataAvailable = false;
    }else if(GetTimeSec()-tScanStart>ScanTimeout){
      TerminalWarning("Scan timed out!");
      bufferIndex = 0;
      scanComplete = true;
      dataAvailable = false;
    }
  }
}
int LaserScanner::getScanData (double _scanData [NumScanRays] )
{
  if(dataAvailable){
    memcpy(_scanData,scanData,NumScanRays*sizeof(double));
    dataAvailable = false;  //To make sure that the same data is not read multiple times
    return(NumScanRays);
  }else{
    return(0);
  }
}

void LaserScanner::SerialSend()
{
  if(scanComplete){
    //Send request for new packet
    char cmd[] = "G00076700\n";
    laserSerial.write(cmd,10);
    tScanStart = GetTimeSec();
    scanComplete = false;
  }
}

void LaserScanner::run()
{
  SerialSend();
  SerialReceive();
}

void LaserScanner::close()
{
  char cmd[] = "L0\n";  //Turn off laser
  laserSerial.write(cmd,3);
  Sleep(0.1);
  laserSerial.close();
}

