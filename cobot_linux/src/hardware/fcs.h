#ifndef _FCS_H_
#define _FCS_H_

#define false 0
#define true 1

unsigned short fcs16(unsigned short fcs, char* cp, int len);
int addfcs(char* dat, int len);
unsigned char checkfcs(char* dat, int len);


#endif


