#ifndef RLE_CODE_H
#define RLE_CODE_H

class CRLECode
{
public:
	bool encode( char* buf, int iN, char** outbuf, int& oN);
	bool decode( char* buf, int iN, char** outbuf, int& oN);
};

#endif