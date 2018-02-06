#include "stdafx.h"
#include "rle_code.h"
#include <vector>

using namespace std;

struct pairR
{
	unsigned char number; 
	char c;
};

bool CRLECode::encode( char* buf, int iN, char** outbuf, int& oN)
{
	int i,j;
	char* pI = buf; 
	vector<pairR> tmp; 
	tmp.reserve(iN);
	for( i=0; i<iN; i++)
	{
		char key = pI[i]; 
		for( j=i+1; j<iN; j++)
		{
			if(pI[j] != key)
				break;
			if(j==i+255)
				break;
		}
		pairR Record; 
		Record.number = j-i;
		Record.c = key;
		tmp.push_back(Record);
	}
	oN = tmp.size()*2; 
	(*outbuf) = (char*)malloc(oN);
	char* pO = *outbuf;
	for(i=0; i<tmp.size(); i++)
	{
		(*pO++) = tmp[i].number;
		(*pO++) = tmp[i].c;
	}
	return true;
}

bool CRLECode::decode(char* buf, int iN, char** outbuf, int& oN)
{
	int N = iN/2;
	int i,j, number;
	char key;
	char* pI = buf;
	(*outbuf) = (char*)malloc(oN);
	char* pO = *outbuf;
	for(i=0; i<N; i++)
	{
		number = (*pI++); 
		key = (*pI++); 
		memset(pO, key, number); 
		pO = pO + number;
	}
	return true;
}

