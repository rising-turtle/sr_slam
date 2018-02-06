#include "stdafx.h"
#include "huffman_code.h"
#include <malloc.h>

const char* SampleString = "this is an example for huffman encoding";
 
INode* CHuffman::BuildTree(const int (&frequencies)[UniqueSymbols])
{
    std::priority_queue<INode*, std::vector<INode*>, NodeCmp> trees;
 
    for (int i = 0; i < UniqueSymbols; ++i)
    {
        if(frequencies[i] != 0)
            trees.push(new LeafNode(frequencies[i], (char)i));
    }
    while (trees.size() > 1)
    {
        INode* childR = trees.top();
        trees.pop();
 
        INode* childL = trees.top();
        trees.pop();
 
        INode* parent = new InternalNode(childR, childL);
        trees.push(parent);
    }
    return trees.top();
}
 
void CHuffman::GenerateCodes(const INode* node, const HuffCode& prefix, HuffCodeMap& outCodes, HuffCodeMapInv& outCodesInv)
{
    if (const LeafNode* lf = dynamic_cast<const LeafNode*>(node))
    {
        outCodes[lf->c] = prefix;
		outCodesInv[prefix] = lf->c;
    }
    else if (const InternalNode* in = dynamic_cast<const InternalNode*>(node))
    {
        HuffCode leftPrefix = prefix;
        leftPrefix.push_back(false);
        GenerateCodes(in->left, leftPrefix, outCodes, outCodesInv);
 
        HuffCode rightPrefix = prefix;
        rightPrefix.push_back(true);
        GenerateCodes(in->right, rightPrefix, outCodes, outCodesInv);
    }
}

bool CHuffman::encode(char* buf, int iN, char** outbuf, int& oN, int (&frequencies)[UniqueSymbols])
{
	unsigned char* ptr = (unsigned char*)buf; 
	for(int i=0; i<iN; i++)
	{
		++frequencies[*ptr++];
		if(frequencies[207] != 0)
		{
			int  a = 10;
		}
	}
	
	INode * root = BuildTree(frequencies); 
	HuffCodeMap codes; 
	HuffCodeMapInv codes_inv;
	GenerateCodes(root, HuffCode(), codes, codes_inv); 
	delete root;

	HuffCode encoded_buf; 
	ptr = (unsigned char*)buf; 
	for(int i=0; i<iN; i++)
	{
		HuffCode& code = codes[*ptr++]; 
		encoded_buf.insert(encoded_buf.end(), code.begin(), code.end());
	}
	int L = encoded_buf.size(); 
	oN = L/8; 
	int addon = L%8; 
	if(addon)
	{
		for(int i=0; i<8-addon; i++) encoded_buf.push_back(false);
		++oN; 
	}
	(*outbuf) = (char*)calloc(oN, sizeof(char));
	char* pbuf = *outbuf;
	for(int i=0; i<oN; i++)
	{
		pbuf[i] = bit2Char(encoded_buf, i*8);
	}
	return true;
}

bool CHuffman::decode( char* buf, int iN, char** outbuf, int& oN, int (&frequencies)[UniqueSymbols])
{
	INode * root = BuildTree(frequencies); 
	HuffCodeMap codes; 
	HuffCodeMapInv codes_inv;
	GenerateCodes(root, HuffCode(), codes, codes_inv); 

	(*outbuf) = (char*)calloc(oN, sizeof(char));
	char * pbuf = *outbuf;
	const char* pS = buf;
	int i, j, l, m;
	int tBN = iN * 8; 
	bool k;
	INode* pR = root; 
	for(i=0, m=0; i<tBN; i++) // this oN can be known in the transmission protocol
	{
		j = i/8; 
		l = i%8;
		k = buf[j] & (0x01<<(7-l));
		if(k) // k == 1
		{
			pR = ((InternalNode*)pR)->right; 
		}else{
			pR = ((InternalNode*)pR)->left; 
		}
		if(pR->isLeaf())
		{
			pbuf[m++] = ((LeafNode*)pR)->c;
			pR = root;
			if(m == oN) break;
		}
	}
	return true;
}

inline char CHuffman::bit2Char(HuffCode& c, int p)
{
	return ((c[p+0] & 1) ? 0x80:0x00) | 
		   ((c[p+1] & 1) ? 0x40:0x00) |
		   ((c[p+2] & 1) ? 0x20:0x00) |
		   ((c[p+3] & 1) ? 0x10:0x00) |
		   ((c[p+4] & 1) ? 0x08:0x00) |
		   ((c[p+5] & 1) ? 0x04:0x00) |
		   ((c[p+6] & 1) ? 0x02:0x00) |
		   ((c[p+7] & 1) ? 0x01:0x00);
}