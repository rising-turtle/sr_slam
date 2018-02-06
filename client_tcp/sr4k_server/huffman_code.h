#ifndef HUFFMAN_CODE_H
#define HUFFMAN_CODE_H
#include <iostream>
#include <queue>
#include <map>
#include <climits> // for CHAR_BIT
#include <iterator>
#include <algorithm>

const int UniqueSymbols = 1 << CHAR_BIT;


typedef std::vector<bool> HuffCode;
typedef std::map<char, HuffCode> HuffCodeMap;
typedef std::map<HuffCode, char> HuffCodeMapInv;
 
class INode
{
public:
    const int f;
 
    virtual ~INode() {}
	virtual bool isLeaf(){ return false;}
 
protected:
    INode(int f) : f(f) {}
};
 
class InternalNode : public INode
{
public:
    INode *const left;
    INode *const right;
 
    InternalNode(INode* c0, INode* c1) : INode(c0->f + c1->f), left(c0), right(c1) {}
    ~InternalNode()
    {
        delete left;
        delete right;
    }
};
 
class LeafNode : public INode
{
public:
    const char c;
 
    LeafNode(int f, char c) : INode(f), c(c) {}
	virtual bool isLeaf(){ return true;}
};
 
struct NodeCmp
{
    bool operator()(const INode* lhs, const INode* rhs) const { return lhs->f > rhs->f; }
};

class CHuffman
{
public:
	bool encode( char* buf, int iN, char** outbuf, int& oN, int (&frequencies)[UniqueSymbols]);
	bool decode( char* buf, int iN, char** outbuf, int& oN, int (&frequencies)[UniqueSymbols]);

	INode* BuildTree(const int (&frequencies)[UniqueSymbols]);
	void GenerateCodes(const INode* node, const HuffCode& prefix, HuffCodeMap& outCodes, HuffCodeMapInv&);
	inline char bit2Char(HuffCode&, int p);
};


#endif