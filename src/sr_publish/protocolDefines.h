// protocolDefines.h : contains defines, enumerations and so on

#ifndef PROTOCOL_DEFINES_H
#define PROTOCOL_DEFINES_H

//RQ=Request
//RS=Respond
enum CmdId {RQ_GetImage,   //IPObj
            RS_GetImage,   //IPRSGetImage
            RQ_GetVersion, //IPObj
            RS_GetVersion, //IPRSGetVersion
            RQ_EchoString, //IPRQEchoString
            RS_EchoString,  //IPObj
            ZH_ImageStreamStart, // Start ImageStream 
            ZH_ImageStreamStop,  // Stop ImageStream
            ZH_ImageConfirm,     // confirm the image has been received
            ZH_ImageCameraFail,  // Fail ImageStream
            ZH_EXIT              // EXIT signal
};

typedef struct
{
  CmdId id;
  int size;
}IPObj;

typedef struct
{
  IPObj _p;
  //data;
}IPRSGetImage;

typedef struct
{
  IPObj _p;
  unsigned short ver[4];
}IPRSGetVersion;

typedef struct
{
  IPObj _p;
  char string[32];
}IPRQEchoString;

#endif
