// mesaServer.cpp : Defines the entry point for the console application.
//

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <arpa/inet.h>
#include <string.h>
#define _getch getchar
#define closesocket close
#define SD_BOTH SHUT_RDWR
//typedef unsigned short WORD;
//typedef unsigned int   DWORD;
typedef unsigned char   BYTE;
typedef int   SOCKET;

#include "protocolDefines.h"
#include "libMesaSR.h"


int TCPRecv(SOCKET s, void *buf, int len, int flags);
int TCPServer1();

//dummy implementation because it is not linked with pthread library
#ifdef __cplusplus
extern "C"
{
#endif
int pthread_mutex_lock(pthread_mutex_t*){return 0;}
int pthread_mutex_unlock(pthread_mutex_t*){return 0;}
int pthread_mutex_init(pthread_mutex_t*, const pthread_mutexattr_t*){return 0;}
#ifdef __cplusplus
};
#endif

int main(int argc, char* argv[])
{
  TCPServer1();
  puts("TCPServer finished.");
  return 0;
}

int TCPServer1()
{
  WORD port=htons(22222);
  struct sockaddr_in saLoc, saRmt;
  SOCKET sLoc,sRmt;
  socklen_t saLen=sizeof(saRmt);
  int res;
  SRCAM srCam;
  int w,h;
  puts("starting TCPServer");
#ifdef __BFIN__
  SR_Open(&srCam);
#else
  // SR_OpenDlg(&srCam,3,0);
  SR_OpenETH(&srCam, "192.168.1.3");
#endif
  w=SR_GetCols(srCam);
  h=SR_GetRows(srCam);

  // follow Soon' configuration
  res = SR_SetIntegrationTime(srCam, 150);  // 0.300 ms + (intTime)*0.1 ms = 15.3 ms
  res = SR_SetMode(srCam, AM_SW_TRIGGER | AM_CONF_MAP);   // AM_SW_TRIGGER : Trigger integration by SR_Acquire()
                                                          // AM_CONF_MAP : For sr4k process a confidence map

  if ((sLoc=socket(AF_INET, SOCK_STREAM, 0))==-1)
  { 
    perror("TCPListen: socket() failed.");
    return -1;
  }
  memset((char *)&saLoc, 0, sizeof(saLoc));
  saLoc.sin_family = AF_INET;
  saLoc.sin_port = port;
  saLoc.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sLoc, (struct sockaddr*)&saLoc, sizeof(saLoc))==-1)
  {
    perror("TCPListen: bind() failed.");
    return -1;
  }
  listen(sLoc,0);
  printf("bind and listen on %s:%d\n",inet_ntoa(saLoc.sin_addr), ntohs(saLoc.sin_port));

  IPObj rx;
  
  // data buffer to store 3d points 
  const size_t FSIZE = w*h; 
  size_t point_buf_size = FSIZE*3*sizeof(float);
  // memset(buf, 0xaf, buf_size);

  // data buffer to store intensity of image
  size_t img_buf_size = FSIZE*sizeof(WORD);
  // WORD * img_buf = (WORD*)malloc(img_buf_size);
  // memset(img_buf, 0, img_buf_size);

  // data buffer to store confidence value
  size_t conf_buf_size = FSIZE*sizeof(WORD);
  // WORD* conf_buf = (WORD*)malloc(conf_buf_size);
  // memset(conf_buf, 0, conf_buf_size);

  size_t buf_size = point_buf_size + img_buf_size + conf_buf_size;
  char* buf =(char*)malloc(buf_size);
  float * fbuf = (float*)buf;
  memset(buf, 0, buf_size);

  // entriance z x y
  float *z = fbuf;
  float *x = fbuf + FSIZE;
  float *y = fbuf + 2*FSIZE;
  
  int pitchX = sizeof(float);
  int pitchY = pitchX;
  int pitchZ = pitchX;

  for(;;)
  {
    sRmt=accept(sLoc,(struct sockaddr*)&saRmt,&saLen);
    if(sRmt<0)
      perror("TCPAccept failed");

    printf("accept %s:%u\n",inet_ntoa(saRmt.sin_addr), htons(saRmt.sin_port));
    //READ WRITE
    for(;;)
    {
      res=TCPRecv(sRmt, (char*)&rx, sizeof(rx), MSG_PEEK);
      printf("TCPRecv peek %d\n",res);
      if(res<=0)
        break;
      switch(rx.id)
      {
        case RQ_GetImage:   
        {
          res=TCPRecv(sRmt, (char*)&rx, sizeof(rx), 0);
          printf("RQ_GetImage consume %d\n",res);

          // 1, start the integration by software triggering
          SR_Acquire(srCam);
            
          // 2, get 3d points 
          SR_CoordTrfFlt(srCam, x, y, z, pitchX, pitchY, pitchZ);

          // 3, image array
          ImgEntry* imgEntryArray;
          int nImg = SR_GetImageList(srCam, &imgEntryArray);
          WORD* p = (WORD*)imgEntryArray[1].data;
          memcpy(buf + point_buf_size, (char*)p, img_buf_size);
          // memcpy(img_buf, p, img_buf_size);

          // 4, confidence array
          WORD* p_conf = (WORD*)imgEntryArray[2].data;
          memcpy(buf + point_buf_size + img_buf_size, (char*)p_conf, conf_buf_size);
          // memcpy(conf_buf, p, conf_buf_size);
          
          // 5, send the buf from internet
          IPRSGetImage tx;
          tx._p.id = RS_GetImage;
          tx._p.size = sizeof(tx) + buf_size;
          res = send(sRmt, (char*)&tx, sizeof(tx), 0);
          printf("sent %d bytes: \n", res);
          res = send(sRmt, buf, buf_size, 0);
          printf("sent %d bytes: \n", res);

          // for debug 
          /*FILE* fp = fopen("debug.log", "w");
          fprintf(fp,"%% Calibrated distance [m]\n");
          for(int i=0; i<FSIZE; i++)
          {
            fprintf(fp, "%7.4f ", z[i]);
          }
          fprintf(fp, "\n %% Calibrated xVector [m]\n");
          for(int i=0; i<FSIZE; i++)
          {
            fprintf(fp, "%7.4f ", x[i]);
          }
          fprintf(fp, "\n %% Calibrated yVector [m]\n");
          for(int i=0; i<FSIZE; i++)
          {
            fprintf(fp, "%7.4f ", y[i]);
          }
          fprintf(fp, "\n %% Amplitude\n");
          for(int i=0; i<FSIZE; i++)
          {
            fprintf(fp, "%10d", p[i]);
          }
          fprintf(fp, "\n %% Confidence\n");
          for(int i=0; i<FSIZE; i++)
          {
            fprintf(fp, "%10d", p_conf[i]);
          }
          fprintf(fp, "\n");
          fclose(fp);
          */

          /*
          char* dst;
          int sz=w*h*2;
          SR_Acquire(srCam);
          dst = (char*)SR_GetImage(srCam, 0);
          IPRSGetImage tx;
          tx._p.id=RS_GetImage;
          tx._p.size=sizeof(tx)+sz;
          res=send(sRmt,(char*)&tx,sizeof(tx),0);
          printf("sent %d bytes\n",res);
          res=send(sRmt,(char*)dst,sz,0);
          printf("sent %d bytes\n",res);
          */
          break;
        }
        case RQ_GetVersion:   
        {
          res=TCPRecv(sRmt, (char*)&rx, sizeof(rx), 0);
          puts("RQ_GetVersion");
          IPRSGetVersion tx;
          tx._p.id=RS_GetVersion;
          res=SR_GetVersion(tx.ver);
          res=send(sRmt,(char*)&tx,sizeof(tx),0);
          printf("sent %d bytes\n",res);
          break;
        }
        case RQ_EchoString:   
        {
          //IPObj tx;
          IPRQEchoString rx;
          res=TCPRecv(sRmt, (char*)&rx, sizeof(rx), 0);
          puts("RQ_EchoString");
          printf("received string '%s'\n",rx.string);
          break;
        }
        default:
        {
          res=TCPRecv(sLoc, (char*)&rx, sizeof(rx), 0);//consume
          fprintf(stderr, "id %u size %u\n",rx.id,rx.size);
          break;
        }
      }
    }
    puts("shutdown");
    goto shutdown;
  }
shutdown:
  shutdown(sRmt, SD_BOTH);//SHUT_RDWR);
  closesocket(sRmt);
  shutdown(sLoc, SD_BOTH);//SHUT_RDWR);
  closesocket(sLoc);
  
  res = SR_Close(srCam);
  printf("SR_Close() called result: %d \n", res);
  puts("ending TCPServer");
  return 0;
}

int TCPRecv(SOCKET s, void *buf, int len, int flags)
{
  int pos,res;
  BYTE* p=(BYTE*)buf;

  for(pos=0,res=0;res<len;)
  {
    res = recv(s, (char*)&p[pos], len-pos, flags);
    if ( res <= 0)
    {
      printf("TCPRecv: recv[] failed %d. received %d/%d\n",res,pos,len);
      break;
    }
    else
    {
      if(!(flags&MSG_PEEK))
      {
        pos+=res;
        //Printf(MK_DEBUG_STRING|MC_ETH,"received %d/%d (+%d) bytes\r",pos,len,res);
        res=pos;
      }
      else
      {
        //Printf(MK_DEBUG_STRING|MC_ETH,"peeked %d/%d bytes\r",res,len);
      }
    }
  }
  //Printf(MK_DEBUG_STRING|MC_ETH,"received %d/%d (+%d) bytes done.\n",pos,len,res);
  return res;
}
