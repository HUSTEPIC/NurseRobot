//*****************************************************************************
// speaker.c
//
// LINE OUT (Speaker Operation)
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************
// Hardware & driverlib library includes
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "hw_ints.h"

// simplelink include
#include "simplelink.h"

// common interface includes
#include "common.h"
#include "uart_if.h"
// Demo app includes
#include "network.h"
#include "circ_buff.h"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern tUDPSocket g_UdpSock;
int g_iReceiveCount =0;
int g_iRetVal =0;
int iCount =0;
extern unsigned long  g_ulStatus;
extern unsigned char g_ucSpkrStartFlag;
extern unsigned char g_loopback;
unsigned char speaker_data[16*1024];
extern tCircularBuffer *pPlayBuffer;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

#define BUF_SIZE  10240
static unsigned char g_cBsdBuf[BUF_SIZE];
void Speaker2() 
{
    SlSockAddrIn_t  sAddr;
    SlSockAddrIn_t  sLocalAddr;
    int             iCounter;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    long            lLoopCount = 0;
    short           sTestBufLen;
    long            lCount = 0;
    long iRetVal = -1;

//#define IP_ADDR            0xc0a80065 /* 192.168.0.101 */    
#define IP_ADDR            0xc0a80069 /* 192.168.0.105 */    
//#define IP_ADDR            0xc0a80069 /* 192.168.0.105 */    

    unsigned int pAddr=IP_ADDR;
    unsigned long usPort=5002;
    tUDPSocket g_UdpReciveSock;

    g_UdpReciveSock.Client.sin_family = AF_INET;
    g_UdpReciveSock.Client.sin_addr.s_addr = htonl(IP_ADDR);
    g_UdpReciveSock.Client.sin_port = htons(usPort);
    g_UdpReciveSock.iClientLength = sizeof(g_UdpReciveSock.Client);
    

    sTestBufLen  = BUF_SIZE;
    //filling the UDP server socket address
    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = sl_Htons(usPort);
    sLocalAddr.sin_addr.s_addr = 0;

    iAddrSize = sizeof(SlSockAddrIn_t);


//    UART_PRINT("Speaker2 info line=%d \r\n",__LINE__);
    MAP_UtilsDelay(10000000);
    MAP_UtilsDelay(10000000);
    MAP_UtilsDelay(10000000);
    MAP_UtilsDelay(10000000);
    MAP_UtilsDelay(10000000);
//    UART_PRINT("Speaker2 info line=%d \r\n",__LINE__);

    // creating a UDP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    if( iSockID < 0 )
    {
         UART_PRINT("Speaker2 err line=%d \r\n",__LINE__);
       // error
    }

    // binding the UDP socket to the UDP server address
    iStatus = sl_Bind(iSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
    if( iStatus < 0 )
    {
          UART_PRINT("Speaker2 err line=%d \r\n",__LINE__);
       // error
        sl_Close(iSockID);
    }

    // no listen or accept is required as UDP is connectionless protocol
    /// waits for 1000 packets from a UDP client
    while (1)
    {   // 接收手机上的音频数据
        iStatus = sl_RecvFrom(iSockID, g_cBsdBuf, sTestBufLen, 0,
                     ( SlSockAddr_t *)&sAddr, (SlSocklen_t*)&iAddrSize );

        if( iStatus < 0 )
        {
            UART_PRINT("Speaker2 err line=%d \r\n",__LINE__);
            // error
            sl_Close(iSockID);
        }
        else
        {
            if(iStatus>0)
            {
                UART_PRINT("R.");  // 填充播放缓冲器实现音频播放
                iRetVal = FillBuffer(pPlayBuffer, (unsigned char*)g_cBsdBuf,\
                                      iStatus);
                if(iRetVal < 0)
                {
                    UART_PRINT("Unable to fill buffer");
                    LOOP_FOREVER();
                }
            }
//            UART_PRINT("Spk =%d\r\n", __LINE__);

        }
//        UART_PRINT("Recieved iStatus=%d \r\n",iStatus);
//        MAP_UtilsDelay(10000);
    }

    //closing the socket after receiving 1000 packets
    sl_Close(iSockID);


}

//*****************************************************************************
//
//! Speaker Routine 
//!
//! \param pvParameters     Parameters to the task's entry function
//!
//! \return None
//
//*****************************************************************************
void Speaker( void *pvParameters )
{
    long iRetVal = -1;
    Speaker2(); 
#if 0    
    while(1)
    {
        while(g_ucSpkrStartFlag || g_loopback)
        {     

            if(!g_loopback)
            {
                fd_set readfds,writefds;
                struct SlTimeval_t tv;
                FD_ZERO(&readfds);
                FD_ZERO(&writefds);
                FD_SET(g_UdpSock.iSockDesc,&readfds);
                FD_SET(g_UdpSock.iSockDesc,&writefds);
                tv.tv_sec = 0;
                tv.tv_usec = 2000000;

                UART_PRINT("Spk =%d\r\n", __LINE__);
                
                int rv = select(g_UdpSock.iSockDesc, &readfds, NULL, NULL, &tv);
                UART_PRINT("rv =%d\r\n", rv);
                if(rv <= 0)
                {
                    continue;
                }
                UART_PRINT("Spk =%d\r\n", __LINE__);
                if (FD_ISSET(g_UdpSock.iSockDesc, &readfds) )
                {
                    UART_PRINT("Spk =%d\r\n", __LINE__);
                    g_iRetVal = recvfrom(g_UdpSock.iSockDesc, (char*)(speaker_data),\
                                          PACKET_SIZE*16, 0,\
                                          (struct sockaddr *)&(g_UdpSock.Client),\
                                          (SlSocklen_t*)&(g_UdpSock.iClientLength));
                }

                if(g_iRetVal>0)
                {
                    UART_PRINT("Spk =%d\r\n", g_iRetVal);
                    iRetVal = FillBuffer(pPlayBuffer, (unsigned char*)speaker_data,\
                                          g_iRetVal);
                    if(iRetVal < 0)
                    {
                        UART_PRINT("Unable to fill buffer");
                        LOOP_FOREVER();
                    }
                }
                UART_PRINT("Spk =%d\r\n", __LINE__);
            }
            else
            {
                MAP_UtilsDelay(1000);
            }
        }
        MAP_UtilsDelay(1000);
    }
#endif    
}

