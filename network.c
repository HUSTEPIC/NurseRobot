//*****************************************************************************
// network.c
//
// Network Interface
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

#include <stdio.h>

// Simplelink includes
#include "simplelink.h"

//driverlib includes
#include "hw_types.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "gpio_if.h"
#include "uart_if.h"
//common interface includes
#include "common.h"

//App Includes
#include "network.h"
#include "audioCodec.h"

// 这里填写你主机（手机）的局域网 IP v4的 地址
#define HOST_IP_ADDR            0xc0a80164 /* 192.168.1.100 */    


//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************

#define ROLE_INVALID                    (-5)
#define AP_SSID_LEN_MAX                 (33)
#define SH_GPIO_9                       (9)            /* Red */
#define SH_GPIO_11                      (11)           /* Green */
#define SH_GPIO_25                      (25)           /* Yellow */
#define AUTO_CONNECTION_TIMEOUT_COUNT   (50)           /* 5 Sec */

#define CC3200_MDNS_NAME  "CC3200._audio._udp.local"

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern tUDPSocket g_UdpSock;
extern OsiTaskHandle g_NetworkTask;
extern unsigned char g_loopback;
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_uiIpAddress = 0; //Device IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//IP字符串转32位int数   
unsigned int IPStrToInt(const char *ip)  
{  
    unsigned uResult = 0;  
    int nShift = 24;  
    int temp = 0;  
    const char *pStart = ip;  
    const char *pEnd = ip;  
      
    while (*pEnd != '\0' || *pEnd != '\D' || *pEnd != '\A')  
    {  
        while (*pEnd!='.' && *pEnd!='\0')  
        {  
            pEnd++;  
        }  
        temp = 0;  
        for (pStart; pStart!=pEnd; ++pStart)  
        {  
            temp = temp * 10 + *pStart - '0';  
        }     
          
        uResult += temp<<nShift;  
        nShift -= 8;  
          
        if (*pEnd == '\0')  
            break;  
        pStart = pEnd + 1;  
        pEnd++;  
    }  
      
    return uResult;  
}   
// 判断 ip 地址是否常 ， 返回1正常
int checkIP(const char* p)
{
  int n[4];
  char c[4];
  if (sscanf(p, "%d%c%d%c%d%c%d%c",
             &n[0], &c[0], &n[1], &c[1],
             &n[2], &c[2], &n[3], &c[3])
      == 7)
  {
    int i;
    for(i = 0; i < 3; ++i)
      if (c[i] != '.')
        return 0;
    for(i = 0; i < 4; ++i)
      if (n[i] > 255 || n[i] < 0)
        return 0;
    return 1;
  } else
    return 0;
}
  
//*****************************************************************************
//
//! mDNS_Task function
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void mDNS_Task()
{
#if 0
    int lRetValmDNS;
    unsigned int pAddr;
    unsigned long usPort;
    unsigned short     ulTextLen = 200;
    char cText[201]; 
 
    //UnRegister mDNS Service if done Previously
    lRetValmDNS = sl_NetAppMDNSUnRegisterService((signed char *)CC3200_MDNS_NAME,
                              strlen(CC3200_MDNS_NAME));

    while(1)
    {  
        lRetValmDNS = 1;
        
        //Read mDNS service.
        while(lRetValmDNS)
        {
            ulTextLen = 200;
            lRetValmDNS = sl_NetAppDnsGetHostByService((signed char *) \
                                    CC3200_MDNS_NAME,
                                    strlen((const char *)CC3200_MDNS_NAME),
                                    SL_AF_INET,(unsigned long *)&pAddr,&usPort,
                                    &ulTextLen,(signed char *)&cText[0]);
        }
        if(lRetValmDNS == 0 && (pAddr!=INVALID_CLIENT_ADDRESS) && \
                                                 (pAddr!=g_uiIpAddress))
        {               
             //Speaker Detected - Add Client
             g_UdpSock.Client.sin_family = AF_INET;
             g_UdpSock.Client.sin_addr.s_addr = htonl(pAddr);
             g_UdpSock.Client.sin_port = htons(usPort);
             g_UdpSock.iClientLength = sizeof(g_UdpSock.Client);

             g_loopback = 0;

        }
         
             MAP_UtilsDelay(80*1000*100);
    }    
#else
    unsigned int pAddr=HOST_IP_ADDR;
    unsigned long usPort=5001;
	char cCmdBuff[20] = {0};

    //  下面这段代码用于串口输入ip地址，这里为了简单意见，我们屏蔽这段代码
    //  所以需要你直接修改代码， 输入 HOST_IP_ADDR 这个正确值，HOST_IP_ADDR 一般就是你主机比如手机的局域网ip地址
    while(0)
    {
    	char str[64] = {0};
        
        UART_PRINT("Input your host IP addr:(EX: 192.168.0.18)\r\n");
        GetCmd(cCmdBuff, 20);
        UART_PRINT("\r\n");

        if(checkIP(cCmdBuff))
        {
            UART_PRINT("checkIP() OK\r\n");
            
            pAddr = IPStrToInt(cCmdBuff);
            sprintf(str, "your IP(uint32) = 0x%04x\r\n",pAddr );
            UART_PRINT(str);
            UART_PRINT("\r\n");      

            break;
        }
        else
        {
            UART_PRINT("checkIP() FAIL\r\n");
        }

    }
    
    g_UdpSock.Client.sin_family = AF_INET;
    g_UdpSock.Client.sin_addr.s_addr = htonl(pAddr);
    g_UdpSock.Client.sin_port = htons(usPort);
    g_UdpSock.iClientLength = sizeof(g_UdpSock.Client);

    g_loopback = 0;
    while(1)
    {
        MAP_UtilsDelay(80*1000*100);
    }
#endif
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        UART_PRINT("Null pointer\r\n");
        LOOP_FOREVER();
    }
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\r\n",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on application's "
                           "request \r\n",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \r\n",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\r\n",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        UART_PRINT("Null pointer\r\n");
        LOOP_FOREVER();
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_uiIpAddress = pEventData->ip;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\r\n",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \r\n",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(pDevEvent == NULL)
    {
        UART_PRINT("Null pointer\r\n");
        LOOP_FOREVER();
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
    {
        return;
    }

    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        	switch( pSock->socketAsyncEvent.SockTxFailData.status )
            {
                case SL_ECLOSE:
                    /*UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->EventData.sd);*/
                    break;
                default:
                    /*UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->EventData.sd, pSock->EventData.status); */
                  break;
            }
            break;

        default:
           /*UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);*/
          break;
    }
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_uiIpAddress = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}

#ifdef MULTICAST
//*****************************************************************************
//
//! Add to Multicast Group to receive Audio Stream 
//!
//! \param none
//!
//! \return  0 - Success
//!            -1 - Failure

//
//*****************************************************************************
long ReceiveMulticastPacket()
{
    long lRetVal = -1;
    SlSockIpMreq mreq;
    memset(&mreq,0,sizeof(SlSockIpMreq));

    // set interface
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    mreq.imr_multiaddr.s_addr = ADDR_MULTICAST_GROUP;

    // do membership call
    lRetVal = setsockopt(g_UdpSock.iSockDesc, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                             &mreq, sizeof(SlSockIpMreq)); 
    ASSERT_ON_ERROR(lRetVal);

    return SUCCESS;
}
#endif

//****************************************************************************
//
//! Confgiures the mode in which the device will work
//!
//! \param iMode is the current mode of the device
//!
//!
//! \return   SlWlanMode_t
//!                        
//
//****************************************************************************
static int ConfigureMode(int iMode)
{
    long   lRetVal = -1;

    lRetVal = sl_WlanSetMode(iMode);
    ASSERT_ON_ERROR(lRetVal);

    /* Restart Network processor */
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    // reset status bits
    CLR_STATUS_BIT_ALL(g_ulStatus);

    return sl_Start(NULL,NULL,NULL);
}

static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    /* Wait */
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
#endif
    }

    return SUCCESS;

}

//*****************************************************************************
//
//! Connect the Device to Network
//!
//! \param  None
//!
//! \return  0 - Success
//!            -1 - Failure
//!
//*****************************************************************************

long ConnectToNetwork()
{
    long lRetVal = -1;
    unsigned int uiConnectTimeoutCnt =0;
    
    //Start Simplelink Device 
    lRetVal =  sl_Start(NULL,NULL,NULL);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("sl_Start lRetVal=%d\r\n", lRetVal);
    
    WlanConnect();

{
    extern unsigned char g_ucMicStartFlag;
    extern unsigned char g_ucSpkrStartFlag;
    g_ucMicStartFlag = 1;
    g_ucSpkrStartFlag = 1;
}
    if(lRetVal != ROLE_STA)
    {
        if (ROLE_AP == lRetVal)
        {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }
        //
        // Configure to STA Mode
        //
        lRetVal = ConfigureMode(ROLE_STA);
        if(lRetVal !=ROLE_STA)
        {
            UART_PRINT("Unable to set STA mode...\r\n");
            lRetVal = sl_Stop(SL_STOP_TIMEOUT);
            CLR_STATUS_BIT_ALL(g_ulStatus);
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
#if 0
    //waiting for the device to Auto Connect
    while(uiConnectTimeoutCnt<AUTO_CONNECTION_TIMEOUT_COUNT &&
        ((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))) 
    {
        //Turn Green LED On       
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);            
        osi_Sleep(50);            
        //Turn Green LED Off
        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);            
        osi_Sleep(50);
        
        uiConnectTimeoutCnt++;
    }
    //Couldn't connect Using Auto Profile
    if(uiConnectTimeoutCnt==AUTO_CONNECTION_TIMEOUT_COUNT)
    {
        CLR_STATUS_BIT_ALL(g_ulStatus);
        
        //Turn Green LED On       
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);  
        
        //Connect Using Smart Config
        lRetVal = SmartConfigConnect();
        ASSERT_ON_ERROR(lRetVal);

        //Waiting for the device to Auto Connect
        while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
        {
            MAP_UtilsDelay(500);              
        }
         
        //Turn Green LED Off      
        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);    
    }
#endif
    return SUCCESS;
    
}
//*****************************************************************************
//
//! Create Socket and Bind to Local IP
//!
//! \param  None
//!
//! \return  0 - Success
//!            -1 - Failure
//
//*****************************************************************************

long CreateUdpServer(tUDPSocket *pSock)
{
    int uiPort = AUDIO_PORT;
    long lRetVal = -1;
    
    pSock->iSockDesc = socket(AF_INET, SOCK_DGRAM, 0);
    pSock->Server.sin_family = AF_INET;
    pSock->Server.sin_addr.s_addr = htonl(INADDR_ANY);
    pSock->Server.sin_port = htons(uiPort);
    pSock->iServerLength = sizeof(pSock->Server);

    pSock->Client.sin_family = AF_INET;
    pSock->Client.sin_addr.s_addr = htonl(INVALID_CLIENT_ADDRESS);
    pSock->Client.sin_port = htons(uiPort);
    pSock->iClientLength = sizeof(pSock->Client);
    lRetVal = bind(pSock->iSockDesc,(struct sockaddr*)&(pSock->Server),
                 pSock->iServerLength);
    ASSERT_ON_ERROR(lRetVal);

    return SUCCESS;

}

//*****************************************************************************
//
//! Network Task
//!
//! \param  pvParameters - Parameters to the task's entry function
//!
//! \return None
//!
//*****************************************************************************
void Network( void *pvParameters )
{
    long lRetVal = -1;
    
    UART_PRINT("Network 1 \r\n");

    //Initialize Global Variable
    InitializeAppVariables();

    UART_PRINT("Network 2 \r\n");

    //Connect to Network
    lRetVal = ConnectToNetwork();
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to establish connection w/ an AP \r\n");
        LOOP_FOREVER();
    } 
    else
    {
        UART_PRINT("Establish connection w/ an AP \r\n");
    }

    UART_PRINT("Network 3 \r\n");
    
    //Cread UDP Socket and Bind to Local IP Address
    lRetVal = CreateUdpServer(&g_UdpSock);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to Create UDP Server \r\n");
        LOOP_FOREVER();
    }

    UART_PRINT("Network 4 \r\n");

#ifdef MULTICAST  
    //Add to Multicast Group
    lRetVal = ReceiveMulticastPacket();
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to Create UDP Server \r\n");
        LOOP_FOREVER();
    }

    //Delete the Networking Task as Service Discovery is not needed
    osi_TaskDelete(&g_NetworkTask);
#else
    //Discover CC3200 Audio Devices  
    mDNS_Task();
#endif    

}
