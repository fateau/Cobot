///////////////////////////////////////////////////////////////////////////////
//
//
//			Copyright (c) 2013 - 2018 IntervalZero, Inc.  All rights reserved.
//
//
//File: rtnapi.h
//
//Abstract:
//
//		This file defines the functions which only Rtnapi applications can use.
//	Rtnapi imports API functionality driectly from the realtime stack.
//
///////////////////////////////////////////////////////////////////////////////
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "Rtx64Base.h"

#ifdef UNDER_RTSS
#undef printf
#define printf	RtPrintf
#undef wprintf
#define wprintf RtWprintf
#endif

#if defined RTX64_RTN_EXPORTS
	#define RTNPROC __declspec (dllexport)
#else
    #define RTNPROC __declspec(dllimport)
#endif

#ifndef RTNDPROC
#define RTNDPROC __declspec(dllexport)
#endif

#ifndef ENIOCBASE

#define ENIOCBASE       ((unsigned short )('e' << 8))

/* Values for raw ethernet ioctls.  These select the action inside
 * the link layer ioctl routine.
 */
#define ENIOCEND        (ENIOCBASE+10)
#define ENIOCNORMAL     ENIOCBASE       /* accept broadcast and specific */
#define ENIOCPROMISC    (ENIOCBASE+1)   /* accept all undamaged packets */
#define ENIOCALL        (ENIOCBASE+2)   /* accept ALL packets */
#define ENIOCRESET      (ENIOCBASE+3)
#define ENIOCWHATRU     (ENIOCBASE+4)   /* return device name from ndevsw */
#define ENIOADDMULTI	(ENIOCBASE+5)	/* add multicast address */
#define	ENIODELMULTI	(ENIOCBASE+6)	/* delete multicast address */

#define ENIOLINKSTATUS  (ENIOCBASE+80)
#endif
//*********************************************

typedef  volatile void* CriticalLock;

#define RTN_ERROR -1

#define RTND_INTERFACE_PREFIX _T("rtnd")
#define RTND_FILTER_PREFIX _T("filter")
#define RTND_SIZE	64
#define MAX_INTERFACES 200
#define DEVICE_UP	1
#define DEVICE_DOWN 0
#define INT_THREAD_PRIORITY 127
#define RECEIVE_THREAD_PRIORITY 126
#define TIMER_THREAD_PRIORITY	125
#define MTU_DEFAULT 1500


#define RTX_CTRL_REG_KEY			_T("SOFTWARE\\INTERVALZERO\\RTX64")
#define RTX_CTRL_TCPIP_REG_KEY		_T("SOFTWARE\\INTERVALZERO\\RTX64\\RTTCPIP\\TCPIP00")
#define RTX_CTRL_RTTCPIP_REG_KEY	_T("SOFTWARE\\INTERVALZERO\\RTX64\\RTTCPIP")

//
//Global Rt-TCPIP Stack level settings
//
#define	RTND_RTTCPIP_VERBOSE			_T("Verbose")
#define RTND_RTTCPIP_STACKDEBUG			_T("StackDebug")
//
//interface strings
//
#define RTND_INTERFACE_RCVIDEALPROC		_T("ReceiveIdealProcessor")
#define RTND_INTERFACE_INTIDEALPROC		_T("InterruptIdealProcessor")
#define RTND_INTERFACE_DRIVER			_T("driver")
#define RTND_INTERFACE_ENABLE			_T("Enable")
#define RTND_INTERFACE_FRNDNAME			_T("FriendlyName")
#define RTND_INTERFACE_IPADDR			_T("ipaddr")
#define RTND_INTERFACE_NETMASK			_T("netmask")
#define RTND_INTERFACE_IPADDRESSES		_T("AdditionalIPAddresses")
#define RTND_INTERFACE_NETMASKS			_T("AdditionalNetmasks")
#define RTND_INTERFACE_GATEWAY			_T("gateway")
#define RTND_INTERFACE_IPV6ADDR			_T("ipv6addr")
#define RTND_INTERFACE_IPV6PREFIX		_T("ipv6prefix")
#define RTND_INTERFACE_LOCATION			_T("location")
#define RTND_INTERFACE_INTPRIORITY		_T("InterruptPriority")
#define RTND_INTERFACE_MTU				_T("MTU")
#define RTND_INTERFACE_RECVPRIORITY		_T("ReceivePriority")
#define RTND_INTERFACE_LINEBASEDONLY	_T("LineBasedOnly")
#define RTND_INTERFACE_PREFERMSI		_T("PreferMsi")
#define RTND_INTERFACE_NUMRECVBUFF		_T("NumRecvBuffers")
#define RTND_INTERFACE_NUMXMITBUFF		_T("NumXmitBuffers")
#define RTND_INTERFACE_INTERRUPTTYPE	_T("InterruptType")
#define RTND_INTERFACE_LNKSTAT  		_T("LinkStatus")
#define RTND_INTERFACE_LNKSTATIDEALPROC	_T("LinkStatusIdealProcessor")
#define RTND_INTERFACE_LNKSTATPRIORITY	_T("LinkStatusPriority")


//
//Filter strings
//
#define RTND_INTERFACE_FILTERSTATE		_T("FilterState")
#define RTND_INTERFACE_FILTER    		_T("Filter")

//
//Stack Instance Strings
//
#define	RTND_INSTANCE_TICKINTERVAL		_T("TickInterval")
#define	RTND_INSTANCE_AFFINITY			_T("Affinity")
#define	RTND_INSTANCE_MEMORY			_T("Memory")
#define	RTND_INSTANCE_MAXSOCKETS		_T("MaxSockets")
#define RTND_INSTANCE_MAXARPENTRIES		_T("MaxArpEntries")
#define RTND_INSTANCE_IPFRAGTIMEOUT		_T("IpFragmentTimeout")
#define	RTND_INSTANCE_TIMERPRIORITY		_T("TimerPriority")
#define	RTND_INSTANCE_TIMERIDEALPROC	_T("TimerIdealProcessor")
#define	RTND_INSTANCE_STACKMTU			_T("StackMTU")

//
//represents a filter interface will also map to internal DeviceEntry
//
typedef struct {
	HANDLE FilterDllHandle;
	FARPROC pRtndRxFilter;
	FARPROC pRtndTxFilter;

	// (Ex)-tended versions that take an additional pointer argument
	//	currently: the network device pointer (ndp)
	FARPROC pRtndRxFilterEx;
	FARPROC pRtndTxFilterEx;

	FARPROC pRtndConfigFilter;
	TCHAR FilterName[MAX_PATH];
	DWORD FilterState;
} RTNFILTERINTERFACE, *PRTNFILTERINTERFACE;

// The maximum allowed number of characters in a string representing and IPv4 address (xxx.xxx.xxx.xxx)
#define IP_STRING_MAX_LENGTH 20

//
// Represents an IP configuration for a physical interface
//
typedef struct _RTNIPCONFIGURATION 
{
	TCHAR szIpv4Address[IP_STRING_MAX_LENGTH];
	TCHAR szNetMask[IP_STRING_MAX_LENGTH];
} RTNIPCONFIGURATION, * PRTNIPCONFIGURATION;

//
// Represents a device interface will map to internal DeviceEntry
//
#pragma pack(push,8)
typedef struct {
	TCHAR szDeviceName[RTND_SIZE];
	TCHAR szDllName[MAX_PATH];
	HANDLE DllHandle;
	FARPROC pRtndInitialize;
	FARPROC pRtndConfigure;
	FARPROC pRtndTransmit;
	FARPROC pRtndReceive;
	FARPROC pRtndIoctl;
	FARPROC pRtndUpDown;
	FARPROC pRtndRequest;
	TCHAR szIpv4Address[IP_STRING_MAX_LENGTH];
	TCHAR szIpv6Address[50];
	TCHAR szNetMask[20];
	TCHAR szDefaultGateway[IP_STRING_MAX_LENGTH];
	TCHAR szLocation[IP_STRING_MAX_LENGTH];
	char macAddress[8];
	ULONG macAddressLen;
	ULONG RecvIdealProcessor;
	ULONG IntIdealProcessor;
	ULONG numFilters;
	ULONG index;
	ULONG IntPriority;
	ULONG RequestedMtu;
	HKEY hRegKey;
	HANDLE hReceiveThread;
	void *recvPacketPtr;
	ULONG dataLen; 
	ULONG ulLineBasedOnly;
	ULONG ulPreferMSi;
	ULONG ulNumRecvBuffers;
	ULONG ulNumXmitBuffers;
	TCHAR regKeyName[MAX_PATH];
	BOOL bEnable;
	BOOL bLinkStatus;
	ULONG ulLinkStatusPriority;
	ULONG ulLinkStatusIdealProcessor;
	void *userInterfaceHandle;
	void *pStackInstance;		//pointer back to stack instance
	RTNFILTERINTERFACE *pFilterInstance;
	ULONG_PTR interruptAffinity;
	unsigned int ipConfigurationCount;
	RTNIPCONFIGURATION ipConfiguration [MAX_IPS_PER_INTERFACE];
} RTNINTERFACE, *PRTNINTERFACE;
#pragma pack(pop)

//
//Filter data structures
//
typedef struct _ethHeader{
	unsigned char ethDstAddr[6];
	unsigned char ethSrcAddr[6];
	unsigned short ethType;
}ETHERNETHEADER, *PETHERNETHEADER;

//
// Filter function prototypes
//

// This function is called when an incoming Ethernet frame is received from the NIC driver. This function will be called 
// once per frame received.
RTNDPROC
BOOL RtndReceiveFilter(
	ETHERNETHEADER *pEthernetHeader,
	void *pData,
	unsigned long ulEthernetDataSize
	);
// This function is called when an outgoing Ethernet frame is received from the stack. This function will be called once 
// per frame transmitted.
RTNDPROC 
BOOL RtndTransmitFilter(
	ETHERNETHEADER *pEthernetHeader,
	void *pData,
	unsigned long ulEthernetDataSize
	);

// This function can be called when the filter wants to transmit a frame pointed to by the parameter EthernetFrame.
// NOTE:  This function requires the RT-TCP/IP Stack to be started.To check the status of the Stack, use RtnIsStackOnline.
// If this function is called while the Stack is not started, an exception may occur.
RTNPROC
BOOL RtndFrameTransmit(
		void *pEthernetFrame,
		unsigned long ulDataSize,
		unsigned int  uLayer );

RTNPROC 
// This function is used by the filter driver to allocate a block of memory to store an Ethernet frame.
// NOTE:  This function requires the RT-TCP/IP Stack to be started.To check the status of the Stack, use RtnIsStackOnline.
// If this function is called while the Stack is not started, an exception may occur.
void *RtndFrameAllocate(
	char *DevName,
	unsigned long ulDataSize);

RTNPROC 
// This function frees data allocated by RtndFrameAllocate.
// NOTE:  This function requires the RT-TCP/IP Stack to be started.To check the status of the Stack, use RtnIsStackOnline.
// If this function is called while the Stack is not started, an exception may occur.
void RtndFrameFree(
	void *pFrame
	);


//
//Enum Data Types
//
typedef  enum _RTND_MEDIA_CONNECT_STATE {
		RtndMediaConnectStateUnknown,
		RtndMediaConnectStateConnected,
		RtndMediaConnectStateDisConnected
} RTND_MEDIA_CONNECT_STATE, *PRTND_MEDIA_CONNECT_STATE;

typedef  enum _RTND_MEDIA_DUPLEX_STATE {
		RtndMediaDuplexStateUnknown,
		RtndMediaDuplexStateHalf,
		RtndMediaDuplexStateFull
} RTND_MEDIA_DUPLEX_STATE, *PRTND_MEDIA_DUPLEX_STATE;

typedef enum _RTND_MEDIA_SPEED {
		RtndMediaSpeedUnknown,
		RtndMediaSpeed10,
		RtndMediaSpeed100,
		RtndMediaSpeed1000,
		RtndMediaSpeed10000
} RTND_MEDIA_SPEED, *PRTND_MEDIA_SPEED;



#define	RtLIST_HEAD(name, type)						\
struct name {								\
	struct type *First;			\
}

#define	RtLIST_ENTRY(type)						\
struct {								\
	struct type *Next;				\
	struct type **Prev;		\
}


#define	RtLIST_EMPTY(head)	((head)->First == NULL)

#define	RtLIST_FIRST(head)	((head)->First)


#define	RtLIST_INIT(head) do {						\
	RtLIST_FIRST((head)) = NULL;					\
} while (0, 0)

#define	RtLIST_INSERT_NEXT(listelm, elm, field) do {			\
	if ((RtLIST_NEXT((elm), field) = RtLIST_NEXT((listelm), field)) != NULL)\
		RtLIST_NEXT((listelm), field)->field.Prev =		\
		    &RtLIST_NEXT((elm), field);				\
	RtLIST_NEXT((listelm), field) = (elm);				\
	(elm)->field.Prev = &RtLIST_NEXT((listelm), field);		\
} while (0, 0)


#define	RtLIST_INSERT_HEAD(head, elm, field) do {				\
	if ((RtLIST_NEXT((elm), field) = RtLIST_FIRST((head))) != NULL)	\
		RtLIST_FIRST((head))->field.Prev = &RtLIST_NEXT((elm), field);\
	RtLIST_FIRST((head)) = (elm);					\
	(elm)->field.Prev = &RtLIST_FIRST((head));			\
} while (0, 0)

#define	RtLIST_NEXT(elm, field)	((elm)->field.Next)

#define	RtLIST_REMOVE(elm, field) do {					\
	if (RtLIST_NEXT((elm), field) != NULL)				\
		RtLIST_NEXT((elm), field)->field.Prev = 		\
		    (elm)->field.le_prev;				\
	*(elm)->field.Prev = RtLIST_NEXT((elm), field);		\
} while (0, 0)

//
//RtndRequest OIDs
//
typedef  unsigned long RTND_OID; 

#define RTND_OID_GEN_BYTES_RCV				\
							((RTND_OID) 0x00020219)
#define RTND_OID_GEN_BYTES_XMIT				\
	 						((RTND_OID) 0x0002021A)
#define RTND_OID_GEN_RCV_DISCARDS			\
	 						((RTND_OID) 0x0002021B)
#define RTND_OID_GEN_XMIT_DISCARDS			\
	 						((RTND_OID) 0x0002021C)
#define RTND_OID_GEN_RCV_OK					\
					 		((RTND_OID) 0x00020102)
#define RTND_OID_GEN_XMIT_OK				\
					 		((RTND_OID) 0x00020101)
#define RTND_OID_GEN_RCV_ERROR				\
	 						((RTND_OID) 0x00020104)
#define RTND_OID_GEN_XMIT_ERROR				\
					 		((RTND_OID) 0x00020103)
#define RTND_OID_GEN_MULTICAST_FRAMES_RCV	\
					 		((RTND_OID) 0x0002020A)
#define RTND_OID_GEN_MULTICAST_FRAMES_XMIT	\
					 		((RTND_OID) 0x00020204)
#define RTND_OID_GEN_BROADCAST_FRAMES_RCV	\
					 		((RTND_OID) 0x0002020C)
#define RTND_OID_GEN_BROADCAST_FRAMES_XMIT	\
					 		((RTND_OID) 0x00020206)
#define RTND_OID_GEN_LINK_SPEED				\
							((RTND_OID) 0x00010107)
#define RTND_OID_GEN_ALL_SUPPORTED_STAT_QUERY	\
							((RTND_OID) 0x80000000)
#define RTND_OID_GEN_STATISTICS					\
							((RTND_OID) 0x00020106)
#define RTND_OID_GEN_MAC_ADDRESS				\
							((RTND_OID) 0x00010205)
#define RTND_OID_GEN_MEDIA_CONNECT_STATUS		\
							((RTND_OID) 0x00010207)
#define RTND_OID_GEN_MEDIA_DUPLEX_STATE			\
							((RTND_OID) 0x0001028C)

//
//Flags for RTND_STATISTICS_INFO->SupportedStatistics structure
//
#define RTND_STATISTICS_FLAGS_VALID_BYTES_RCV                       0x00000008
#define RTND_STATISTICS_FLAGS_VALID_RCV_DISCARDS                    0x00000010
#define RTND_STATISTICS_FLAGS_VALID_RCV_ERROR                       0x00000020
#define RTND_STATISTICS_FLAGS_VALID_BYTES_XMIT                      0x00000200
#define RTND_STATISTICS_FLAGS_VALID_XMIT_ERROR                      0x00000400
#define RTND_STATISTICS_FLAGS_VALID_XMIT_DISCARDS                   0x00008000
#define RTND_STATISTICS_FLAGS_VALID_MULTICAST_BYTES_RCV             0x00020000
#define RTND_STATISTICS_FLAGS_VALID_BROADCAST_BYTES_RCV             0x00040000
#define RTND_STATISTICS_FLAGS_VALID_MULTICAST_BYTES_XMIT            0x00100000
#define RTND_STATISTICS_FLAGS_VALID_BROADCAST_BYTES_XMIT            0x00200000

#define RTND_STATISTICS_FLAGS_VALID_RCV_OK					        0x01000000
#define RTND_STATISTICS_FLAGS_VALID_XMIT_OK							0x02000000

//
//RTND_STATUS 
//
typedef  int 							\
								RTND_STATUS, *PRTND_STATUS; 
#define  RTND_STATUS_SUCCESS			\
								((RTND_STATUS)0x00000001)
#define  RTND_STATUS_INVALID_REQUEST	\
								((RTND_STATUS)0xC0000010)
#define  RTND_STATUS_INVALID_LENGTH		\
								((RTND_STATUS)0xC0000100)
#define  RTND_STATUS_FAILURE			\
								((RTND_STATUS)0xFFFFFFFF)

//
//Network Adapter Info Structure
//
typedef struct _RTND_REQUEST {
	RTND_OID			Oid;
	void*		 		InformationBuffer;
	unsigned int		InformationBufferLength;
}RTND_REQUEST, *PRTND_REQUEST;


//
//Statistics info structure
//
typedef struct _RTND_STATISTICS_INFO{
   unsigned long			SupportedStatistics;
   unsigned long			BytesRcv; 
   unsigned long			BytesXmit; 
   unsigned long			RcvDiscards;
   unsigned long			XmitDiscards;
   unsigned long			RcvOk;
   unsigned long			XmitOk;
   unsigned long			RcvError; 
   unsigned long			XmitError; 
   unsigned long			RcvMulticastPackets;
   unsigned long			XmitMulticastPackets;
   unsigned long			RcvBroadcastPackets; 
   unsigned long			XmitBroadcastPackets;
} RTND_STATISTICS_INFO, *PRTND_STATISTICS_INFO;

//
//Enum device values used to query network device pointer.
//
typedef enum _device_value{
	DEVICE_NAME,
	DEVICE_MAC
}DEVICE_VALUE;

//
// Stack functions
//
// This function sets the gateway address (in the stack) for an instance of the driver.
RTNPROC int RtnSetDefaultGateway(void *ndp, unsigned long Gateway);
// This function returns a pointer to an RT-TCP/IP Stack packet structure.
RTNPROC void *RtnGetPacket(void *ndp, long Length);
// This function notifies the RT-TCP/IP stack that the most recent packet handed to RtndTransmit should be returned 
// to the stack buffer pool for reuse.
RTNPROC void RtnTransmitDone(void *ndp);
// This function sets the hardware address (in the stack) for an instance of the driver.
RTNPROC void RtnSetLinkAddress(void *ndp, unsigned char *Addr, long Length, int txpendlimit);
// This function gets the network device pointer associated with ASCII network device name.
RTNPROC void *RtnGetDevicePtr(void *pBuffer, DEVICE_VALUE eVal);
// This function first validates if it has received a proper multicast IPv4 address. Then it adds multicast IPv4 address 
// to the routing table on the selected device. After allocating a new entry in the IGMP table, it then sends the IGMP 
// report of the new entry.
RTNPROC int RtnAddMultiRoute(char *DeviceName, int Family, unsigned long IpAddress);
// This function first validates if it has received a proper multicast IPv4 address. It then deletes the multicast IPv4 
// address from the routing table on the selected device.
RTNPROC int RtnDeleteMultiRoute(char *DeviceName, int Family, unsigned long IpAddress);
// This function notifies the stack that data has been received.
RTNPROC void RtnNotifyRecv(void *ndp, unsigned long recvCount);

extern unsigned nextTimerExecuteTs;
//
// Utility functions
//

// This function is used in RtndIoctl to determine the multicast address count.
RTNPROC int RtnGetMcastCount(void *ndp);
// This function converts a binary IP address in network byte order into the network device pointer for the particular device.
RTNPROC void *RtnGetDeviceFromIpAddress(unsigned long IpAddress);
// This function accesses various fields in the stack defined packet structure.
RTNPROC void RtnDecodePacket(void *mptr,
				  unsigned long** ndp,
				  unsigned long** data,
				  unsigned long *length);
// This functions sets the value at offset index of the network device pointed to by ndp.
RTNPROC void RtnSetDataLong(void *ndp, long Offset, unsigned long Data);
// This function retrieves the value placed at offset index of the network device pointed to by ndp.
RTNPROC unsigned long RtnGetDataLong(void *ndp, long Offset);
// This function returns a pointer to the ASCII character name associated with this driver instance.
RTNPROC char *RtnGetDeviceName(void *ndp);
// This function returns the IP address associated with this device instance.
RTNPROC unsigned long RtnGetIpAddress(void *ndp);
// This function exits a CriticalLock section.
RTNPROC void RtnLeaveCriticalLock(CriticalLock *pLock);
// This function waits on the associated CriticalLock until no other thread is inside a RtnEnterCritcalLock/RtnLeaveCriticalLock 
// pairing.
RTNPROC void RtnEnterCriticalLock(CriticalLock *pLock);
// This function initializes a CriticalLock data structure.
RTNPROC void RtnInitializeCriticalLock(CriticalLock *pLock);
// This function frees any associated memory from a CriticalLock.
RTNPROC void RtnDeleteCriticalLock(CriticalLock *pLock);
// This function returns the tick interval, in milliseconds per tick, that the stack timer is configured to use.
RTNPROC int RtnGetMsPerTick(void);
// This function forwards a request to the underlying driver to query the requested capabilities and/or statistics of the RTX64-converted 
// NIC card.
RTNPROC void RtnRequest(PRTND_STATUS Status, PCHAR DeviceName,PRTND_REQUEST	RtndRequest);
// This function reports the status for an instance of the driver for the RTX64-converted NIC card.
RTNPROC void RtnIndicateStatus(void *ndp, PRTND_REQUEST	RtndRequest, PRTND_STATUS	Status);


//RTNPROC int RtnInstallStaticRoute(void *ndp, const char *pTarget, 
//								int prefixLen, const char *pNextHop);

// This function installs a static IPv4 or IPv6 route in RT-TCP/IP stack's forwarding database.
RTNPROC int RtnInstallStaticRoute(char * pDevName, const char *pTarget, int prefixLen, const char *pNextHop, int metric);
// This function removes a static IPv4 or IPv6 route from RtTcpIP stack's forwarding database.
RTNPROC int RtnRemoveStaticRoute(void *ndp, const char *pTarget, 
								 int prefixLen, const char *pNextHop);
// This function displays all static IPv4 and IPv6 routes from RT-TCP/IP stack's forwarding database.
RTNPROC void RtnDisplayRoutingTable(void);
// This function displays the neighbor cache entry, the neighboring node's IPv6 address, the corresponding link-layer address, and the 
// state of the neighbor cache entry for the network device identifier.
RTNPROC void RtnDisplayNbrCacheTable(void *ndp);

RTNDPROC void RtndRequest (
							PRTND_STATUS	Status,
							char*	 		DeviceName,
							PRTND_REQUEST	RtndRequest
						  );
// This function retrieves the online status of the RT-TCP/IP stack for link status monitoring.
RTNPROC BOOL RtnIsStackOnline();
// This function retrieves the online status of a network device for link status monitoring.
RTNPROC BOOL RtnIsDeviceOnline(char *szDeviceName);

RTNPROC BOOL RtnAttachProcessExitHandler(LPTHREAD_START_ROUTINE pRoutine, LPVOID pContext, ULONG Priority);
RTNPROC BOOL RtnReleaseProcessExitHandler(void);

BOOL 
GetDwordRegistryValue(
					HKEY hive,
					LPCTSTR key, 
					LPCTSTR value, 
					DWORD *dw
					);

BOOL 
GetStringRegistryValue(HKEY hive, 
						LPCTSTR key, 
						LPCTSTR value, 
						LPCTSTR buffer, 
						DWORD size
						);

BOOL GetMultystringRegistryValue( HKEY hive, LPCTSTR key, LPCTSTR pValueName , int * length , int * maxStringSize , LPCTSTR * result);
		
#ifndef __RTNAPI__
// This function calls the fnCheckPciCard function for every PCI card found within the system (or until fnCheckPciCard returns TRUE).
RTNPROC int RtnEnumPciCards
(
	int (*fnCheckPciCard)
	(
		PPCI_COMMON_CONFIG pPciInfo,
		ULONG bus,
		PCI_SLOT_NUMBER SlotNumber,
		PVOID pCardInfo,
		int fMacAddressPresent
	),
	PVOID pCardInfo,
	int fMacAddressPresent
);
#endif

RTNPROC
// This function is called when the Stack or application requires that the driver configure certain NIC modes or characteristics.  
void RtnIOCTLDriver(void *ndp,
  			   int cmd,
			   char *addr);
RTNPROC
// This function forwards a request to the underlying driver to query the requested capabilities and/or statistics of the RTX64 converted 
// NIC card.
void RtnRequest(PRTND_STATUS	Status,
		   PCHAR	 		DeviceName,
		   PRTND_REQUEST	RtndRequest);
//  
// Stack called (only) utility functions
//
typedef int (*FNINITTYPE)(int fDisplayErrorMessages,
							char *deviceName,
							PRTNINTERFACE pNetInterface);
typedef int (*FNCONFIGTYPE)(PRTNINTERFACE pNetInterface);
typedef int (*FNUPDOWNTYPE)(void *ndp,      
						 unsigned short flags,      
						 char *options );
typedef int (*FNTRANSMITTYPE)(void *mp);
typedef int (*FNIOCTLTYPE)(  void *ndp,      
							int cmd,      
							char *addr );
typedef int (*FNRECEIVETYPE)(void *ndp);

typedef int (*FNREQUESTTYPE)(PRTND_STATUS	Status,
							 PCHAR	 		DeviceName,
							 PRTND_REQUEST	RtndRequest);

typedef BOOL (*FNRECEIVEFILTER)(
	ETHERNETHEADER *pEthernetHeader,
	void *pData,
	unsigned long ulEthernetDataSize
	);
typedef BOOL (*FNRECEIVEFILTEREX)(
	ETHERNETHEADER *pEthernetHeader,
	void *pData,
	unsigned long ulEthernetDataSize,
	void *ndp
	);


typedef BOOL (*FNTRANSMITFILTER)(
	ETHERNETHEADER *pEthernetHeader,
	void *pData,
	unsigned long ulEthernetDataSize
	);
typedef BOOL (*FNTRANSMITFILTEREX)(
	ETHERNETHEADER *pEthernetHeader,
	void *pData,
	unsigned long ulEthernetDataSize,
	void *ndp
	);


typedef void (*FNCONFIGFILTER)(
	void *ndp
	);


#ifdef __cplusplus
}
#endif


