#ifndef REMORACOMMS_H
#define REMORACOMMS_H

#include <cstring>

extern "C"
{
	#include "lwip/api.h"
	#include "lwip/pbuf.h"
	#include "lwip/udp.h"
	#include "lwip/tcp.h"
}

#include "configuration.h"
#include "remora.h"

#include "../module.h"

class RemoraComms  : public Module
{
  private:

    volatile rxData_t*  ptrRxData;
    volatile txData_t*  ptrTxData;
    rxData_t            rxBuffer;
    uint8_t             rejectCnt;
    bool                data;
    bool                dataError;

    struct netif 		gnetif;
    ip4_addr_t 			ipaddr;
    ip4_addr_t 			netmask;
    ip4_addr_t 			gw;
    uint8_t 			IP_ADDRESS[4];
    uint8_t 			NETMASK_ADDRESS[4];
    uint8_t 			GATEWAY_ADDRESS[4];

	struct udp_pcb*		upcb;				// UDP Control Block structure
	ip_addr_t 			myIPADDR;

	void processPacket(void *, struct udp_pcb*, struct pbuf* , const ip_addr_t*, u16_t);

  public:

	RemoraComms(volatile rxData_t*, volatile txData_t*);

	void update();
    void init(void);
    bool getStatus(void);
    void setStatus(bool);
    bool getError(void);
    void setError(bool);

};




#endif
