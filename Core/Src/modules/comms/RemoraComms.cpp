#include "RemoraComms.h"


RemoraComms::RemoraComms(volatile rxData_t* ptrRxData, volatile txData_t* ptrTxData) :
	ptrRxData(ptrRxData),
	ptrTxData(ptrTxData)
{
	printf("Creating an Ethernet communication module\n");

	  /* IP addresses initialization */
	  this->IP_ADDRESS[0] = 10;
	  this->IP_ADDRESS[1] = 10;
	  this->IP_ADDRESS[2] = 10;
	  this->IP_ADDRESS[3] = 10;
	  this->NETMASK_ADDRESS[0] = 255;
	  this->NETMASK_ADDRESS[1] = 255;
	  this->NETMASK_ADDRESS[2] = 255;
	  this->NETMASK_ADDRESS[3] = 0;
	  this->GATEWAY_ADDRESS[0] = 10;
	  this->GATEWAY_ADDRESS[1] = 10;
	  this->GATEWAY_ADDRESS[2] = 0;
	  this->GATEWAY_ADDRESS[3] = 1;


	  /* Initilialize the LwIP stack without RTOS */
	  lwip_init();

	  /* IP addresses initialization without DHCP (IPv4) */
	  IP4_ADDR(&this->ipaddr, this->IP_ADDRESS[0], this->IP_ADDRESS[1], this->IP_ADDRESS[2], this->IP_ADDRESS[3]);
	  IP4_ADDR(&this->netmask, this->NETMASK_ADDRESS[0], this->NETMASK_ADDRESS[1] , this->NETMASK_ADDRESS[2], this->NETMASK_ADDRESS[3]);
	  IP4_ADDR(&this->gw, this->GATEWAY_ADDRESS[0], this->GATEWAY_ADDRESS[1], this->GATEWAY_ADDRESS[2], this->GATEWAY_ADDRESS[3]);

	  /* add the network interface (IPv4/IPv6) without RTOS */
	  netif_add(&this->gnetif, &this->ipaddr, &this->netmask, &this->gw, NULL, &this->ethernetif_init, &this->ethernet_input);

	  /* Registers the default network interface */
	  netif_set_default(&this->gnetif);

	  if (netif_is_link_up(&this->gnetif))
	  {
	    /* When the netif is fully configured this function must be called */
	    netif_set_up(&this->gnetif);
	  }
	  else
	  {
	    /* When the netif link is down this function must be called */
	    netif_set_down(&this->gnetif);
	  }

	  /* Set the link callback function, this function is called on change of link status*/
	  netif_set_link_callback(&this->gnetif, ethernetif_update_config);
}


void RemoraComms::init()
{
	err_t err;

	this->upcb = udp_new();

	IP_ADDR4(&myIPADDR, 10, 10, 10, 10);

	err = udp_bind(upcb, &myIPADDR, 27181);  // 27181 is the server UDP port

	if(err == ERR_OK)
	{
	   udp_recv(upcb, &(RemoraComms::processPacket), NULL);
	}
	else
	{
	   udp_remove(upcb);
	}
}


void RemoraComms::processPacket(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	int txlen;
	struct pbuf *txBuf;

	// copy the UDP payload into the rxData structure
	memcpy(&this->rxBuffer.rxBuffer, p->payload, p->len);

	if (rxBuffer.header == PRU_READ)
	{
		txData.header = PRU_DATA;
		txlen = BUFFER_SIZE;
	}
	else if (rxBuffer.header == PRU_WRITE)
	{
		txData.header = PRU_ACKNOWLEDGE;
		txlen = sizeof(txData.header);

		// then move the data
		for (int i = 0; i < BUFFER_SIZE; i++)
		{
			rxData.rxBuffer[i] = rxBuffer.rxBuffer[i];
		}
	}


	// allocate pbuf from RAM
	txBuf = pbuf_alloc(PBUF_TRANSPORT, txlen, PBUF_RAM);

	// copy the data into the buffer
	pbuf_take(txBuf, (char*)&txData.txBuffer, txlen);

	// Connect to the remote client
	udp_connect(upcb, addr, port);

	// Send a Reply to the Client
	udp_send(upcb, txBuf);

	// free the UDP connection, so we can accept new clients
	udp_disconnect(upcb);

	// Free the p_tx buffer
	pbuf_free(txBuf);

	// Free the p buffer
	pbuf_free(p);
}
