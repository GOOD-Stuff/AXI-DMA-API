#include "axidma_packet.h"


uint32_t packet_buffer_push(packet_buffer *pktbufinstptr, uint32_t buffer_address,
		                    uint32_t size) {

	if (pktbufinstptr->proc_index == PACKET_BUFFER_SIZE-1) {
		return ERR_PKT_BAD_INDEX;
	}

	pktbufinstptr->pktarray[pktbufinstptr->proc_index].buffer_address = buffer_address;
	pktbufinstptr->pktarray[pktbufinstptr->proc_index].size = size;
	pktbufinstptr->pktarray[pktbufinstptr->proc_index].c_flaq = 1;
	pktbufinstptr->proc_index ++;
	return PKT_OK;
}



packet packet_buffer_get(packet_buffer *pktbufinstptr) {

	packet temp = pktbufinstptr->pktarray[pktbufinstptr->current_index];
	pktbufinstptr->current_index++;
	if (pktbufinstptr->current_index == pktbufinstptr->proc_index) {
		pktbufinstptr->current_index = 0;
	}
	return temp;
}



uint32_t packet_buffer_free(packet_buffer *pktbufinstptr) {
	if (pktbufinstptr->proc_index != 0) {
		pktbufinstptr->proc_index = 0;
	}
	return 0;
}
