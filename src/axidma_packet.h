#ifndef AXIDMA_PACKET_H
#define AXIDMA_PACKET_H

#include <stdint.h>			//for using uint32_t
#include <sys/mman.h>
#include <fcntl.h>			//for file system function open()
#include <stdlib.h>
#include <unistd.h>


#define PACKET_BUFFER_SIZE			256


#define PKT_OK						0
#define ERR_PKT_BAD_INDEX			21


typedef struct packet {
	uint32_t* baseaddr;
	uint32_t buffer_address;
	uint32_t c_flaq;
	int size;
} packet;


typedef struct packet_buffer {
	packet pktarray[256];
	int count;
	int current_index;
	int proc_index;
} packet_buffer;


uint32_t 	packet_buffer_push	(packet_buffer *pktbufinstptr,
		                         uint32_t buffer_address, uint32_t size);
packet 		packet_buffer_get	(packet_buffer *pktbufinstptr);
uint32_t 	packet_buffer_free	(packet_buffer *pktbufinstptr);

#endif // AXIDMA_PACKET_H