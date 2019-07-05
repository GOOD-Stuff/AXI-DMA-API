#ifndef AXIDMA_MEM_H
#define AXIDMA_MEM_H

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>


#define MEM_OK						0
#define ERR_MEM_SIZE				18
#define ERR_MEM_MAP					19
#define ERR_MEM_FD_OPEN				20



typedef struct memory_buffer {
	size_t 		size;					// Size of buffer in bytes
	int 		file_descriptor;		// file descriptor (/dev/mem)
	uint8_t*	baseaddress;			// base memory address
	uint32_t	buffer_address;			// physical address of the data buffer
} memory_buffer_t;



uint32_t memory_buffer_create	   (memory_buffer_t *membufinstptr,
		 	  	   	   	    	    uint32_t buffer_address, size_t size);
uint32_t memory_buffer_get_offset  (memory_buffer_t *membufinstptr,
		                            uint32_t packet_addr);
uint32_t memory_buffer_set_counter (memory_buffer_t *membufinstptr, size_t size);
void     memory_buffer_print	   (memory_buffer_t *instance_ptr);

#endif // AXIDMA_MEM_H