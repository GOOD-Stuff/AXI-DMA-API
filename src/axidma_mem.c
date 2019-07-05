#include "axidma_mem.h"

static const char *mem_path = "/dev/mem";

/**
 * @brief Allocate memory for data transfers
 * @param[out] membufinstptr  - pointer to memory_buffer structure;
 * @param[in]  buffer_address - base address of buffer place;
 * @param[in]  size 		  - size of buffer (in bytes)
 *
 * @return MEM_OK          - if memory was allocate successfully;
 *  	   ERR_MEM_SIZE    - if size incorrect;
 *  	   ERR_MEM_FD_OPEN - if can't get access to /dev/mem;
 *  	   ERR_MEM_MAP     - if can't allocate memory;
 * @note After using this function, user can't get access to file_descriptor field
 *
 * @author Chemodanov Maxim
 */
uint32_t memory_buffer_create(memory_buffer_t *membufinstptr, uint32_t buffer_address,
	                          size_t size) {
	if (size == 0)
		return ERR_MEM_SIZE;

	membufinstptr->size            = size;
	membufinstptr->file_descriptor = open(mem_path, O_RDWR | O_SYNC);
	if (membufinstptr->file_descriptor < 0)
		return ERR_MEM_FD_OPEN;

	membufinstptr->buffer_address = buffer_address;
	membufinstptr->baseaddress = mmap(NULL, size *
			                          sizeof(membufinstptr->baseaddress[0]),
									  PROT_READ | PROT_WRITE,
			                          MAP_SHARED,
									  membufinstptr->file_descriptor,
									  membufinstptr->buffer_address);
	if (membufinstptr->baseaddress == MAP_FAILED)
		return ERR_MEM_MAP;

	close(membufinstptr->file_descriptor);
	return MEM_OK;
}


/**
 * @brief Get offset from packet
 * @param[in] membufinstptr - pointer to memory_buffer structure;
 * @param[in] packet_addr   - address of packet data;
 *
 * @return offset from packet data;
 *
 * @author Chemodanov Maxim
 */
uint32_t memory_buffer_get_offset(memory_buffer_t *membufinstptr, uint32_t packet_addr) {
	uint32_t offset = packet_addr - membufinstptr->buffer_address;

	return ((uint32_t)membufinstptr->baseaddress + offset);
}
