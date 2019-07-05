#include "axidma_bdring.h"


/**
 * @brief Initialization of BDRing.
 * Mandatory function. Without it, the ring can not be initialized
 *
 * @param[out] ringinstptr - pointer to BdRing;
 * @param[in]  bdbase      - address of the beginning of the buffer descriptors;
 * @param[in]  buffer_base - start address of the buffer of data;
 * @param[in]  buffer_size - size of buffer;
 * @param[in]  init_mode   - initialization mode;
 *
 * @return status:
 *			- DMA_OK           - if the initialization was successful
 *			- ERR_RING_UNINIT  - If you encounter problems during initialization
 *			- ERR_RING_OPEN_FD - if an error occurs at the opening level of the
 *	file descriptor
 * @note Initialization modes - the developer has a choice of how to initialize
 *   the bdring structure when INIT_DEFAULT_CFG is initialized by the base
 *   parameters. These include the size of the descriptor in memory, the number
 *   of bytes of the buffer per descriptor. This makes it possible to implement
 *   the chain of descriptors without problems. After such initialization,
 *   the initialized flag for bdring = 1. With INIT_CUSTOM_CFG, the base values
 *   are entered later.
 *
 * @date 10.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bdring_initialize(bdring_t *ringinstptr,
						   uint32_t bdbase,
						   uint32_t buffer_base,
						   uint32_t buffer_size,
						   int init_mode) {
	/* Setting the value that bdring is not initialized */
	bdring_set_uninit_state(ringinstptr);

	ringinstptr->file_descriptor = open("/dev/mem", O_RDWR | O_SYNC);
	if (ringinstptr->file_descriptor < 0)
		return ERR_RING_OPEN_FD;

	/* Set the physical start address of the chain */
	ringinstptr->bdring_phys_baseaddr = bdbase;
	ringinstptr->buffer_phys_baseaddr = buffer_base;

	int status = bdring_set_buffer_size(ringinstptr, buffer_size);
	if (status != RING_OK)
		return status;

	bdring_set_buffer_lastaddr(ringinstptr, buffer_base, buffer_size);

	ringinstptr->bd_alignment  = BD_MIN_ALIGNMENT;
	ringinstptr->buffer_per_bd = BYTES_PER_DESC;

	int bdcnt = api_get_bdcnt(BYTES_PER_DESC, buffer_size);

	status = bdring_set_bdcount(ringinstptr, bdcnt);
	if (status != RING_OK)
		return status;

	uint32_t bd_space_size = bdcnt * BD_MIN_ALIGNMENT;

	status = bdring_set_bdring_size(ringinstptr, bd_space_size);
	status = bdring_set_lastaddr   (ringinstptr, bdbase + bd_space_size);

	status = bdring_alloc(ringinstptr);
	if (status != RING_OK)
		return status;


	if (init_mode == INIT_RXCHAN_CFG) {
		bdring_init_rx_bd(ringinstptr);
		ringinstptr->isRx = 1;
	}

	if (init_mode == INIT_TXCHAN_CFG) {
		bdring_init_tx_bd(ringinstptr);
		ringinstptr->isRx = 0;
	}

	bdring_set_init_state(ringinstptr);

	return RING_OK;
}

/*
 * @brief Set init_state in bdring to 1
 * @param[out] ringinstptr - pointer to bdring
 *
 * @return none

 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
void bdring_set_init_state(bdring_t *ringinstptr) {
	ringinstptr->initialized = 1;
}

/*
 * @brief Set init_state in bdring to 0
 * @param[out] ringinstptr - pointer to bdring
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
void bdring_set_uninit_state(bdring_t *ringinstptr) {
	ringinstptr->initialized = 0;
}


/**
 * @brief Returns the current state of bdring
 * @param[out] ringinstptr - pointer to bdring
 *
 * @return 0 - if bdring is not initialized;
 * 		   1 - if bdring is initialized.
 * @note Without initialization, it is impossible to work with a chain of
 *   descriptors. This function allows you to check the status before starting
 *   the dma.
 *
 * @date 10.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bdring_get_state(bdring_t *ringinstptr) {
	return ringinstptr->initialized;
}


/*
 * @brief Sets the size of the data buffer with which the entire descriptor
 *   chain is running.
 * @param[out] ringinstptr - handler of bdring
 * @param[in]  buffer_size - size of buffer in bytes
 *
 * @return RING_OK                - if the size request was successful
 * 		   ERR_RING_BAD_BUFFERLEN - if an invalid buffer_size parameter is passed
 *
 * @author Chemodanov Maxim
 **/
uint32_t bdring_set_buffer_size(bdring_t *ringinstptr, int buffer_size) {
	if (buffer_size < 0)
		return ERR_RING_BAD_BUFFERLEN;

	ringinstptr->buffer_size = buffer_size;
	return RING_OK;
}


/*
 * @brief Set the size of the chain of descriptors in bytes. The size is stored
 *   in the bdring structure
 * @param[out] ringinstptr - handler of bdring
 * @param[in]  bdring_size - the size of the descriptor chain in bytes
 *
 * @return RING_OK         - if without errors
 * 		   ERR_BDRING_SIZE - If at the time of setting the size occurred errors
 * @note It is necessary to remember that the size of a chain of descriptors
 *   should be a multiple of the size of one descriptor. we start from the
 *   condition of the multiplicity of the descriptor to the size 0x40
 *
 * @date 10.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t bdring_set_bdring_size(bdring_t *ringinstptr, int bdring_size) {
	if (bdring_size < BD_MIN_ALIGNMENT)
		return ERR_BDRING_SIZE;

	if (bdring_size % BD_MIN_ALIGNMENT != 0)
		return ERR_BDRING_SIZE;

	ringinstptr->bdring_size = bdring_size;

	return RING_OK;
}


/*
 * @brief Sets the bd_alignment factor, which is responsible for the size of
 *   the descriptor in memory.
 * @param[out] ringinstptr - handle of bdring;
 * @param[in]  align       - size of bd_align.
 *
 * @return RING_OK        - if everything went well
 * 		   ERR_RING_ALIGN - if an attempt is made to initialize with invalid values
 * @note The function is needed when it is necessary
 *   to institute a check upon initialization of buffer descriptors.
 *   Because of the documentation, the size of the descriptor must be specified
 *   by certain conditions, such as:
 *     1) the size should not be less than 0;
 *     2) the size must not be less than 64 bytes;
 *     3) The size must not be a multiple of 0x40.
 *
 * @date 10.04.2017
 * @author Chemodanov Maxim
 **/
uint32_t bdring_set_bd_align(bdring_t *ringinstptr, int align) {
	if      (align < 0)            		return ERR_RING_ALIGN;
	else if (align < BD_MIN_ALIGNMENT)  return ERR_RING_ALIGN;

	if (align % BD_MIN_ALIGNMENT != 0){
		return ERR_RING_ALIGN;
	}
	ringinstptr->bd_alignment = align;
	return RING_OK;
}


/*
 * @brief Function sets lastaddr value in ringinstptr
 * @param[out] ringinstptr - handle of bdring
 * @param[in]  buffer_base - the value of the address of the beginning of
 *   the data buffer
 * @param[in]  buffer_size - size in bytes
 *
 * @return none
 *
 * @date 10.04.2017
 * @author Chemodanov Maxim
 **/
void bdring_set_buffer_lastaddr(bdring_t *ringinstptr, uint32_t buffer_base,
		                             uint32_t buffer_size) {
	uint32_t lastaddr = buffer_base + buffer_size;
	ringinstptr->buffer_phys_lastaddr = lastaddr;
}


/*
 * @brief Sets the size of the buffer to the handle
 * @param[out] ringinstptr        - handle of bdring;
 * @param[in]  buffer_per_bd_size - buffer size per descriptor in bytes.
 *
 * @return RING_OK - if buffer_per_bd_size is set;
 *         ERR_BPD - if the size of the bytes_per_descriptor is larger than
 * the size of the entire buffer
 *
 * @note Checking the buffer_per_bd_size value is necessary in order to avoid
 * the moment at which the buffer size under the entire DMA is allocated less
 * than buffer_per_bd_size. Hence it follows that for correctness
 * it is necessary to perform the function bdring_initialization, into which
 * the buffer size is explicitly transferred.
 *
 * @date 10.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t bdring_set_buffer_per_bd(bdring_t *ringinstptr,
		                          uint32_t buffer_per_bd_size) {
	if ((buffer_per_bd_size == 0) ||
	    (buffer_per_bd_size > ringinstptr->buffer_size))
		return ERR_BPD;

	ringinstptr->buffer_per_bd = buffer_per_bd_size;

	return RING_OK;
}


/*
 * @brief Setting the bdcount value to the bdring_t structure
 * @param[out] ringinstptr - handle of bdring;
 * @param[in]  bd_cnt      - count of descriptors;
 *
 * @return RING_OK   - if value of bd_cnt is set;
 *         ERR_BDCNT - if value of bd_cnt < 1;
 * @note This is necessary, since we will constantly use it
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t bdring_set_bdcount (bdring_t *ringinstptr, int bd_cnt) {
	if (bd_cnt < 1)
		return ERR_BDCNT;
	ringinstptr->bd_count = bd_cnt;

	return RING_OK;
}

/**
 * @brief Setting the last address in the bdring structure;
 * @param[out] ringinstptr       - handle of bdring;
 * @param[in]  bd_space_lastaddr - the last address of the chain of descriptors;
 *
 * @return RING_OK           - if the value of the last address is set to bdring
 * 		   ERR_RING_LASTADDR - if lastaddr is less than the initial value
 * @note To explicitly define boundaries.
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bdring_set_lastaddr(bdring_t *ringinstptr, uint32_t bd_space_lastaddr) {
	if (ringinstptr->bdring_phys_baseaddr > bd_space_lastaddr){
		return ERR_RING_LASTADDR;
	}
	ringinstptr->bdring_phys_lastaddr = bd_space_lastaddr;

	return RING_OK;
}


/**
 * @brief Counts the number of descriptors in the chain based on the buffer size
 *   parameters and the number of bytes of the buffer per descriptor.
 * @param[in] buffer_per_bd_size - count of bytes by descriptor;
 * @param[in] buffer_size        - number of bytes of the whole buffer.
 *
 * @return Number of descriptors
 * @note Function performs only counting
 *
 * @date 10.04.2017
 * @author Chemodanov maxim
 */
uint32_t api_get_bdcnt (int buffer_per_bd_size, int buffer_size) {
	int taildesc_factor = 0;

	if (buffer_size % buffer_per_bd_size != 0)
		taildesc_factor ++;

	return ((buffer_size / buffer_per_bd_size) + taildesc_factor);
}


/**
 * @brief Get tail address of memory
 * @param[in] headmem_addr - address of memory;
 * @param[in] bd_cnt       - number of descriptors;
 *
 * @return Tail address of memory;
 *
 * @author Chemodanov maxim
 */
uint32_t api_get_tailmem_addr (uint32_t headmem_addr, int bd_cnt) {
	return (headmem_addr + ((bd_cnt-1) * BD_MIN_ALIGNMENT));
}


/**
 * @brief The function of executing the mapping to the memory of
 *   the correct size so that you can initialize a chain of buffer descriptors
 *   in the physical area.
 * @param[out] ringinstptr - handle of bdring
 *
 * @return RING_OK      - if memory mapping was successfully;
 * 		   ERR_RING_MAP - if memory mapping is not performed.
 * @note It is necessary to take care that bdring_size and the file descriptor
 *   are open. You also need to take care that the bdring structure has been
 *   passed the correct address of  bdring
 *   Added: Now where the markup - there is cleaning the memory for descriptors
 *
 * @date 15.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t bdring_alloc (bdring_t *ringinstptr) {
	ringinstptr->bdring_virt_baseaddr = (uint32_t*)mmap(NULL,
			                                            ringinstptr->bdring_size,
														PROT_READ | PROT_WRITE,
														MAP_SHARED,
														ringinstptr->file_descriptor,
														ringinstptr->bdring_phys_baseaddr);
	if (ringinstptr->bdring_virt_baseaddr == MAP_FAILED)
		return ERR_RING_MAP;

	bdring_clear_mem(ringinstptr);

	return RING_OK;
}


#define PRINT_BD_INFO_RX 0
/*
 * @brief The function initializes the descriptors of the bdring. Before
 *   initialization, data must be entered in the bdring structure:
 *   1) the initial address of the chain of descriptors, physical
 *   2) the end address of the chain of descriptors, physical
 *   3) the size of the descriptors (bd_alignment)
 *   4) the number of descriptors (bd_count)
 *   5) virtual start address of the descriptors
 *   6) virtual end address of descriptors
 *   7) the physical address of the buffer
 *   8) buffer size
 *   9) the number of bytes per descriptor
 * @param[in] ringinstptr - handle of bdring
 *
 * @return none
 * @note The function initializes a chain of descriptors. Before calling it,
 *   you need to make sure that all of the above parameters have been set
 *   correctly. Based on the description of the descriptor fields, each descriptor
 *   has 5 32-bit registers for user data. Since the pointer is not passed to
 *   this function, we will use these fields to store the necessary parameters
 *   in the descriptor structure. This is done not to access the bdring structure
 *   every time. The list of parameters is presented below:
 *     field app0 - virtual address of the first descriptor
 *     field app1 - virtual address of the last descriptor
 *     field app2 - placement of the descriptor (bd_alignment)
 *     field app3 - pointer to the previous descriptor (this makes it possible
 *       to implement a bi-directional list for enumerating descriptors not only
 *       in one direction, but also in the opposite direction.
 *   This will speed up the search for the desired descriptor in the analysis,
 *   and should in theory increase the performance of the dma by speeding up the
 *   search. Thanks to the analysis, the initialization of the descriptors is
 *   faster compared to the algorithm proposed in standalone
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
void bdring_init_rx_bd(bdring_t *ringinstptr) {
	/* the number of descriptors in the channel */
	int count_bd	= ringinstptr->bd_count;
	/* the size of the buffer to set the buffer size in the last descriptor */
	int buffer_size	= ringinstptr->buffer_size;
	/* descriptor placement size */
	int alignmem 	= ringinstptr->bd_alignment;

	/* physical address for bd-> bufferaddr */
	uint32_t buffer_addr = ringinstptr->buffer_phys_baseaddr;
	/*The first descriptor address for app_0 */
	uint32_t *headmem_address = bdring_get_head(ringinstptr);
	/* physical address of bdring */
	uint32_t next_address_phys = ringinstptr->bdring_phys_baseaddr;
	/* virtual descriptor address for app_1 */
	uint32_t tailmem_address = 0;
	/* the previous descriptor address for app_3 */
	uint32_t prevmem_address = 0;

	bd_t* current_bd = (bd_t*)bdring_get_head(ringinstptr);

	bd_set_headmem(current_bd, (uint32_t)headmem_address);

	tailmem_address 		 = api_get_tailmem_addr((uint32_t)headmem_address,
			count_bd);
	ringinstptr->bd_headaddr = ringinstptr->bdring_phys_baseaddr;
	ringinstptr->bd_lastaddr = api_get_tailmem_addr(ringinstptr->bd_headaddr,
			count_bd);
	prevmem_address = tailmem_address;

	int bd_index;
	for (bd_index = 0; bd_index < (count_bd - 1); bd_index++) {
		next_address_phys = bdring_get_nextaddr(ringinstptr, next_address_phys);
		bd_set_prevbdmem(current_bd, prevmem_address);

		prevmem_address = bdring_get_nextaddr(ringinstptr, prevmem_address);
		if (bd_index == 0)
			prevmem_address = (uint32_t)current_bd;

		bd_set_tailmem	  (current_bd, tailmem_address);
		bd_set_alignmem	  (current_bd, alignmem);
		bd_set_nextdesc	  (current_bd, next_address_phys);
		bd_set_status	  (current_bd, 0x00); // XXX: ???
		bd_set_length	  (current_bd, BYTES_PER_DESC);
		bd_set_bufferaddr (current_bd, buffer_addr);
		bd_set_headmem	  (current_bd, (uint32_t)headmem_address);

		buffer_addr += ringinstptr->buffer_per_bd;
		buffer_size -= ringinstptr->buffer_per_bd;
		current_bd = (bd_t*)bdring_get_nextaddr(ringinstptr, (uint32_t)current_bd);
	}

	/* do not forget about the last descriptor = pointer to the first */
	bd_set_prevbdmem	(current_bd, prevmem_address);
	bd_set_alignmem		(current_bd, alignmem);
	bd_set_headmem		(current_bd, (uint32_t)headmem_address);
	bd_set_nextdesc		(current_bd, ringinstptr->bdring_phys_baseaddr);
	bd_set_bufferaddr	(current_bd, buffer_addr);
	bd_set_length		(current_bd, buffer_size);
	bd_set_tailmem		(current_bd, tailmem_address);
	/* the end of filling the last descriptor */

#if PRINT_BD_INFO_RX
	current_bd = (bd_t*)bdring_get_head(ringinstptr);
	printf("Count: %d\r\n", count_bd);
	for (bd_index = 0; bd_index < count_bd; bd_index++){

		printf("RX%4d\tNEXT_DESC: 0x%08x\r\n", bd_index, current_bd->nextdesc);
		printf("RX%4d\tBUFF_ADDR: 0x%08x\r\n", bd_index, current_bd->buffer_addr);
		printf("RX%4d\tCTRL_REGR: 0x%08x\r\n", bd_index, current_bd->control);
		printf("RX%4d\tSTTS_REGR: 0x%08x\r\n", bd_index, current_bd->status);
		printf("RX%4d\tAPP_0_FLD: 0x%08x\r\n", bd_index, current_bd->app_0);
		printf("RX%4d\tAPP_1_FLD: 0x%08x\r\n", bd_index, current_bd->app_1);
		printf("RX%4d\tAPP_2_FLD: 0x%08x\r\n", bd_index, current_bd->app_2);
		printf("RX%4d\tAPP_3_FLD: 0x%08x\r\n", bd_index, current_bd->app_3);
		printf("RX%4d\tAPP_4_FLD: 0x%08x\r\n", bd_index, current_bd->app_4);
		printf("\r\n");
		current_bd = (bd_t*)bdring_next_bd(current_bd);

	}
	printf("RX\tCurrentCount BD : %d\r\n", count_bd);
	printf("RX\tBD_PHYS_BASEADDR: 0x%08x\r\n", ringinstptr->bd_headaddr);
	printf("RX\tBD_PHYS_TAILADDR: 0x%08x\r\n", ringinstptr->bd_lastaddr);
#endif
}


#define PRINT_BD_INFO_TX	0
/**
 * @brief
 *
 * @return none
 * @note You can not install CTRL, because it is set in axidma.c _api_send*!
 *
 * @author Chemodanov Maxim
 */
void bdring_init_tx_bd(bdring_t *ringinstptr) {
	int buffer_size = ringinstptr->buffer_size;
	int alignmem 	= ringinstptr->bd_alignment;
	int count_bd    = ringinstptr->bd_count;

	uint32_t buffer_addr 	   = ringinstptr->buffer_phys_baseaddr;
	uint32_t *headmem_address  = bdring_get_head(ringinstptr);
	uint32_t next_address_phys = ringinstptr->bdring_phys_baseaddr;
	uint32_t tailmem_address   = 0;
	uint32_t prevmem_address   = 0;

	bd_t *current_bd = (bd_t*)bdring_get_head(ringinstptr);

	tailmem_address 		 = api_get_tailmem_addr((uint32_t)headmem_address,
			count_bd);
	ringinstptr->bd_headaddr = ringinstptr->bdring_phys_baseaddr;
	ringinstptr->bd_lastaddr = api_get_tailmem_addr(ringinstptr->bd_headaddr,
			count_bd);
	prevmem_address 		 = tailmem_address;

	int bd_index;
	for (bd_index = 0; bd_index < (count_bd - 1); bd_index++) {
		next_address_phys = bdring_get_nextaddr(ringinstptr, next_address_phys);

		bd_set_prevbdmem(current_bd, prevmem_address);
		prevmem_address = bdring_get_nextaddr(ringinstptr, prevmem_address);

		if (bd_index == 0)
			prevmem_address = (uint32_t)current_bd;

		bd_set_tailmem	  (current_bd, tailmem_address);
		bd_set_alignmem	  (current_bd, alignmem);
		bd_set_nextdesc	  (current_bd, next_address_phys);
		bd_set_bufferaddr (current_bd, buffer_addr);
		bd_set_headmem	  (current_bd, (uint32_t)headmem_address);

		buffer_addr += ringinstptr->buffer_per_bd;
		buffer_size -= ringinstptr->buffer_per_bd;

		current_bd = (bd_t*)bdring_get_nextaddr(ringinstptr, (uint32_t)current_bd);
	}
	bd_set_prevbdmem  (current_bd, prevmem_address);
	bd_set_alignmem	  (current_bd, alignmem);
	bd_set_headmem	  (current_bd, (uint32_t)headmem_address);
	bd_set_nextdesc	  (current_bd, ringinstptr->bdring_phys_baseaddr);
	bd_set_bufferaddr (current_bd, buffer_addr);
	bd_set_tailmem	  (current_bd, tailmem_address);

#if PRINT_BD_INFO_TX
	current_bd = (bd_t*)bdring_get_head(ringinstptr);
	for (bd_index = 0; bd_index < count_bd; bd_index++){
		printf("TX%4d\tNEXT_DESC: 0x%08x\r\n", bd_index, current_bd->nextdesc);
		printf("TX%4d\tBUFF_ADDR: 0x%08x\r\n", bd_index, current_bd->buffer_addr);
		printf("TX%4d\tCTRL_REGR: 0x%08x\r\n", bd_index, current_bd->control);
		printf("TX%4d\tSTTS_REGR: 0x%08x\r\n", bd_index, current_bd->status);
		printf("TX%4d\tAPP_0_FLD: 0x%08x\r\n", bd_index, current_bd->app_0);
		printf("TX%4d\tAPP_1_FLD: 0x%08x\r\n", bd_index, current_bd->app_1);
		printf("TX%4d\tAPP_2_FLD: 0x%08x\r\n", bd_index, current_bd->app_2);
		printf("TX%4d\tAPP_3_FLD: 0x%08x\r\n", bd_index, current_bd->app_3);
		printf("TX%4d\tAPP_4_FLD: 0x%08x\r\n", bd_index, current_bd->app_4);
		printf("\r\n");
		current_bd = (bd_t*)bdring_next_bd(current_bd);
	}
	printf("TX\tCurrentCount BD : %d\r\n", count_bd);
	printf("TX\tBD_PHYS_BASEADDR: 0x%08x\r\n", ringinstptr->bd_headaddr);
	printf("TX\tBD_PHYS_TAILADDR: 0x%08x\r\n", ringinstptr->bd_lastaddr);
#endif
}


/*
* @brief Clearing physical memory for descriptors
* @param[out] ringinstptr - handle of bdring.
*
* @return none
* @note Works through memset. When calling this function, you need to make sure
*   that bdring is initialized with the values of bdring_size, and
*   the base address
*
* @date 11.04.2017
* @author Chemodanov Maxim
* */
void bdring_clear_mem(bdring_t *ringinstptr) {
	uint32_t *bdring_base = bdring_get_head(ringinstptr);
	memset(bdring_base, 0x00, ringinstptr->bdring_size);
}


/*
 * @brief Returns the virtual address of the beginning of the chain of descriptors
 * @param[in] ringinstptr - handle of bdring
 *
 * @return Pointer to the first descriptor
 * @note Before calling, you need to make sure that we initialized the memory
 *   mapping to descriptors
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t *bdring_get_head(bdring_t *ringinstptr) {
	return ringinstptr->bdring_virt_baseaddr;
}


/**
 * @brief Returns a pointer to the following descriptor
 * @param[in] current_bd - pointer to the current descriptor.
 *
 * @return The pointer to the following descriptor.
 * @note The function works with a virtual address.
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bdring_next_bd(bd_t* current_bd) {
	if ((uint32_t)current_bd == (uint32_t)bd_get_tailmem(current_bd))
		return bd_get_headmem(current_bd);
	else
		return (uint32_t)current_bd + bd_get_alignmem(current_bd);
}


/*
 * @brief Returns a pointer to the previous handle
 * @param[in] current_bd - the pointer to the current descriptor
 *
 * @return The pointer to the previous handle.
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t bdring_prev_bd	(bd_t* current_bd) {
	if ((uint32_t)current_bd == (uint32_t)bd_get_headmem(current_bd))
		return bd_get_tailmem(current_bd);
	else
		return (uint32_t)(current_bd) - bd_get_alignmem(current_bd);
}


/*
 * @brief Calculates the following address
 * @param[in] ringinstptr     - handle of bdring
 * @param[in] current_address - current address value
 *
 * @return The next address
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t bdring_get_nextaddr (bdring_t *ringinstptr, uint32_t current_address) {
	uint32_t alignment_factor = ringinstptr->bd_alignment;
	return alignment_factor + current_address;
}


/*
 * @brief Sets the data in the app0 field of the descriptor
 * @param[out] bdinstptr - handle of descriptor;
 * @param[in]  data0     - data for field APP0 (SG descriptor's field)
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
void bd_set_app0 (bd_t *bdinstptr, uint32_t data0) {
	bdinstptr->app_0 = data0;
}


/*
 * @brief Sets the data in the app1 field of the descriptor
 * @param[out] bdinstptr - handle of desecriptor;
 * @param[in]  data1     - data for field APP1;
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_app1 (bd_t *bdinstptr, uint32_t data1) {
	bdinstptr->app_1 = data1;
}


/*
 * @brief Sets the data in the app2 field of the descriptor
 * @param[out] bdinstptr - handle of desecriptor;
 * @param[in]  data2     - data for field APP2;
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_app2 (bd_t *bdinstptr, uint32_t data2) {
	bdinstptr->app_2 = data2;
}


/*
 * @brief Sets the data in the app3 field of the descriptor
 * @param[out] bdinstptr - handle of desecriptor;
 * @param[in]  data3     - data for field APP3;
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_app3(bd_t *bdinstptr, uint32_t data3) {
	bdinstptr->app_3 = data3;
}


/*
 * @brief Sets the data in the app4 field of the descriptor
 * @param[out] bdinstptr - handle of desecriptor;
 * @param[in]  data4     - data for field APP4;
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_app4(bd_t *bdinstptr, uint32_t data4) {
	bdinstptr->app_4 = data4;
}


/**
 * @brief Returns the value of register app0 of the current descriptor
 * @param[out] bdinstptr - handle of the current descriptor;
 *
 * @return the value of the app0 field
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_get_app0(bd_t *bdinstptr) {
	return (uint32_t)(bdinstptr->app_0);
}


/**
 * @brief Returns the value of register app1 of the current descriptor
 * @param[out] bdinstptr - handle of the current descriptor;
 *
 * @return the value of the app1 field
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_get_app1(bd_t *bdinstptr) {
	return (uint32_t)(bdinstptr->app_1);

}


/**
 * @brief Returns the value of register app2 of the current descriptor
 * @param[out] bdinstptr - handle of the current descriptor;
 *
 * @return the value of the app2 field
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_get_app2(bd_t *bdinstptr) {
	return (uint32_t)(bdinstptr->app_2);

}


/**
 * @brief Returns the value of register app3 of the current descriptor
 * @param[out] bdinstptr - handle of the current descriptor;
 *
 * @return the value of the app3 field
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_get_app3(bd_t *bdinstptr) {
	return (uint32_t)(bdinstptr->app_3);

}


/**
 * @brief Returns the value of register app4 of the current descriptor
 * @param[out] bdinstptr - handle of the current descriptor;
 *
 * @return the value of the app4 field
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_get_app4(bd_t *bdinstptr) {
	return (uint32_t)(bdinstptr->app_4);
}


/**
 * @brief Sets the virtual address of the initial descriptor in the app0 field
 *   of the current handle.
 * @param[in] bdinstptr - handle of the current descriptor;
 * @param[in] headmem   - value of the address of the first descriptor in the chain
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_headmem(bd_t *bdinstptr, uint32_t headmem) {
	bd_set_app0(bdinstptr, headmem);
}


/**
 * @brief sets the value of the virtual address of the last descriptor in the
 *   app1 field of the current descriptor.
 * @param[in] bdinstptr - handle of the current descriptor
 * @param[in] tailmem   - the value of the address of the last descriptor in the
 *   chain
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_tailmem(bd_t *bdinstptr, uint32_t tailmem) {
	bd_set_app1(bdinstptr, tailmem);
}


/**
 * @brief Setting the alignment value in the app2 field of the current descriptor
 * @param[in] bdinstptr        - pointer to the current descriptor
 * @param[in] alignment_factor - value of the alingment
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_alignmem(bd_t *bdinstptr, uint32_t alignment_factor) {
	bd_set_app2(bdinstptr, alignment_factor);
}


/**
 * @brief Setting the virtual address value to the current descriptor in the app3
 *  field
 * @param[in] bdinstptr - pointer to the current descriptor
 * @param[in] prev_addr - value of the virtual address of the previous descriptor
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_prevbdmem(bd_t *bdinstptr, uint32_t prev_addr){
	bd_set_app3(bdinstptr, prev_addr);
}


/*
 * @brief Getting the headmem value from the current descriptor
 * @param[in] bdinstptr - pointer to the current descriptor
 *
 * @return the pointer to the first descriptor in the chain
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t bd_get_headmem(bd_t *bdinstptr) {
	return bd_get_app0(bdinstptr);
}


/**
 * @brief Getting tailmem value from current descriptor
 * @param[in] bdinstptr - pointer to the current descriptor
 *
 * @return the pointer to the last descriptor in the chain
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_get_tailmem(bd_t *bdinstptr) {
	return bd_get_app1(bdinstptr);
}


/**
 * @brief Obtaining the value of the placement of the descriptor (alignment)
 *   from the current descriptor
 * @param[in] bdinstptr - pointer to the current descriptor
 *
 * @return value of the alingment
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_get_alignmem(bd_t *bdinstptr) {
	return bd_get_app2(bdinstptr);
}


/**
 * @brief Getting a pointer to the previous descriptor
 * @param[in] bdinstptr - pointer to the current descriptor
 *
 * @return value of the address of the previous descriptor
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_get_prevbdmem(bd_t *bdinstptr) {
	return bd_get_app3(bdinstptr);
}


/**
 * @brief Setting the nextdesc field in the current descriptor
 * @param[in] bdinstptr    - pointer to the current descriptor
 * @param[in] nextdescaddr - the physical address of the next descriptor in the
 *   chain.
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_nextdesc(bd_t *bdinstptr, uint32_t nextdescaddr) {
	bdinstptr->nextdesc = nextdescaddr;
}


/**
 * @brief Setting the value of the data buffer address in the bufaddr field in
 *   the current descriptor
 * @param[in] bdinstptr   - pointer to the current descriptor
 * @param[in] buffer_addr - the address of the data buffer
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_bufferaddr(bd_t *bdinstptr, uint32_t buffer_addr) {
	bdinstptr->buffer_addr = buffer_addr;
}


/**
 * @brief Setting a control register in the current descriptor
 * @param[in] bdinstptr  - pointer to the current descriptor
 * @param[in] controlreg - value of the control register
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
void bd_set_control(bd_t *bdinstptr, uint32_t controlreg) {
	bdinstptr->control = controlreg;
}


/*
 * @brief Setting the status register in the current descriptor
 * @param[in] bdinstptr - pointer to the current descriptor
 * @param[in] statusreg - value of the status register
 *
 * @return none
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
void bd_set_status(bd_t *bdinstptr, uint32_t statusreg) {
	bdinstptr->status = statusreg;
}


/*
 * @brief Return the value of the control register from the current descriptor
 * @param[out] bdinstptr - pointer to the current descriptor
 *
 * @return value of the control register
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t bd_get_control(bd_t *bdinstptr) {
	return bdinstptr->control;
}


/**
 * @brief Return the value of the status register of the current handle
 * @param[out] bdinstptr - pointer to the current descriptor
 *
 * @return value of the status register
 *
 * @date 11.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_get_status(bd_t *bdinstptr) {
	return bdinstptr->status;
}


/**
 * @brief Setting the length in the current descriptor
 * @param[in] bdinstptr - pointer to the current descriptor
 * @param[in] length    - value of the length
 *
 * @return RING_OK           - if the value is set
 * 		   ERR_RING_BDMAXLEN - if an incorrect length value is passed
 * @note The function checks that the set value was not 0, was not less than 0,
 *   and was not greater than (2 ^ 23) -1, because based on the AXI documentation,
 *   the DMA can not work with a buffer larger than 8 MByte
 *
 * @date 10.04.2017
 * @author Chemodanov Maxim
 */
uint32_t bd_set_length(bd_t *bdinstptr, size_t length) {
	if ((length > BD_MAX_LEN) || (length <= 0))
		return ERR_RING_BDMAXLEN;

	uint32_t control_reg = bd_get_control(bdinstptr) & ~BD_MAX_LEN;

	control_reg |= length;
	bd_set_control(bdinstptr, control_reg);

	return RING_OK;
}


/**
 * @brief Get address of the next descriptor
 * @param[out] bdinstptr - pointer to the current descriptor
 *
 * @return the address of the next descriptor
 *
 * @author Chemodanov Maxim
 */
uint32_t bd_get_nextdesc(bd_t *bdinstptr) {
	return bdinstptr->nextdesc;
}


/**
 * @brief Get address of the buffer
 * @param[out] bdinstptr - pointer to the current descriptor
 *
 * @return the address of the buffer
 *
 * @author Chemodanov Maxim
 */
uint32_t bd_get_bufferaddr(bd_t *bdinstptr) {
	return bdinstptr->buffer_addr;
}


/*
 * @brief The function returns the length value from the control register.
 * @param[in] bdinstptr - pointer to the current descriptor
 *
 * @return value of the length from the control register
 * @note This is not the length that is obtained when the descriptor is used up.
 *
 * @note
 * @date 11.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t bd_get_length(bd_t *bdinstptr) {
	return bd_get_control(bdinstptr) & BD_MAX_LEN;
}


/**
 * @brief Set SOF in control register of the current descriptor
 * @param[out] bdinstptr - pointer to the current descriptor
 *
 * @return none
 *
 * @author Chemodanov Maxim
 */
void bd_set_sof(bd_t *bdinstptr) {
	bdinstptr->control |= BD_STATUS_SOF;
}


/**
 * @brief Set EOF in control register of the current descriptor
 * @param[out] bdinstptr - pointer to the current descriptor
 *
 * @return none
 *
 * @author Chemodanov Maxim
 */
void bd_set_eof(bd_t *bdinstptr) {
	bdinstptr->control |= BD_STATUS_EOF;
}


uint32_t bd_get_transferred_len(bd_t *bdinstptr) {
	return (bd_get_status(bdinstptr) & BD_MAX_LEN);
}


uint32_t bd_isCompleted(bd_t *bdinstptr) {
	return (bd_get_status(bdinstptr) & BD_STATUS_COMPLETE) ? 1 : 0;
}


uint32_t bd_isSof(bd_t *bdinstptr) {
	return (bdinstptr->status & BD_STATUS_SOF) ? 1 : 0;
}


uint32_t bd_isEof(bd_t *bdinstptr) {
	return (bdinstptr->status & BD_STATUS_EOF) ? 1 : 0;
}


/**
 * @brief Return true if current descriptor in the middle of the chain
 */
uint32_t bd_isIof(bd_t *bdinstptr) {
	return ((bd_isCompleted(bdinstptr)) && (!bd_isSof(bdinstptr))
			                            && (!bd_isEof(bdinstptr))) ? 1 : 0;
}


/**/
uint32_t bd_isSEof(bd_t *bdinstptr) {
	return  ((bd_isSof(bdinstptr)) && (bd_isEof(bdinstptr))) ? 1 : 0;
}


uint32_t bd_free(bd_t *bdinstptr, int channel_factor) {
	//channel_factor :: 1 - receive, 0 - transfer
	if (channel_factor == 0){
		bdinstptr->control = 0;
		bdinstptr->status = 0;
	}
	else if (channel_factor == 1){
		bdinstptr->status = 0;
	}
	return RING_OK;
}


int bdring_get_processed_bds(bdring_t *ringinstptr) {
	int processed_bds = 0;

	bd_t *current_bd = (bd_t*)bdring_get_head(ringinstptr);
	while(bd_isCompleted(current_bd)){
		processed_bds ++;
		current_bd = (bd_t*) bdring_next_bd(current_bd);
	}

	return processed_bds;
}


int bdring_extract_packet(bdring_t *ringinstptr, packet_buffer * pktbufinstptr,
		                  int processed_bds){

	bd_t* current_bd = (bd_t*)bdring_get_head(ringinstptr);
	packet temp_packet;
	int packet_count = 0;
	int bd_index = 0;
	/*
	 * Analysis of four situations :
	 * 1) start - remember the base address of the buffer and set the size of
	 *      the received bytes in the packet;
	 * 2) middle - increment the number of bytes in the current package;
	 * 3) end - but we used several descriptors: increment the number of bytes in
	 *      the packet
	 * 4) end of the packet - we put the resulting packet structure into a packet
	 *      buffer
	 */
	for (bd_index = 0; bd_index < processed_bds; bd_index++) {
		if (bd_isSof(current_bd)) {
			temp_packet.buffer_address = bd_get_bufferaddr(current_bd);
			temp_packet.size = bd_get_transferred_len(current_bd);
			temp_packet.c_flaq = 0;
		}
		else if (bd_isIof(current_bd)) {
			temp_packet.size += bd_get_bufferaddr(current_bd);
		}
		else if ((bd_isEof(current_bd)) && (!bd_isSof(current_bd))) {
			temp_packet.size += bd_get_bufferaddr(current_bd);

		}
		else if (bd_isEof(current_bd)) {
			packet_count++;
			temp_packet.c_flaq = 1;
			packet_buffer_push(pktbufinstptr, temp_packet.buffer_address,
					           temp_packet.size);
		}

		current_bd = (bd_t*)bdring_next_bd(current_bd);
	}

	return packet_count;
}


void bdring_free_bds(bdring_t *ringinstptr, int bds_count) {
	bd_t *current_bd = (bd_t*) bdring_get_head(ringinstptr);
	int bd_index;
	for (bd_index = 0; bd_index < bds_count; bd_index++) {
		bd_free(current_bd, bdring_isRx(ringinstptr));
		current_bd = (bd_t*) bdring_next_bd(current_bd);
	}
}


int bdring_isRx	(bdring_t *ringinstptr) {
	return ringinstptr->isRx;
}


void DebugBD(bd_t *current_bd) {
	printf("next_desc      : 0x%08x\r\n", current_bd->nextdesc);
	printf("buffer_address : 0x%08x\r\n", current_bd->buffer_addr);
	printf("control        : 0x%08x\r\n", current_bd->control);
	printf("status         : 0x%08x\r\n", current_bd->status);
	printf("\r\n");

}
