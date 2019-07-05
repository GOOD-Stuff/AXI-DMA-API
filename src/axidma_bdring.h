#ifndef AXIDMA_BDRING_H
#define AXIDMA_BDRING_H

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "axidma_packet.h"


/***************************** MACROS DEFINITIONS *****************************/
/**** ERROR CODES ****/
#define RING_OK						0
#define ERR_RING_OPEN_FD			8
#define ERR_RING_UNINIT				9
#define ERR_RING_BAD_BUFFERLEN		10
#define ERR_BDRING_SIZE				11
#define ERR_RING_ALIGN				12
#define ERR_BPD						13
#define ERR_BDCNT					14
#define ERR_RING_LASTADDR			15
#define ERR_RING_MAP				16
#define ERR_RING_BDMAXLEN			17

/***** Defaults *****/
#define BYTES_PER_DESC				8192 // 4096
#define	BD_MIN_ALIGNMENT			0x40
#define INIT_RXCHAN_CFG				0
#define INIT_TXCHAN_CFG				1
#define BDRING_UNINIT_STATE			0

/***** MAXIMAL LENGTH FOR ONE BD *****/
#define BD_MAX_LEN					0x007FFFFF // 8 MByte
/***** ANALYSIS DESCRIPTOR *****/
#define BD_STATUS_COMPLETE			0x80000000
#define BD_STATUS_SOF				0x08000000
#define BD_STATUS_EOF				0x04000000


/***** Structure of the chain of descriptors ****/
typedef struct bdring {
	int 		file_descriptor;	   // File descriptor (/dev/mem)
	int 		initialized;		   // Initialization state
	int 		bdring_size;		   // Memory size for storing descriptors
	uint32_t 	bdring_phys_baseaddr;  // Hardware address of DDR
	uint32_t 	bdring_phys_lastaddr;  // End of address range
	uint32_t 	bd_headaddr;		   // Address of the first descriptor (physical)
	uint32_t	bd_lastaddr;		   // Address of the last descriptor (physical)
	int 		buffer_size;		   // Size of buffer
	uint32_t	buffer_phys_baseaddr;  // The starting address of the data buffer
	uint32_t 	buffer_phys_lastaddr;  // The final address of the data buffer
	int  		bd_alignment;		   // The placement of one descriptor must be a multiple of 0x40
	uint32_t 	*bdring_virt_baseaddr; // Virtual address of the beginning of the segment of the chain of descriptors
	uint32_t 	buffer_per_bd;		   // Buffer size per descriptor
	int 		bd_count;
	uint32_t 	firstdesc_addr;
	uint32_t 	taildesc_addr;
	int 		isRx;				   // Channel of reception or transmission?
} bdring_t;

/***** Structure of the buffer descriptor *****/
typedef struct bd {
	uint32_t nextdesc;
	uint32_t nextdesc_msb;
	uint32_t buffer_addr;
	uint32_t buffer_addr_msb;
	uint32_t reserved_1;
	uint32_t reserved_2;
	uint32_t control;
	uint32_t status;
	uint32_t app_0;					// will be contains address of firstmem
	uint32_t app_1;					// will be contains address of tailmem
	uint32_t app_2;					// will be contains separation_factor
	uint32_t app_3;					// will be contains prev bd
	uint32_t app_4;
} bd_t;


/*********************** FUNCTIONS DEFINITIONS FOR BDRING ********************/
uint32_t bdring_initialize			(bdring_t *ringinstptr, uint32_t bdbase,
		                             uint32_t buffer_base, uint32_t buffer_size,
									 int init_mode);
void     bdring_set_init_state		(bdring_t *ringinstptr);
void     bdring_set_uninit_state	(bdring_t *ringinstptr);
uint32_t bdring_get_state			(bdring_t *ringinstptr);

uint32_t bdring_set_buffer_size		(bdring_t *ringinstptr, int buffer_size);
uint32_t bdring_set_bdring_size		(bdring_t *ringinstptr, int bdring_size);
uint32_t bdring_set_bd_align		(bdring_t *ringinstptr, int align);
void     bdring_set_buffer_lastaddr	(bdring_t *ringinstptr, uint32_t buffer_base,
		                             uint32_t buffer_size);
uint32_t bdring_set_buffer_per_bd	(bdring_t *ringinstptr, uint32_t buffer_per_bd_size);
uint32_t bdring_set_bdcount	 		(bdring_t *ringinstptr, int bd_cnt);
uint32_t bdring_set_lastaddr		(bdring_t *ringinstptr, uint32_t bd_space_lastaddr);

uint32_t api_get_bdcnt 				(int buffer_per_bd_size, int buffer_size);
uint32_t api_get_tailmem_addr		(uint32_t headmem_addr, int bd_cnt);
uint32_t bdring_alloc				(bdring_t *ringinstptr);
void     bdring_init_rx_bd			(bdring_t *ringinstptr);
void     bdring_init_tx_bd			(bdring_t *ringinstptr);
void     bdring_clear_mem			(bdring_t *ringinstptr);
uint32_t *bdring_get_head			(bdring_t *ringinstptr);
uint32_t bdring_next_bd			    (bd_t* current_bd);
uint32_t bdring_prev_bd				(bd_t* current_bd);
uint32_t bdring_get_nextaddr		(bdring_t *ringinstptr, uint32_t current_address);


void     bd_set_app0				(bd_t *bdinstptr, uint32_t data0);
void     bd_set_app1				(bd_t *bdinstptr, uint32_t data1);
void     bd_set_app2				(bd_t *bdinstptr, uint32_t data2);
void     bd_set_app3				(bd_t *bdinstptr, uint32_t data3);
void     bd_set_app4				(bd_t *bdinstptr, uint32_t data4);
uint32_t bd_get_app0				(bd_t *bdinstptr);
uint32_t bd_get_app1				(bd_t *bdinstptr);
uint32_t bd_get_app2				(bd_t *bdinstptr);
uint32_t bd_get_app3				(bd_t *bdinstptr);
uint32_t bd_get_app4				(bd_t *bdinstptr);

void     bd_set_headmem				(bd_t *bdinstptr, uint32_t headmem);
void     bd_set_tailmem 			(bd_t *bdinstptr, uint32_t tailmem);
void     bd_set_alignmem			(bd_t *bdinstptr, uint32_t alignment_factor);
void     bd_set_prevbdmem			(bd_t *bdinstptr, uint32_t prev_addr);
uint32_t bd_get_headmem				(bd_t *bdinstptr);
uint32_t bd_get_tailmem 			(bd_t *bdinstptr);
uint32_t bd_get_alignmem			(bd_t *bdinstptr);
uint32_t bd_get_prevbdmem			(bd_t *bdinstptr);

void     bd_set_nextdesc			(bd_t *bdinstptr, uint32_t nextdescaddr);
void     bd_set_bufferaddr			(bd_t *bdinstptr, uint32_t buffer_addr);
void     bd_set_control				(bd_t *bdinstptr, uint32_t controlreg);
void     bd_set_status			 	(bd_t *bdinstptr, uint32_t statusreg);
uint32_t bd_get_control 			(bd_t *bdinstptr);
uint32_t bd_get_status 				(bd_t *bdinstptr);
uint32_t bd_set_length		 		(bd_t *bdinstptr, size_t length);

uint32_t bd_get_nextdesc			(bd_t *bdinstptr);
uint32_t bd_get_bufferaddr			(bd_t *bdinstptr);
uint32_t bd_get_length		 		(bd_t *bdinstptr);

void     bd_set_sof					(bd_t *bdinstptr);
void     bd_set_eof					(bd_t *bdinstptr);



/** STATUS_REGISTER **/
uint32_t bd_get_transferred_len	 	(bd_t *bdinstptr);
uint32_t bd_isCompleted				(bd_t *bdinstptr);
uint32_t bd_isSof					(bd_t *bdinstptr);
uint32_t bd_isEof					(bd_t *bdinstptr);
uint32_t bd_isIof					(bd_t *bdinstptr);
uint32_t bd_isSEof					(bd_t *bdinstptr);
uint32_t bd_free					(bd_t *bdinstptr, int channel_factor);

/* processing stage */
int bdring_get_processed_bds		(bdring_t *ringinstptr);
int bdring_extract_packet			(bdring_t *ringinstptr, packet_buffer *pktbufinstptr,
		                             int processed_bds);
void bdring_free_bds				(bdring_t *ringinstptr, int bds_count);
int  bdring_isRx					(bdring_t *ringinstptr);


void DebugBD(bd_t *current_bd);

#endif // AXIDMA_BDRING_H
