#ifndef AXIDMA_H
#define AXIDMA_H

#include <stdint.h>			//for using uint32_t
#include <sys/mman.h>		//
#include <fcntl.h>			//for file system function open()
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "axidma_bdring.h"
#include "axidma_mem.h"

#define RX_BASEADDR				0x01000000
#define RX_BUFFER_BASE			0x02000000
#define RX_BUFFER_SIZE			3145728 //131072
#define RX_DELAY				255
#define RX_THRESHOLD			255

#define TX_BASEADDR				0x03000000
#define TX_BUFFER_BASE			0x04000000
#define TX_BUFFER_SIZE			3145728 // 131072
#define TX_DELAY				255
#define TX_THRESHOLD			255

#define RX_CHANNEL				1
#define TX_CHANNEL				0

/**
 * Hardware part of AXI DMA
 */
typedef struct dma_device {
	uint32_t mm2s_dmacr;					//0x00
	uint32_t mm2s_dmasr;					//0x04
	uint32_t mm2s_curdesc;					//0x08
	uint32_t mm2s_curdesc_msb;				//0x0C
	uint32_t mm2s_taildesc;					//0x10
	uint32_t mm2s_taildesc_msb;				//0x14
	uint32_t reserved_1;					//0x18 - in sg not using
	uint32_t reserved_2;					//0x1C - in sg not using
	uint32_t reserved_3; 					//0x20 - in sg not using
	uint32_t reserved_4;					//0x24 - in sg not using
	uint32_t reserved_5; 					//0x28 - in sg not using
	uint32_t sg_ctl;						//0x2C
	uint32_t s2mm_dmacr;					//0x30
	uint32_t s2mm_dmasr;					//0x34
	uint32_t s2mm_curdesc;					//0x38
	uint32_t s2mm_curdesc_msb;				//0x3C
	uint32_t s2mm_taildesc;					//0x40
	uint32_t s2mm_taildesc_msb;				//0x44
} dma_device_t;



/**
 * Software part of AXI DMA
 * Uninitialized dma means no connection between software and hardware parts
 */
typedef struct axidma {
	int 		 dma_file_descriptor; // file's descriptor for allocate memory for axi dma
	int 		 status_rx;			 // RX channel - data from soft structure of DMA
	int 		 status_tx;			 // TX channel
	int 		 isRx;				 // Is exist RX channel
	int 		 isTx;				 // Is exist TX channel
	int 		 Initialized;		 // Initialization state - whether memory is allocated, whether the physical address of the dma
	uint32_t 	 baseaddr_hw;		 // Base address of AXI DMA
	dma_device_t *dma_hw;			 // Pointer to hardware AXI DMA
	bdring_t	 rxring;				 // Descriptor chain to receive data
	bdring_t	 txring;				 // descriptor chain to send data
} axidma_t;


/***************************CONTROLREG HW FIELD MASK***************************/
/*****MASK*****/
#define DMACR_RS_MASK				0x00000001 		//bit 0
#define DMACR_RESET_MASK			0x00000004		//bit 2
#define DMACR_KEYHOLE_MASK			0x00000008		//bit 3
#define DMACR_CYCLICBD_MASK			0x00000010		//bit 4
#define DMACR_IOCIRQ_MASK			0x00001000		//bit 12
#define DMACR_DLYIRQ_MASK			0x00002000		//bit 13
#define DMACR_ERRIRQ_MASK			0x00004000		//bit 14
#define DMACR_ALLIRQ_MASK			0x00007000		//0x0000[0111]000
#define DMACR_THRESHOLD_MASK		0x00FF0000		//0x00[1111][1111]0000
#define DMACR_DELAY_MASK			0xFF000000		//0x[1111][1111]000000
/*****FIELD VALUES MACROS*****/
#define CR_RUN						1
#define CR_STOP						0
#define THRESHOLD_SHIFT				16
#define DELAY_SHIFT					24
#define IRQ_SOFT_OFFSET				12
#define IRQ_SOFT_MASK 				0x00000007

/******************************************************************************/


/***************************STATUS REG HW FIELD MASK***************************/
#define DMASR_HALT_MASK				0x00000001
#define DMASR_IDLE_MASK				0x00000002
#define DMASR_ISSG_MASK				0x00000008
#define DMASR_DMAINTERR_MASK		0x00000010
#define DMASR_DMASLVERR_MASK 		0x00000020
#define DMASR_DMADECERR_MASK		0x00000040
#define DMASR_DMAERR_ALL_MASK 		0x00000070
#define DMASR_SGINTERR_MASK			0x00000100
#define DMASR_SGSLVERR_MASK			0x00000200
#define DMASR_SGDECERR_MASK			0x00000400
#define DMASR_SGERR_ALL_MASK 		0x00000700
#define DMASR_IOCIRQ_MASK 			0x00001000
#define DMASR_DLYIRQ_MASK 			0x00002000
#define DMASR_ERRIRQ_MASK 			0x00004000
#define DMASR_IRQALL_MASK 			0x00007000
#define DMASR_THRESHOLD_MASK 		0x00FF0000
#define DMASR_DELAY_MASK			0xFF000000
/******************************************************************************/
#define RESET_TIMEOUT 				300000// 100000

/*****Return values of AXI DMA functions*****/
#define DMA_OK						0
#define ERR_DMA_OPEN_FD				1	//ошибка открытия файлового дескриптора
#define ERR_DMA_DEV_BAD_ADDRESS 	2 	//ошибка указания адреса аппаратного AXI DMA
#define ERR_DMA_DEV_MAP				3	//ошибка mmap
#define ERR_DMA_UNINIT				4	//если программная структура не была инициализирована
#define ERR_DMA_BAD_DELAY			5	//если передан неверный параметр delay
#define ERR_DMA_BAD_THRESHOLD		6	//если передан неверный параметр threshold
#define ERR_DMA_HAD_WORK			7	//если дма в текущий момент работает
#define ERR_DMA_RESET_RX			21	//если сброс axi dma завершен с ошибкой на канале RX
#define ERR_DMA_RESET_TX			22	//если сброс axi dma завершен с ошибкой на канале TX

/****************************DMA Macros Definitions****************************/
/**
 * @brief Is the S2MM channel represented in the hardware AXI DMA
 */
#define axidma_hasrx(axidma)	\
	((axidma)->dma_hw->s2mm_dmacr) ? 1 : 0


/**
 * @brief Is the MM2S channel represented in the hardware AXI DMA
 */
#define axidma_hastx(axidma)	\
	((axidma)->dma_hw->mm2s_dmacr) ? 1 : 0


/**************************DMA Functions Definitions***************************/
int 		axidma_soft_set_tx_threshold (axidma_t *dmainstptr, int thres);
int 		axidma_soft_set_rx_threshold (axidma_t *dmainstptr, int thres);
int 		axidma_soft_set_tx_delay	 (axidma_t *dmainstptr, int delay);
int 		axidma_soft_set_rx_delay	 (axidma_t *dmainstptr, int delay);
int 		axidma_soft_set_tx_irq		 (axidma_t *dmainstptr, int irq_vector);
int 		axidma_soft_set_rx_irq		 (axidma_t *dmainstptr, int irq_vector);
int 		axidma_soft_get_tx_threshold (axidma_t *dmainstptr);
int 		axidma_soft_get_rx_threshold (axidma_t *dmainstptr);
int 		axidma_soft_get_tx_delay	 (axidma_t *dmainstptr);
int 		axidma_soft_get_rx_delay	 (axidma_t *dmainstptr);
int 		axidma_soft_get_tx_irq		 (axidma_t *dmainstptr);
int 		axidma_soft_get_rx_irq		 (axidma_t *dmainstptr);

int 		axidma_initialization		 (axidma_t *dmainstptr, uint32_t axidma_address);
int 		axidma_initialized			 (axidma_t *dmainstptr);
int			axidma_is_sg				 (axidma_t *dmainstptr);

/*****Reading registers of AXI DMA*****/
uint32_t 	axidma_get_tx_status	 	 (axidma_t *dmainstptr);
uint32_t 	axidma_get_rx_status		 (axidma_t *dmainstptr);
uint32_t 	axidma_get_tx_control		 (axidma_t *dmainstptr);
uint32_t 	axidma_get_rx_control		 (axidma_t *dmainstptr);
uint32_t 	axidma_get_tx_curdesc		 (axidma_t *dmainstptr);
uint32_t 	axidma_get_rx_curdesc		 (axidma_t *dmainstptr);
uint32_t 	axidma_get_tx_taildesc		 (axidma_t *dmainstptr);
uint32_t 	axidma_get_rx_taildesc		 (axidma_t *dmainstptr);

/*****Writing registers of AXI DMA*****/
uint32_t 	axidma_set_tx_status		 (axidma_t *dmainstptr, uint32_t status_reg);
uint32_t 	axidma_set_rx_status		 (axidma_t *dmainstptr, uint32_t status_reg);
uint32_t 	axidma_set_tx_control		 (axidma_t *dmainstptr, uint32_t control_reg);
uint32_t 	axidma_set_rx_control		 (axidma_t *dmainstptr, uint32_t control_reg);
void     	axidma_set_tx_curdesc		 (axidma_t *dmainstptr, uint32_t curdesc_reg);
void    	axidma_set_rx_curdesc		 (axidma_t *dmainstptr, uint32_t curdesc_reg);
void    	axidma_set_tx_taildesc		 (axidma_t *dmainstptr, uint32_t taildesc_reg);
void    	axidma_set_rx_taildesc		 (axidma_t *dmainstptr, uint32_t taildesc_reg);

/*****функции запуска/останова AXI DMA*****/
uint32_t 	axidma_tx_start				 (axidma_t *dmainstptr);
uint32_t 	axidma_rx_start				 (axidma_t *dmainstptr);
uint32_t 	axidma_api_start			 (axidma_t *dmainstptr);
uint32_t	axidma_tx_pause				 (axidma_t *dmainstptr);
uint32_t	axidma_rx_pause				 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_iswork			 (axidma_t *dmainstptr);
uint32_t 	axidma_rx_iswork			 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_pauseisdone		 (axidma_t *dmainstptr);
uint32_t 	axidma_rx_pauseisdone		 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_resume			 (axidma_t *dmainstptr);
uint32_t 	axidma_rx_resume			 (axidma_t *dmainstptr);
uint32_t 	axidma_api_reset			 (axidma_t *dmainstptr);
uint32_t	axidma_tx_reset				 (axidma_t *dmainstptr);
uint32_t	axidma_rx_reset				 (axidma_t *dmainstptr);
uint32_t	axidma_tx_resetisdone		 (axidma_t *dmainstptr);
uint32_t	axidma_rx_resetisdone		 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_busy				 (axidma_t *dmainstptr);
uint32_t	axidma_rx_busy				 (axidma_t *dmainstptr);

/*****Режимы работы AXI DMA*****/
uint32_t  	axidma_tx_setKeyholeOn		 (axidma_t *dmainstptr);
uint32_t  	axidma_rx_setKeyholeOn		 (axidma_t *dmainstptr);
uint32_t  	axidma_tx_setKeyholeOff		 (axidma_t *dmainstptr);
uint32_t  	axidma_rx_setKeyholeOff		 (axidma_t *dmainstptr);
uint32_t  	axidma_tx_isKeyholeMode		 (axidma_t *dmainstptr);
uint32_t  	axidma_rx_isKeyholeMode		 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_setDelay			 (axidma_t *dmainstptr, uint32_t delay_factor);
uint32_t 	axidma_rx_setDelay			 (axidma_t *dmainstptr, uint32_t delay_factor);
uint32_t 	axidma_tx_setThreshold		 (axidma_t *dmainstptr, uint32_t threshold_factor);
uint32_t 	axidma_rx_setThreshold		 (axidma_t *dmainstptr, uint32_t threshold_factor);
uint32_t 	axidma_tx_getDelay			 (axidma_t *dmainstptr, uint32_t delay_factor);
uint32_t 	axidma_rx_getDelay			 (axidma_t *dmainstptr, uint32_t delay_factor);
uint32_t 	axidma_tx_getThreshold		 (axidma_t *dmainstptr, uint32_t threshold_factor);
uint32_t 	axidma_rx_getThreshold		 (axidma_t *dmainstptr, uint32_t threshold_factor);

/*****Interrupt of AXI DMA*****/
uint32_t 	axidma_tx_set_ioc_irq		 (axidma_t *dmainstptr);
uint32_t 	axidma_rx_set_ioc_irq		 (axidma_t *dmainstptr);
uint32_t	axidma_tx_set_dly_irq		 (axidma_t *dmainstptr);
uint32_t	axidma_rx_set_dly_irq		 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_set_err_irq	 	 (axidma_t *dmainstptr);
uint32_t 	axidma_rx_set_err_irq	  	 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_enable_all_irq	 (axidma_t *dmainstptr);
uint32_t 	axidma_rx_enable_all_irq	 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_disable_all_irq 	 (axidma_t *dmainstptr);
uint32_t 	axidma_rx_disable_all_irq	 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_get_irq			 (axidma_t *dmainstptr);
uint32_t 	axidma_rx_get_irq			 (axidma_t *dmainstptr);
uint32_t 	axidma_tx_ack_irq			 (axidma_t *dmainstptr, uint32_t irq);
uint32_t 	axidma_rx_ack_irq			 (axidma_t *dmainstptr, uint32_t irq);


/*****Work with BdRing*****/
bdring_t	*axidma_get_rxring	      	(axidma_t *dmainstptr);
bdring_t	*axidma_get_txring		    (axidma_t *dmainstptr);


uint32_t axidma_api_run       (axidma_t *dmainstptr, uint32_t axidma_hw_address);
uint32_t axidma_api_hasrx     (axidma_t *dmainstptr);
uint32_t axidma_api_hastx     (axidma_t *dmainstptr);
uint32_t axidma_api_send      (axidma_t *dmainstptr, memory_buffer_t *membufinstptr,
		                       uint32_t size);
uint32_t axidma_api_receive   (axidma_t *dmainstptr, memory_buffer_t *membufinstptr);
uint32_t axidma_api_process_tx(axidma_t *dmainstptr);
uint32_t axidma_api_process_rx(axidma_t *dmainstptr);
uint32_t axidma_api_rx_reload (axidma_t *dmainstptr);


#endif // AXIDMA_H
