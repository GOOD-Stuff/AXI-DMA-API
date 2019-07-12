//
// Created by vldmr on 25.06.19.
//

#ifndef AXIDMA_API_AXIDMA_H
#define AXIDMA_API_AXIDMA_H

#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <mutex>
#include "AxiDmaBuffer.h"
#include "AxiDmaDescriptors.h"


/**
 * @class Used for transferring data via AXI DMA from userspace
 */
class AxiDMA {
public:
    enum error_codes {
        DMA_OK = 0,              // 00 - no error
        ERR_DMA_OPEN_FD,         // 01 - can't open /dev/mem
        ERR_DMA_BAD_CHAIN,       // 02 - wrong AXI DMA descriptors chain
        ERR_DMA_DEV_MAP,         // 03 - can't mmap to /dev/mem
        ERR_DMA_UNINIT,          // 04 - can't initialize descriptors
        ERR_DMA_BAD_DELAY,       // 05 - set wrong delay (to mm2s/s2mm control register)
        ERR_DMA_BAD_THRESHOLD,   // 06 - set wrong threshold (to control register)
        ERR_DMA_HALT_WORK,       // 07 - AXI DMA in halt state
        ERR_DMA_ALR_WORK,        // 08 - AXI DMA already work
        ERR_DMA_RESET_RX,        // 09 - can't reset Rx channel
        ERR_DMA_RESET_TX,        // 10 - can't reset Tx channel
        ERR_DMA_TX_IRQ,          // 11 - get error Tx interrupt
        ERR_DMA_RX_IRQ,          // 12 - get error Rx interrupt
        ERR_DMA_TX_TIMEOUT,      // 13 - timeout (when transfer data)
        ERR_DMA_RX_TIMEOUT,      // 14 - timeout (when transfer data)
        ERR_DMA_BAD_ALLOC,       // 15 - can't allocate memory
        ERR_DMA_TX_IDLEOUT,      // 16 - timeout by waiting IDLE state
        ERR_DMA_RX_IDLEOUT,      // 17 - timeout by waiting IDLE state
    };

    explicit AxiDMA();
    explicit AxiDMA(uint32_t dma_base_addr);
    /************* SG part *****************/
    int Send  (const AxiDmaBuffer *buff);
    int Recv  (AxiDmaBuffer *buff, size_t size);
    int Transf(const AxiDmaBuffer *tx, AxiDmaBuffer *rx, size_t rx_len);

    int Send_repeat(const AxiDmaBuffer *buff);

    int  ManualPrepareTransfer(AxiDmaDescriptors *desc_chain, bool way);
    int  ManualStartTransfer  (AxiDmaDescriptors *desc_chain, bool way);
    int  ManualPollIrq        (bool way);
    void ManualResetChannel   (bool way);
    void *ManualAllocMemory   (size_t buffer_size, uint32_t buffer_base_address);
    /******************************************/

    /************** Direct mode ***************/
    int DirectSend(const AxiDmaBuffer *buff);
    int DirectRecv(AxiDmaBuffer *buff, size_t size);
    /******************************************/
    uint32_t GetControl (bool way);
    uint32_t GetStatus  (bool way);
    uint32_t GetCurrDesc(bool way);
    uint32_t GetTailDesc(bool way);

    virtual ~AxiDMA();
private:
    /**
     * Hardware part of AXI DMA, see PG 021
     */
    typedef struct dma_device {
        uint32_t mm2s_dmacr;		// 0x00
        uint32_t mm2s_dmasr;		// 0x04
        uint32_t mm2s_curdesc;		// 0x08
        uint32_t mm2s_curdesc_msb;	// 0x0C
        uint32_t mm2s_taildesc;		// 0x10
        uint32_t mm2s_taildesc_msb;	// 0x14
        uint32_t mm2s_src_addr;		// 0x18 - in sg not using, only for dm
        uint32_t mm2s_src_addr_msb; // 0x1C - in sg not using, only for dm
        uint32_t reserved_0; 		// 0x20 - not using
        uint32_t reserved_1;		// 0x24 - not using
        uint32_t mm2s_length; 		// 0x28 - in sg not using, only for dm
        uint32_t sg_ctl;			// 0x2C - only in multichannel mode
        uint32_t s2mm_dmacr;		// 0x30
        uint32_t s2mm_dmasr;		// 0x34
        uint32_t s2mm_curdesc;		// 0x38
        uint32_t s2mm_curdesc_msb;	// 0x3C
        uint32_t s2mm_taildesc;		// 0x40
        uint32_t s2mm_taildesc_msb;	// 0x44
        uint32_t s2mm_dst_addr;    	// 0x48 - in sg not using, only for dm
        uint32_t s2mm_dst_addr_msb; // 0x4C - in sg not using, only for dm
        uint32_t reserved_2;        // 0x50
        uint32_t reserved_3;        // 0x54
        uint32_t s2mm_length;       // 0x58 - in sg not using, only for dm
    } dma_device_t;

    static constexpr uint32_t AXIDMA_BASEADDR       = 0x80400000;
    static constexpr size_t   DESCRIPTORS_BUFF_SIZE = 3145728;
    static constexpr int      RESET_TIMEOUT         = 300000;
    static constexpr uint32_t UNKNOWN_SIZE          = 0xFFFF;

    // need to rewrite
    static constexpr uint32_t RX_BASEADDR	 = 0x01000000;
    static constexpr uint32_t RX_BUFFER_BASE = 0x02000000;
    static constexpr uint32_t TX_BASEADDR	 = 0x03000000;
    static constexpr uint32_t TX_BUFFER_BASE = 0x04000000;

    /************************* MASK ****************************/
    // Control
    static constexpr uint32_t DMACR_RUN_MASK	   = 0x00000001; //bit 0
    static constexpr uint32_t DMACR_RESET_MASK	   = 0x00000004; //bit 2
    static constexpr uint32_t DMACR_KEYHOLE_MASK   = 0x00000008; //bit 3
    static constexpr uint32_t DMACR_CYCLICBD_MASK  = 0x00000010; //bit 4
    static constexpr uint32_t DMACR_IOCIRQ_MASK	   = 0x00001000; //bit 12
    static constexpr uint32_t DMACR_DLYIRQ_MASK	   = 0x00002000; //bit 13
    static constexpr uint32_t DMACR_ERRIRQ_MASK	   = 0x00004000; //bit 14
    static constexpr uint32_t DMACR_ALLIRQ_MASK	   = 0x00007000; //0x0000[0111]000
    static constexpr uint32_t DMACR_THRESHOLD_MASK = 0x00FF0000; //0x00[1111][1111]0000
    static constexpr uint32_t DMACR_DELAY_MASK	   = 0xFF000000; //0x[1111][1111]000000
    // Status
    static constexpr uint32_t DMASR_HALT_MASK	    = 0x00000001;
    static constexpr uint32_t DMASR_IDLE_MASK		= 0x00000002;
    static constexpr uint32_t DMASR_ISSG_MASK		= 0x00000008;
    static constexpr uint32_t DMASR_DMAINTERR_MASK	= 0x00000010;
    static constexpr uint32_t DMASR_DMASLVERR_MASK 	= 0x00000020;
    static constexpr uint32_t DMASR_DMADECERR_MASK	= 0x00000040;
    static constexpr uint32_t DMASR_DMAERR_ALL_MASK = 0x00000070;
    static constexpr uint32_t DMASR_SGINTERR_MASK	= 0x00000100;
    static constexpr uint32_t DMASR_SGSLVERR_MASK	= 0x00000200;
    static constexpr uint32_t DMASR_SGDECERR_MASK	= 0x00000400;
    static constexpr uint32_t DMASR_SGERR_ALL_MASK 	= 0x00000700;
    static constexpr uint32_t DMASR_IOCIRQ_MASK 	= 0x00001000;
    static constexpr uint32_t DMASR_DLYIRQ_MASK 	= 0x00002000;
    static constexpr uint32_t DMASR_ERRIRQ_MASK 	= 0x00004000;
    static constexpr uint32_t DMASR_IRQALL_MASK 	= 0x00007000;
    static constexpr uint32_t DMASR_THRESHOLD_MASK 	= 0x00FF0000;
    static constexpr uint32_t DMASR_DELAY_MASK		= 0xFF000000;
    /**************************************************************/
    uint32_t _base_addr { AXIDMA_BASEADDR };
    std::mutex mute; // for thread safe (?)
    /**
     * Software part of AXI DMA
     * Uninitialized dma means no connection between software and hardware parts
     */
    int              fd   { 0 };
    int 		     isRx { 0 }; // Is exist RX channel
    int 		     isTx { 0 }; // Is exist TX channel
    dma_device_t     *dma_hw;    // Pointer to hardware AXI DMA

    int  run();
    int  initialization();

    void setTxDefault();
    void setRxDefault();
    int  resetChannels();
    void resetTx();
    void resetRx();
    bool isTxReset();
    bool isRxReset();

    /***** Reading registers of AXI DMA *****/
    uint32_t getTxControl();
    uint32_t getTxStatus();
    uint32_t getRxStatus();
    uint32_t getRxControl();

    /***** Writing registers of AXI DMA *****/
    void setTxControl (uint32_t control_reg);
    void setTxStatus  (uint32_t status_reg);
    void setTxCurDesc (uint32_t curr_desc);
    void setTxTailDesc(uint32_t taildesc);

    void setRxControl (uint32_t control_reg);
    void setRxStatus  (uint32_t status_reg);
    void setRxCurDesc (uint32_t curr_desc);
    void setRxTailDesc(uint32_t tail_desc);

    void setSourceAddress     (uint32_t src_addr);
    void setDestinationAddress(uint32_t dst_addr);
    void setTxLength          (uint32_t length);
    void setRxLength          (uint32_t length);
    uint32_t getTxLength      ();
    uint32_t getRxLength      ();

    void startTx();
    void startRx();

    int  setTxDelay    (uint32_t delay);
    int  setRxDelay    (uint32_t delay);
    int  setTxThreshold(uint32_t threshold);
    int  setRxThreshold(uint32_t threshold);

    bool isTxWork();
    bool isRxWork();

    bool isTxRun();
    bool isRxRun();

    /***** Interrupt of AXI DMA *****/
    void enableAllTxIrq();
    void enableAllRxIrq();
    void disableAllTxIrq();
    void disableAllRxIrq();

    uint32_t getTxIRQ();
    uint32_t getRxIRQ();

    uint32_t ackTxIRQ(uint32_t irq);
    uint32_t ackRxIRQ(uint32_t irq);

    void *allocBufferMem(size_t buff_size, uint32_t buffer_base_address);
    bool checkTxHalt();
    bool checkRxHalt();

    int  waitTxComplete();
    int  waitRxComplete();

    int  waitTxDirectComplete();
    int  waitRxDirectComplete();

    /****** Check existence of Tx/Rx channels *******/
    static bool hasTx(dma_device_t *dma_hw) {
        return dma_hw->mm2s_dmacr != 0;
    }

    static bool hasRx(dma_device_t *dma_hw) {
        return dma_hw->s2mm_dmacr != 0;
    }
};


#endif //AXIDMA_API_AXIDMA_H
