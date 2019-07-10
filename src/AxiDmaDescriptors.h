//
// Created by vldmr on 25.06.19.
//

#ifndef AXIDMA_API_AXIDMADESCRIPTOR_H
#define AXIDMA_API_AXIDMADESCRIPTOR_H


#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdexcept>
#include <unistd.h>


/**
 * @class Used for working with AXI DMA descriptors
 */
class AxiDmaDescriptors {
public:
    enum error_code {
        RING_OK     = 0,
        ERR_OPEN_FD = 16,
        ERR_UNINIT  = 17,
        ERR_BAD_BUFFERLEN,
        ERR_SIZE,
        ERR_ALIGN,
        ERR_BPD,
        ERR_BDCNT,
        ERR_LASTADDR,
        ERR_MAP,
        ERR_BDMAX_LEN
    };

    explicit AxiDmaDescriptors(bool chan);

    void SetBytesPerDesc    (uint32_t value);
    int  GetCountDescriptors();

    uint32_t GetHeadDescriptorAddr();
    uint32_t GetTailDescriptorAddr();
    uint32_t GetBufferBaseAddr    ();

    int      InitDescriptors   (uint32_t buffer_addr, size_t buffer_size);
    size_t   ProcessDescriptors(bool soft);

    uint32_t GetStatus();
    bool     IsRx();

    virtual ~AxiDmaDescriptors();
    void DebugDescs(); // for debug
private:
    /***** Structure of the descriptor *****/
    typedef struct descriptor {
        uint32_t nextdesc;          // 0x00
        uint32_t nextdesc_msb;      // 0x04
        uint32_t buffer_addr;       // 0x08
        uint32_t buffer_addr_msb;   // 0x0C
        uint32_t reserved_1;        // 0x10
        uint32_t reserved_2;        // 0x14
        uint32_t control;           // 0x18
        uint32_t status;            // 0x1C
        uint32_t app_0;			    // 0x20, will be contains address of firstmem
        uint32_t app_1;			    // 0x24, will be contains address of tailmem
        uint32_t app_2;			    // 0x28, will be contains separation_factor
        uint32_t app_3;			    // 0x2C, will be contains prev bd
        uint32_t app_4;             // 0x30
    } descr_t;

    // BD - bound descriptor
    static constexpr uint32_t BD_MIN_ALIGNMENT = 0x40;
    static constexpr uint32_t BD_MAX_LEN       = 0x007FFFFF; // 8 MByte
    static constexpr uint32_t BD_STATUS_COMP   = 0x80000000; // Complete
    static constexpr uint32_t BD_STATUS_SOF    = 0x08000000; // Start
    static constexpr uint32_t BD_STATUS_EOF    = 0x04000000; // End
    static constexpr uint32_t BD_OPTIMAL_SIZE  = 3670016;  /*7340032*/ /*8192*/

    static constexpr uint32_t RX_BASEADDR	 = 0x01000000;
    static constexpr uint32_t RX_BUFFER_BASE = 0x02000000;
    static constexpr uint32_t TX_BASEADDR	 = 0x03000000;
    static constexpr uint32_t TX_BUFFER_BASE = 0x04000000;

    int         fd { 0 };
    size_t      remainder_size { 0 };    // Size of remainder buffer
    uint32_t    bytes_per_desc { BD_OPTIMAL_SIZE }; // Buffer size per descriptor

    uint32_t 	chain_phys_baseaddr;   // Hardware address of DDR
    uint32_t	buffer_phys_baseaddr;  // The starting address of the data buffer
    uint32_t 	*chain_virt_baseaddr;  // Virtual address of the beginning of
                                       // the segment of the chain of descriptors
    uint32_t 	_chain_size { 0 };     // Memory size for storing descriptors
    int 		bd_count    { 0 };
    bool 		isRx        { false }; // Channel of reception or transmission?


    int  prepareChain(size_t buffer_size);
    int  calcDescrCount(size_t bytes_per_descr, size_t buffer_size);
    int  setDescrCount(int descriptors_count);
    int  setChainSize(size_t chain_size);
    int  allocDescriptorMem();
    void clearMemory();

    void    *getHeadOfDescriptors();
    uint32_t getNextAddress(uint32_t current_address);
    void    *getNextAddress(void *current_descriptor);

    uint32_t getTailMem(std::uintptr_t headmem_addr, int count_descr);
    uint32_t getControl(descr_t *curr_descriptor);
    uint32_t getStatus (descr_t *curr_descriptor);

    void setHeadMem      (descr_t *curr_descriptor, uint32_t headmem);
    void setTailMem      (descr_t *curr_descriptor, uint32_t tailmem);
    void setAlignMem     (descr_t *curr_descriptor, uint32_t alignmem);
    void setNextDescrAddr(descr_t *curr_descriptor, uint32_t next_address);
    void setBufferAddr   (descr_t *curr_descriptor, uint32_t buff_address);
    void setControl      (descr_t *curr_descriptor, uint32_t control_reg);
    void setStatus       (descr_t *curr_descriptor, uint32_t status_reg);
    int  setLength       (descr_t *curr_descriptor, size_t length);

    void setApp0(descr_t *curr_descr, uint32_t data);
    void setApp1(descr_t *curr_descr, uint32_t data);
    void setApp2(descr_t *curr_descr, uint32_t data);

    void setSof(descr_t *curr_descr);
    void setEof(descr_t *curr_descr);

    /** STATUS_REGISTER **/
    uint32_t getTransferredLen(descr_t *curr_descr);
    bool isCompleted(descr_t *curr_descr);
    bool isSof(descr_t *curr_descr);
    bool isIof(descr_t *curr_descr);
    bool isEof(descr_t *curr_descr);

    void freeDescriptor    (descr_t *curr_descr);
    int countProcessedDescs();
};


#endif //AXIDMA_API_AXIDMADESCRIPTOR_H
