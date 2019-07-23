//
// Created by vldmr on 25.06.19.
//

#include "AxiDMA.h"

/***** FIELD VALUES MACROS *****/
#define THRESHOLD_SHIFT				16
#define DELAY_SHIFT					24


AxiDMA::AxiDMA() {
    if (run() != DMA_OK)
        throw std::runtime_error("Can't initialize AXI DMA");
}


AxiDMA::AxiDMA(uint32_t dma_base_addr) {
    if ((dma_base_addr < 0x40000000) || (dma_base_addr > 0xC0000000))
        throw std::runtime_error("Wrong AXI DMA base address");

    _base_addr = dma_base_addr;
    if (run() != DMA_OK)
        throw std::runtime_error("Can't initialize AXI DMA");
}


AxiDMA::~AxiDMA() {
    resetChannels();
    disableAllTxIrq();
    disableAllRxIrq();

    munmap(dma_hw, UNKNOWN_SIZE);
    close(fd);

    isRx       = 0;
    isTx       = 0;
    isSg       = false;
    _base_addr = AXIDMA_BASEADDR;
}


/**
 * @brief Send data via AXI DMA
 * @param[in] buff - buffer with data for sending
 *
 * @return > 0 - length of sended data.
 * @return -ERR_DMA_IS_DIRECT - Your AXI DMA is configure in Direct Mode
 * @return -ERR_DMA_BAD_ALLOC - can't allocate memory for buffer
 * @return -ERR_DMA_HAD_WORK  - AXI DMA is halting
 * @return -ERR_DMA_TX_IRQ    - get error interrupt
 * @return -ERR_DMA_TIMEOUT   - timeout of sending data
 * @note Stop MM2S channel after execution
 */
int AxiDMA::Send(const AxiDmaBuffer *buff) {
    try {
        if (!isSg)
            return -ERR_DMA_IS_DIRECT;

        std::lock_guard<std::mutex> lock(tx_mute);
        size_t buff_size = buff->GetSize() * sizeof(uint8_t);
        auto *txring     = new AxiDmaDescriptors(false);
        txring->InitDescriptors(txring->GetBufferBaseAddr(), buff_size);

        auto *tx_data    = (uint8_t *)allocBufferMem(buff_size,
                                                  txring->GetBufferBaseAddr());
        if (tx_data == nullptr) {
            delete txring;
            return -ERR_DMA_BAD_ALLOC;
        }
        buff->CopyInto(tx_data); // Copy from buff to tx_data

        /******* Launch sending *********/
        setTxCurDesc(txring->GetHeadDescriptorAddr());
        startTx();

        if (checkTxHalt()) {
            delete txring;
            munmap(tx_data, buff_size);
            return -ERR_DMA_HALT_WORK;
        }
        setTxTailDesc(txring->GetTailDescriptorAddr()); // here DMA start sending

        int status = waitTxComplete();
        if (status != DMA_OK) {
            delete txring;
            munmap(tx_data, buff_size);
            return status;
        }

        /*********************************/

        int transferred_bytes = (int)txring->ProcessDescriptors(true);

        /***** Set channel by default *****/
        setTxDefault();
        /*********************************/

        delete txring;
        munmap(tx_data, buff_size);

        return transferred_bytes;
    } catch (...) {
        throw;
    }
}


/**
 * @brief Send data via AXI DMA
 * @param[in] buff - buffer with data for sending
 *
 * @return > 0 - length of sended data
 * @return -ERR_DMA_IS_DIRECT - Your AXI DMA is configure in Direct Mode
 * @return -ERR_DMA_BAD_ALLOC - can't allocate memory for buffer
 * @return -ERR_DMA_HAD_WORK  - AXI DMA is halting
 * @return -ERR_DMA_TX_IRQ    - get error interrupt
 * @return -ERR_DMA_TIMEOUT   - timeout of sending data
 * @note Doesn't stop channel
 */
int AxiDMA::Send_repeat(const AxiDmaBuffer *buff) {
    if (!isSg)
        return -ERR_DMA_IS_DIRECT;

    std::lock_guard<std::mutex> lock(tx_mute);
    size_t buff_size = buff->GetSize() * sizeof(uint8_t);
    auto *txring     = new AxiDmaDescriptors(false);
    int status = txring->InitDescriptors(txring->GetBufferBaseAddr(), buff_size);
    if (status != 0) {
        delete txring;
        return status;
    }

    auto *tx_data = (uint8_t *)allocBufferMem(buff_size,
                                                 txring->GetBufferBaseAddr());
    if (tx_data == nullptr) {
        delete txring;
        return -ERR_DMA_BAD_ALLOC;
    }
    buff->CopyInto(tx_data); // Copy from buff to tx_data

    /******* Launch sending *********/
    if (!isTxRun()) {
        setTxCurDesc(txring->GetHeadDescriptorAddr());
        startTx();
    }

    if (checkTxHalt()) {
        delete txring;
        munmap(tx_data, buff_size);
        return -ERR_DMA_HALT_WORK;
    }
    setTxTailDesc(txring->GetTailDescriptorAddr()); // here DMA start sending

    status = waitTxComplete();
    if (status != DMA_OK) {
        setTxDefault();

        delete txring;
        munmap(tx_data, buff_size);
        return status;
    }
    /*********************************/

    int transferred_bytes = (int)txring->ProcessDescriptors(true);

    delete txring;
    munmap(tx_data, buff_size);
    return transferred_bytes;
}




/**
 * @brief Receive data via AXI DMA
 * @param[out] buff - buffer with data received from DMA
 * @param[in]  size - the size of expecting data (in bytes)
 *
 * @return > 0 - length of received data
 * @return -ERR_DMA_IS_DIRECT - Your AXI DMA is configure in Direct Mode
 * @return -ERR_DMA_BAD_ALLOC - can't allocate memory for buffer
 * @return -ERR_DMA_HAD_WORK  - AXI DMA is halting
 * @return -ERR_DMA_RX_IRQ    - get error interrupt
 * @return -ERR_DMA_TIMEOUT   - timeout of sending data
 */
int AxiDMA::Recv(AxiDmaBuffer *buff, size_t size) {
    try {
        if (!isSg)
            return -ERR_DMA_IS_DIRECT;

        std::lock_guard<std::mutex> lock(rx_mute);
        auto *rxring  = new AxiDmaDescriptors(true);
        auto *rx_data = (uint8_t *)allocBufferMem(size, rxring->GetBufferBaseAddr());
        if (rx_data == nullptr) {
            delete rxring;
            return -ERR_DMA_BAD_ALLOC;
        }
        memset(rx_data, 0x00, size);
        rxring->InitDescriptors(rxring->GetBufferBaseAddr(), size);

        /******* Launch receiving *********/
        setRxCurDesc(rxring->GetHeadDescriptorAddr());
        startRx();

        if (checkRxHalt()) {
            delete rxring;
            munmap(rx_data, size);
            return -ERR_DMA_HALT_WORK;
        }
        setRxTailDesc(rxring->GetTailDescriptorAddr()); // here DMA start receive

        int status = waitRxComplete();
        if (status != DMA_OK) {
            delete rxring;
            munmap(rx_data, size);
            return status;
        }
        /*********************************/

        auto transferred_bytes = rxring->ProcessDescriptors(false);
        buff->CopyFrom(rx_data, transferred_bytes);

        /***** Set channel by default *****/
        setRxDefault();
        /*********************************/

        delete rxring;
        munmap(rx_data, size);

        return (int)transferred_bytes;
    } catch (...) {
        throw;
    }
}


/**
 * @brief Two-way transfer data via AXI DMA
 * @param[in]  tx     - buffer with data for sending
 * @param[out] rx     - buffer with receiving data
 * @param[in]  rx_len - the size of expecting receiving data
 *
 * @return > 0 - length of received data
 * @return -ERR_DMA_IS_DIRECT - Your AXI DMA is configure in Direct Mode
 * @return -ERR_DMA_BAD_ALLOC - can't allocate memory for buffer
 * @return -ERR_DMA_HAD_WORK  - AXI DMA is halting
 * @return -ERR_DMA_TX_IRQ    - get error interrupt from MM2S channel
 * @return -ERR_DMA_RX_IRQ    - get error interrupt from S2MM channel
 * @return -ERR_DMA_TIMEOUT   - timeout of sending data
 *
 * @note Prefer to use it, when you need fast sending and receiving data
 */
int AxiDMA::Transf(const AxiDmaBuffer *tx, AxiDmaBuffer *rx, size_t rx_len) {
    try {
        if (!isSg)
            return -ERR_DMA_IS_DIRECT;

        std::lock_guard<std::mutex> lock(tx_mute);
        size_t tx_len = tx->GetSize() * sizeof(uint8_t);
        auto *txring = new AxiDmaDescriptors(false);
        auto *rxring = new AxiDmaDescriptors(true);

        auto *tx_data = (uint8_t *)allocBufferMem(tx_len, txring->GetBufferBaseAddr());
        if (tx_data == nullptr) {
            delete txring;
            delete rxring;
            return -ERR_DMA_BAD_ALLOC;
        }
        auto *rx_data = (uint8_t *)allocBufferMem(rx_len, rxring->GetBufferBaseAddr());
        if (rx_data == nullptr) {
            delete txring;
            delete rxring;
            munmap(tx_data, tx_len);
            return -ERR_DMA_BAD_ALLOC;
        }

        memset(tx_data, 0x00, tx_len);
        memset(rx_data, 0x00, rx_len);
        tx->CopyInto(tx_data);

        if (txring->InitDescriptors(txring->GetBufferBaseAddr(), tx_len)
            != AxiDmaDescriptors::RING_OK) {
            delete txring;
            delete rxring;
            munmap(tx_data, tx_len);
            munmap(rx_data, rx_len);
            return -ERR_DMA_UNINIT;
        }

        if (rxring->InitDescriptors(rxring->GetBufferBaseAddr(), rx_len)
            != AxiDmaDescriptors::RING_OK) {
            delete txring;
            delete rxring;
            munmap(tx_data, tx_len);
            munmap(rx_data, rx_len);
            return -ERR_DMA_UNINIT;
        }

        /******* Launch transfering *********/
        setTxCurDesc(txring->GetHeadDescriptorAddr());
        setRxCurDesc(rxring->GetHeadDescriptorAddr());
        startTx();
        startRx();

        if (checkTxHalt()) {
            delete txring;
            delete rxring;
            munmap(tx_data, tx_len);
            munmap(rx_data, rx_len);
            return -ERR_DMA_HALT_WORK;
        }
        if (checkRxHalt()) {
            delete txring;
            delete rxring;
            munmap(tx_data, tx_len);
            munmap(rx_data, rx_len);
            return -ERR_DMA_HALT_WORK;
        }

        setRxTailDesc(rxring->GetTailDescriptorAddr());
        setTxTailDesc(txring->GetTailDescriptorAddr());

        int status = waitTxComplete();
        if (status != DMA_OK) {
            resetChannels();
            delete txring;
            delete rxring;
            munmap(tx_data, tx_len);
            munmap(rx_data, rx_len);
            return status;
        }

        status = waitRxComplete();
        if (status != DMA_OK) {
            resetChannels();
            delete txring;
            delete rxring;
            munmap(tx_data, tx_len);
            munmap(rx_data, rx_len);
            return status;
        }
        /*********************************/

        txring->ProcessDescriptors(false); // free MM2S descriptors
        int transferred_bytes = (int)rxring->ProcessDescriptors(false);

        /***** Set channels by default *****/
        resetChannels();
        enableAllTxIrq();
        enableAllRxIrq();

        setTxDelay(0);
        setTxThreshold(1);
        setRxDelay(0);
        setRxThreshold(1);
        /***********************************/

        rx->CopyFrom(rx_data, rx_len);
        delete txring;
        delete rxring;
        munmap(tx_data, tx_len);
        munmap(rx_data, rx_len);

        return transferred_bytes;
    } catch (...) {
        std::rethrow_exception(std::current_exception());
    }
}


/**
 * @brief Manual launch of transfer data via AXI DMA
 * @param[in] desc_chain - chain of descriptors
 * @param[in] way        - the way of transfer: true - RX; false - TX
 *
 * @return  DMA_OK            - launch successfuly
 * @return -ERR_DMA_IS_DIRECT - Your AXI DMA is configure in Direct Mode
 * @return -ERR_DMA_BAD_CHAIN - pass wrong chain of descriptors
 * @return -ERR_DMA_ALR_WORK  - AXI DMA already run (set Curdesc is useless)
 */
int AxiDMA::ManualPrepareTransfer(AxiDmaDescriptors *desc_chain, bool way) {
    try {
        if (!isSg)
            return -ERR_DMA_IS_DIRECT;

        if (way) {
            if (!desc_chain->IsRx())
                return -ERR_DMA_BAD_CHAIN;

            if (isRxRun())
                return -ERR_DMA_ALR_WORK;

            setRxCurDesc(desc_chain->GetHeadDescriptorAddr());
            startRx();
        } else {
            if (desc_chain->IsRx())
                return -ERR_DMA_BAD_CHAIN;

            if (isTxRun())
                return -ERR_DMA_ALR_WORK;

            setTxCurDesc(desc_chain->GetHeadDescriptorAddr());
            startTx();
        }

        return DMA_OK;
    } catch (const std::exception &exp) {
        std::rethrow_exception(std::current_exception());
    }
}


/**
 * @brief Manual launch of transfer data via AXI DMA
 * @param[in] desc_chain - chain of descriptors
 * @param[in] way        - the way of transfer: true - RX; false - TX
 *
 * @return  DMA_OK           - launch successfuly
 * @return -ERR_DMA_HAD_WORK - DMA is halt
 */
int AxiDMA::ManualStartTransfer(AxiDmaDescriptors *desc_chain, bool way) {
    try {
        if (way) {
            if (!desc_chain->IsRx())
                return -ERR_DMA_BAD_CHAIN;

            if (checkRxHalt())
                return -ERR_DMA_HALT_WORK;

            setRxTailDesc(desc_chain->GetTailDescriptorAddr());
        } else {
            if (desc_chain->IsRx())
                return -ERR_DMA_BAD_CHAIN;

            if (checkTxHalt())
                return -ERR_DMA_HALT_WORK;
            setTxTailDesc(desc_chain->GetTailDescriptorAddr());
        }

        return DMA_OK;
    } catch (...) {
        std::rethrow_exception(std::current_exception());
    }
}


/**
 * @brief Polling AXI DMA interrupts
 * @param[in] way - the way of transfer: true - RX; false - TX
 *
 * @return  DMA_OK             - get interrupt of complete
 * @return -ERR_DMA_RX_TIMEOUT - get timeout on RX channel
 * @return -ERR_DMA_TX_TIMEOUT - get timeout on TX channel
 * @return -ERR_DMA_RX_IRQ     - get error interrupt on RX channel
 * @return -ERR_DMA_TX_IRQ     - get error interrupt on TX channel
 * @note All got interrupts is acknowledge here
 */
int AxiDMA::ManualPollIrq(bool way) {
    int status;

    if (way) {
        status = waitRxComplete();
        if (status != DMA_OK)
            return status;
    } else {
        status = waitTxComplete();
        if (status != DMA_OK)
            return status;
    }

    return status;
}


/**
 * @brief Reset chosen channel, enable all interrupts
 * @param[in] way - the way of transfer: true - RX; false - TX
 *
 * @return none
 */
void AxiDMA::ManualResetChannel(bool way) {
    if (way)
        setRxDefault();
    else
        setTxDefault();
}


/**
 * @brief Allocate memory for buffer which used in descriptor register Buffer
 *   Address (0x08)
 * @param[in] buffer_size         - size of buffer (in bytes)
 * @param[in] buffer_base_address - the base address of the buffer
 *
 * @return pointer to allocated memory or nullptr if fail
 */
void *AxiDMA::ManualAllocMemory(size_t buffer_size, uint32_t buffer_base_address) {
    return allocBufferMem(buffer_size, buffer_base_address);
}


/**
 * @brief Check mode of AXI DMA
 * @param none
 *
 * @return true  - AXI DMA configured in Scatter Gather mode
 * @return false - AXI DMA configured in Direct Register mode
 */
bool AxiDMA::IsSg() {
    return isSg;
}


/**
 * @brief Send data via AXI DMA which configure in Direct Mode
 * @param[in] buff - buffer with data for sending
 *
 * @return > 0                 - the size of transferring data
 * @return -ERR_DMA_IS_SG      - Your AXI DMA is configure in Scatter/Gather mode
 * @return -ERR_DMA_BAD_ALLOC  - can't allocate memory for buffer
 * @return -ERR_DMA_HAD_WORK   - AXI DMA is halting
 * @return -ERR_DMA_TX_IRQ     - get error interrupt
 * @return -ERR_DMA_TX_IDLEOUT - timeout of waiting completion transaction
 * @return -ERR_DMA_TX_TIMEOUT - timeout of sending data
 */
int AxiDMA::DirectSend(const AxiDmaBuffer *buff) {
    try {
        if (isSg)
            return -ERR_DMA_IS_SG;

        std::lock_guard<std::mutex> lock(tx_mute);
        size_t buff_size = buff->GetSize() * sizeof(uint8_t);
        auto *tx_data    = (uint8_t *)allocBufferMem(buff_size, TX_BUFFER_BASE);
        if (tx_data == nullptr)
            return -ERR_DMA_BAD_ALLOC;

        startTx();
        if (checkTxHalt()) {
            munmap(tx_data, buff_size);
            return -ERR_DMA_HALT_WORK;
        }

        buff->CopyInto(tx_data);
        setSourceAddress(TX_BUFFER_BASE); // XXX: need to add check SA alignment

        // XXX: need to add check size
        setTxLength(buff_size); // start transfer

        int status = waitTxDirectComplete();
        if (status != DMA_OK) {
            munmap(tx_data, buff_size);
            return status;
        }
        status = waitTxComplete();
        if (status != DMA_OK) {
            munmap(tx_data, buff_size);
            return status;
        }

        munmap(tx_data, buff_size);
        return getTxLength();
    } catch (...) {
        throw;
    }
}


/**
 * @brief Recv data via AXI DMA which configure in Direct Mode
 * @param[out] buff - buffer with data for receiving
 * @param[in]  size - the (max) size of incoming buffer (in bytes)
 *
 * @return > 0                 - the size of receiving data
 * @return -ERR_DMA_IS_SG      - Your AXI DMA is configure in Scatter/Gather mode
 * @return -ERR_DMA_BAD_ALLOC  - can't allocate memory for buffer
 * @return -ERR_DMA_HAD_WORK   - AXI DMA is halting
 * @return -ERR_DMA_RX_IRQ     - get error interrupt
 * @return -ERR_DMA_RX_IDLEOUT - timeout of waiting completion transaction
 * @return -ERR_DMA_RX_TIMEOUT - timeout of receiving data
 */
int AxiDMA::DirectRecv(AxiDmaBuffer *buff, size_t size) {
    try {
        if (isSg)
            return -ERR_DMA_IS_SG;

        auto *rx_data = (uint8_t *)allocBufferMem(size, RX_BUFFER_BASE);
        if (rx_data == nullptr)
            return -ERR_DMA_BAD_ALLOC;

        std::lock_guard<std::mutex> lock(rx_mute);
        int status = getRxIRQ(); // check, if already have some data
        if (status != DMA_OK) {
            if (ackRxIRQ(status) == DMA_OK) {
                uint32_t transferred_len = getRxLength();
                buff->CopyFrom(rx_data, transferred_len);
                munmap(rx_data, size);

                return transferred_len;
            }
        }

        startRx();
        if (checkRxHalt()) {
            munmap(rx_data, size);
            return -ERR_DMA_HALT_WORK;
        }
        setDestinationAddress(RX_BUFFER_BASE);
        setRxLength(size);

        status = waitRxDirectComplete();
        if (status != DMA_OK) {
            // ???
        }
        status = waitRxComplete();
        if (status != DMA_OK) {
            munmap(rx_data, size);
            return status;
        }

        uint32_t transferred_len = getRxLength();

        buff->CopyFrom(rx_data, transferred_len);
        munmap(rx_data, size);

        return transferred_len;
    } catch (...) {
        throw;
    }
}


/**
 * @brief Recv data via AXI DMA which configure in Direct Mode
 * @param[in] dst_addr - address of physical allocated memory for receiving data
 * @param[in] size - the (max) size of incoming buffer (in bytes)
 *
 * @return > 0                 - the size of receiving data
 * @return -ERR_DMA_IS_SG      - Your AXI DMA is configure in Scatter/Gather mode
 * @return -ERR_DMA_HAD_WORK   - AXI DMA is halting
 * @return -ERR_DMA_RX_IRQ     - get error interrupt
 * @return -ERR_DMA_RX_IDLEOUT - timeout of waiting completion transaction
 * @return -ERR_DMA_RX_TIMEOUT - timeout of receiving data
 */
int AxiDMA::DirectManualRecv(uint32_t dst_addr, size_t size) {
    try {
        std::lock_guard<std::mutex> lock(rx_mute);
        int status = getRxIRQ();
        if (status != DMA_OK) {
            if (ackRxIRQ(status) == DMA_OK) {
                uint32_t transferred_len = getRxLength();
                return transferred_len;
            }
        }

        startRx();
        if (checkRxHalt())
            return -ERR_DMA_HALT_WORK;
        setDestinationAddress(dst_addr);
        setRxLength(size);

        status = waitRxDirectComplete();
        if (status != DMA_OK) {
            // ???
        }
        status = waitRxComplete();
        if (status != DMA_OK) {
            return status;
        }

        uint32_t transferred_len = getRxLength();
        return transferred_len;
    } catch (...) {
        throw;
    }
}

/**
 * @brief Get control register of chosen channel
 * @param[in] way - the flag of channel: true - S2MM; false - MM2S
 *
 * @return control register (0x00/0x30)
 * @note used for debug, may be deprecated
 */
uint32_t AxiDMA::GetControl(bool way) {
    if (way) return getRxControl();
    else     return getTxControl();
}


/**
 * @brief Get status register of chosen channel
 * @param[in] way - the flag of channel: true - S2MM; false - MM2S
 *
 * @return status register (0x04/0x34)
 * @note used for debug, may be deprecated
 */
uint32_t AxiDMA::GetStatus(bool way) {
    if (way) return getRxStatus();
    else     return getTxStatus();
}


/**
 * @brief Get CURDESC register of chosen channel
 * @param[in] way - the flag of channel: true - S2MM; false - MM2S
 *
 * @return CURDESC register (0x08/0x38)
 * @note used for debug, may be deprecated
 */
uint32_t AxiDMA::GetCurrDesc(bool way) {
    if (way) return dma_hw->s2mm_curdesc;
    else     return dma_hw->mm2s_curdesc;
}


/**
 * @brief Get TAILDESC register of chosen channel
 * @param[in] way - the flag of channel: true - S2MM; false - MM2S
 *
 * @return TAILDESC register (0x10/0x40)
 * @note used for debug, may be deprecated
 */
uint32_t AxiDMA::GetTailDesc(bool way) {
    if (way) return dma_hw->s2mm_taildesc;
    else     return dma_hw->mm2s_taildesc;
}


/**
 * @brief Launch AXI DMA channels, enable all interrupts
 * @param none
 *
 * @return  DMA_OK           - initialization was successful
 * @return -ERR_DMA_OPEN_FD  - can't get access to /dev/mem
 * @return -ERR_DMA_DEV_MAP  - can't mmap to /dev/mem
 * @return -ERR_DMA_RESET_TX - can't reset Tx channel
 * @return -ERR_DMA_RESET_RX - can't reset Rx channel
 * @return -ERR_DMA_HAD_WORK - AXI DMA already run
 * @return -EXIT_FAILURE     - catch exception
 */
int AxiDMA::run() {
    try {
        int status = initialization();
        if (status != DMA_OK) {
            return status;
        }

        status = resetChannels();
        if (status != DMA_OK) {
            return status;
        }

        if (isTx) {
            disableAllTxIrq();
            enableAllTxIrq();

            status = setTxDelay(0);
            if (status != DMA_OK)
                return status;

            status = setTxThreshold(1);
            if (status != DMA_OK)
                return status;
        }

        if (isRx) {
            disableAllRxIrq();
            enableAllRxIrq();

            status = setRxDelay(0);
            if (status != DMA_OK) {
                return status;
            }

            status = setRxThreshold(1);
            if (status != DMA_OK) {
                return status;
            }
        }

        return DMA_OK;
    } catch (const std::exception &exp) {
        return -EXIT_FAILURE;
    }
}


/**
 * @brief Memory map of AXI DMA with dma_hw structure
 * @param none
 *
 * @return  DMA_OK          - mmap was successful
 * @return -ERR_DMA_OPEN_FD - can't open /dev/mem
 * @return -ERR_DMA_DEV_MAP - can't mmap to /dev/mem
 */
int AxiDMA::initialization() {
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
        return -ERR_DMA_OPEN_FD;

    dma_hw = (dma_device_t *) mmap(nullptr, UNKNOWN_SIZE, PROT_READ | PROT_WRITE,
                                   MAP_SHARED, fd, _base_addr);
    if (dma_hw == MAP_FAILED)
        return -ERR_DMA_DEV_MAP;

    if (hasRx(dma_hw)) isRx = 1;
    if (hasTx(dma_hw)) isTx = 1;
    if (hasSg(dma_hw)) isSg = true;

    return DMA_OK;
}


/**
 * @brief Reset MM2S channel, enable all interrupts, set delay and threshold value
 * @param none
 *
 * @return none
 */
inline void AxiDMA::setTxDefault() {
    int delay = RESET_TIMEOUT;

    resetTx();
    while(!isTxReset()) {
        delay--;
        if (delay == 0)
            return;
    }
    enableAllTxIrq();

    setTxDelay(0);
    setTxThreshold(1);
}


/**
 * @brief Reset S2MM channel, enable all interrupts, set delay and threshold value
 * @param none
 *
 * @return none
 */
inline void AxiDMA::setRxDefault() {
    int delay = RESET_TIMEOUT;

    resetRx();
    while(!isRxReset()) {
        delay--;
        if (delay == 0)
            return;
    }
    enableAllRxIrq();

    setRxDelay(0);
    setRxThreshold(1);
}


/**
 * @brief Soft reset signal for both channels
 * @param none
 *
 * @return  DMA_OK           - soft reset complete successfully
 * @return -ERR_DMA_RESET_TX - can't reset Tx channel
 * @return -ERR_DMA_RESET_RX - can't reset Rx channel
 */
int AxiDMA::resetChannels() {
    int delay = RESET_TIMEOUT;

    if (isTx) {
        resetTx();
        while(!isTxReset()) {
            delay--;
            if (delay == 0)
                return -ERR_DMA_RESET_TX;
        }
        delay = RESET_TIMEOUT;
    }

    if (isRx) {
        resetRx();
        while(!isRxReset()) {
            delay--;
            if (delay == 0)
                return -ERR_DMA_RESET_RX;
        }
    }

    return DMA_OK;
}


/**
 * @brief Set soft reset signal (3 bit in MM2S channel) for Tx channel
 * @param none
 *
 * @return none
 */
void AxiDMA::resetTx() {
    setTxControl(getTxControl() | DMACR_RESET_MASK); // XXX: in orig is &
}


/**
 * @brief Set soft reset signal (3 bit in S2MM channel) for Rx channel
 * @param none
 *
 * @return none
 */
void AxiDMA::resetRx() {
    setRxControl(getRxControl() | DMACR_RESET_MASK);
}


/**
 * @brief Check value of soft reset bit from MM2S control register
 * @param none
 *
 * @return true  - if channel in normal operation (in non reset state)
 * @return false - if reset in progress
 */
inline bool AxiDMA::isTxReset() {
    return (getTxControl() & DMACR_RESET_MASK) == 0;
}


/**
 * @brief Check value of soft reset bit from S2MM control register
 * @param none
 *
 * @return true  - if channel in normal operation (in non reset state)
 * @return false - if reset in progress
 */
inline bool AxiDMA::isRxReset() {
    return (getRxControl() & DMACR_RESET_MASK) == 0;
}


/**
 * @brief Get value of MM2S control register
 * @param none
 *
 * @return value of MM2S control register
 */
inline uint32_t AxiDMA::getTxControl() {
    return dma_hw->mm2s_dmacr;
}


/**
 * @brief Get value of MM2S status register
 * @param none
 *
 * @return value of MM2S status register
 */
inline uint32_t AxiDMA::getTxStatus() {
    return dma_hw->mm2s_dmasr;
}


/**
 * @brief Get value of S2MM control register
 * @param none
 *
 * @return value of S2MM control register
 */
inline uint32_t AxiDMA::getRxControl() {
    return dma_hw->s2mm_dmacr;
}


/**
 * @brief Get value of S2MM status register
 * @param none
 *
 * @return value of S2MM status register
 */
inline uint32_t AxiDMA::getRxStatus() {
    return dma_hw->s2mm_dmasr;
}


/**
 * @brief Write into MM2S control register
 * @param[in] control_reg - value for writing
 *
 * @return none
 */
inline void AxiDMA::setTxControl(uint32_t control_reg) {
    dma_hw->mm2s_dmacr = control_reg;
}


/**
 * @brief Write into MM2S status register
 * @param[in] status_reg - value for writing
 *
 * @return none
 */
inline void AxiDMA::setTxStatus(uint32_t status_reg) {
    dma_hw->mm2s_dmasr = status_reg;
}


/**
 * @brief Write into MM2S curdes register
 * @param[in] curr_desc - value for writing
 *
 * @return none
 */
inline void AxiDMA::setTxCurDesc(uint32_t curr_desc) {
    dma_hw->mm2s_curdesc = curr_desc;
}


/**
 * @brief Write into MM2S taildesc register
 * @param[in] tail_desc - value for writing
 *
 * @return none
 */
inline void AxiDMA::setTxTailDesc(uint32_t tail_desc) {
    dma_hw->mm2s_taildesc = tail_desc;
}


/**
 * @brief Write into S2MM control register
 * @param[in] control_reg - value for writing
 *
 * @return none
 */
inline void AxiDMA::setRxControl(uint32_t control_reg) {
    dma_hw->s2mm_dmacr = control_reg;
}


/**
 * @brief Write into S2MM status register
 * @param[in] status_reg - value for writing
 *
 * @return none
 */
inline void AxiDMA::setRxStatus(uint32_t status_reg) {
    dma_hw->s2mm_dmasr = status_reg;
}


/**
 * @brief Write into S2MM curdes register
 * @param[in] curr_desc - value for writing
 *
 * @return none
 */
inline void AxiDMA::setRxCurDesc(uint32_t curr_desc) {
    dma_hw->s2mm_curdesc = curr_desc;
}


/**
 * @brief Write into S2MM taildes register
 * @param[in] tail_desc - value for writing
 *
 * @return none
 */
inline void AxiDMA::setRxTailDesc(uint32_t tail_desc) {
    dma_hw->s2mm_taildesc = tail_desc;
}


/**
 * @brief Write into MM2S SA register
 * @param[in] src_addr - value for writing
 *
 * @return none
 */
inline void AxiDMA::setSourceAddress(uint32_t src_addr) {
    dma_hw->mm2s_src_addr = src_addr;
}


/**
 * @brief Write into S2MM DA register
 * @param[in] dst_addr - value for writing
 *
 * @return none
 */
inline void AxiDMA::setDestinationAddress(uint32_t dst_addr) {
    dma_hw->s2mm_dst_addr = dst_addr;
}


/**
 * @brief Write into MM2S LENGTH register
 * @param[in] length - value for writing
 *
 * @return none
 */
inline void AxiDMA::setTxLength(uint32_t length) {
    dma_hw->mm2s_length = length;
}


/**
 * @brief Write into S2MM LENGTH register
 * @param[in] length - value for writing
 *
 * @return none
 */
inline void AxiDMA::setRxLength(uint32_t length) {
    dma_hw->s2mm_length = length;
}


/**
 * @brief Get length of transferred data
 * @param none
 *
 * @return length of sending bytes on MM2S
 */
inline uint32_t AxiDMA::getTxLength() {
    return dma_hw->mm2s_length;
}


/**
 * @brief Get length of transferred data
 * @param none
 *
 * @return length of actual bytes received on S2MM
 */
inline uint32_t AxiDMA::getRxLength() {
    return dma_hw->s2mm_length;
}


/**
 * @brief Run MM2S channel
 * @param none
 *
 * @return none
 */
void AxiDMA::startTx() {
    if (!isTxWork())
        setTxControl(getTxControl() | DMACR_RUN_MASK);
}


/**
 * @brief Run S2MM channel
 * @param none
 *
 * @return none
 */
void AxiDMA::startRx() {
    if (!isRxWork())
        setRxControl(getRxControl() | DMACR_RUN_MASK);
}


/**
 * @brief Set Interrupt Delay Timeout (IRQDelay) for MM2S channel
 * @param[in] delay - delay time period
 *
 * @return  DMA_OK            - if set delay successfully
 * @return -ERR_DMA_BAD_DELAY - incorrect delay value
 * @return -ERR_DMA_HAD_WORK  - channel is launched
 * @note See PG 021. Setting @delay value to zero disables the delay timer interrupt
 */
int AxiDMA::setTxDelay(uint32_t delay) {
    if (delay > 255)
        return -ERR_DMA_BAD_DELAY;
    if (isTxWork())
        return -ERR_DMA_HALT_WORK;

    delay = delay << DELAY_SHIFT;
    uint32_t control_reg = (getTxControl() & ~DMACR_DELAY_MASK) | delay;
    setTxControl(control_reg);

    return DMA_OK;
}


/**
 * @brief Set Interrupt Delay Timeout (IRQDelay) for S2MM channel
 * @param[in] delay - delay time period
 *
 * @return  DMA_OK            - if set delay successfully
 * @return -ERR_DMA_BAD_DELAY - incorrect delay value
 * @return -ERR_DMA_HAD_WORK  - channel is launched
 * @note See PG 021. Setting @delay value to zero disables the delay timer interrupt
 */
int AxiDMA::setRxDelay(uint32_t delay) {
    if (delay > 255)
        return -ERR_DMA_BAD_DELAY;
    if (isRxWork())
        return -ERR_DMA_HALT_WORK;

    delay = delay << DELAY_SHIFT;
    uint32_t control_reg = (getRxControl() & ~DMACR_DELAY_MASK) | delay;
    setRxControl(control_reg);

    return DMA_OK;
}


/**
 * @brief Set Interrupt Threshold (IRQThreshold) for MM2S channel
 * @param[in] threshold - threshold value
 *
 * @return  DMA_OK            - if set threshold successfully
 * @return -ERR_DMA_BAD_DELAY - incorrect value
 * @return -ERR_DMA_HAD_WORK  - channel is launched
 * @note See PG 021. The minimum setting for the threshold is 0x01.
 */
int AxiDMA::setTxThreshold(uint32_t threshold) {
    if (threshold > 255 || threshold < 1)
        return -ERR_DMA_BAD_THRESHOLD;
    if (isTxWork())
        return -ERR_DMA_HALT_WORK;

    threshold = threshold << THRESHOLD_SHIFT;
    uint32_t control_reg = (getTxControl() & ~DMACR_THRESHOLD_MASK) | threshold;
    setTxControl(control_reg);

    return DMA_OK;
}


/**
 * @brief Set Interrupt Threshold (IRQThreshold) for S2MM channel
 * @param[in] threshold - threshold value
 *
 * @return  DMA_OK            - if set threshold successfully
 * @return -ERR_DMA_BAD_DELAY - incorrect value
 * @return -ERR_DMA_HAD_WORK  - channel is launched
 * @note See PG 021. The minimum setting for the threshold is 0x01.
 */
int AxiDMA::setRxThreshold(uint32_t threshold) {
    if (threshold > 255 || threshold < 1)
        return -ERR_DMA_BAD_THRESHOLD;
    if (isRxWork())
        return -ERR_DMA_HALT_WORK;

    threshold = threshold << THRESHOLD_SHIFT;
    uint32_t control_reg = (getRxControl() & ~DMACR_THRESHOLD_MASK) | threshold;
    setRxControl(control_reg);

    return DMA_OK;
}


/**
 * @brief Check work state of MM2S channel
 * @param none
 *
 * @return true  - if channel is worked
 * @return false - if channel is halted
 */
inline bool AxiDMA::isTxWork() {
    return (dma_hw->mm2s_dmasr & DMASR_HALT_MASK) == 0;
}


/**
 * @brief Check work state of S2MM channel
 * @param none
 *
 * @return true  - if channel is worked
 * @return false - if channel is halted
 */
inline bool AxiDMA::isRxWork() {
    return (dma_hw->s2mm_dmasr & DMASR_HALT_MASK) == 0;
}


/**
 * @brief Check run state of MM2S channel
 * @param none
 *
 * @return true  - if channel is run
 * @return false - if channel is stopped
 */
inline bool AxiDMA::isTxRun() {
    return (dma_hw->mm2s_dmacr & DMACR_RUN_MASK) != 0;
}


/**
 * @brief Check run state of S2MM channel
 * @param none
 *
 * @return true  - if channel is run
 * @return false - if channel is stopped
 */
inline bool AxiDMA::isRxRun() {
    return (dma_hw->s2mm_dmacr & DMACR_RUN_MASK) != 0;
}


/**
 * @brief Enable all interrupts for MM2S channel
 * @param none
 *
 * @return none
 */
void AxiDMA::enableAllTxIrq() {
    volatile uint32_t control_reg = getTxControl() | DMACR_ALLIRQ_MASK;
    setTxControl(control_reg);
}


/**
 * @brief Enable all interrupts for S2MM channel
 * @param none
 *
 * @return none
 */
void AxiDMA::enableAllRxIrq() {
    volatile uint32_t control_reg = getRxControl() | DMACR_ALLIRQ_MASK;
    setRxControl(control_reg);
}


/**
 * @brief Disable all interrupts for MM2S channel
 * @param none
 *
 * @return none
 */
void AxiDMA::disableAllTxIrq() {
    volatile uint32_t control_reg = getTxControl() & ~DMACR_ALLIRQ_MASK;
    setTxControl(control_reg);
}


/**
 * @brief Disable all interrupts for S2MM channel
 * @param none
 *
 * @return none
 */
void AxiDMA::disableAllRxIrq() {
    volatile uint32_t control_reg = getRxControl() & ~DMACR_ALLIRQ_MASK;
    setRxControl(control_reg);
}


/**
 * @brief Get value of interrupts of MM2S channel
 * @param none
 *
 * @return masked interrupts value (i.e. 0x1000 - IOC_IrqEn, Interrupt on Complete);
 */
inline uint32_t AxiDMA::getTxIRQ() {
    return (getTxStatus() & DMASR_IRQALL_MASK);
}


/**
 * @brief Get value of interrupts of S2MM channel
 * @param none
 *
 * @return masked interrupts value (i.e. 0x1000 - IOC_IrqEn, Interrupt on Complete);
 */
inline uint32_t AxiDMA::getRxIRQ() {
    return (getRxStatus() & DMASR_IRQALL_MASK);
}


/**
 * @brief Clear interrupt bits of MMM2S channel
 * @param irq - interrupt for clearing
 *
 * @return  DMA_OK             - doesn't get error interrupt (Err_Irq)
 * @return  irq                - value of error interrupt
 * @return -ERR_DMA_TX_TIMEOUT - IRQ is absent, may be was timeout?
 */
uint32_t AxiDMA::ackTxIRQ(uint32_t irq) {
    if (irq == 0)
        return -ERR_DMA_TX_TIMEOUT;

    setTxStatus(irq);
    if (irq & DMASR_ERRIRQ_MASK)
        return irq;

    return DMA_OK;
}


/**
 * @brief Clear interrupt bits of MMM2S channel
 * @param irq - interrupt for clearing
 *
 * @return DMA_OK              - get interrupt on complete (IoC)
 * @return irq                 - value of error interrupt
 * @return -ERR_DMA_RX_TIMEOUT - IRQ is absent, may be was timeout?
 */
uint32_t AxiDMA::ackRxIRQ(uint32_t irq) {
    if (irq == 0)
        return -ERR_DMA_RX_TIMEOUT;

    setRxStatus(irq);
    if (irq & DMASR_ERRIRQ_MASK)
        return irq;

    return DMA_OK;
}


/**
 * @brief Allocate memory (mmap) for user buffer data.
 * @param[in] buff_size           - size of buffer (in bytes)
 * @param[in] buffer_base_address - base address of buffer which using for DMA
 *   descriptors
 *
 * @return data    - pointer to allocated memory
 * @return nullptr - if can't allocate memory
 * @note Used mmap to buffer address which write into 0x08 register in DMA descriptor.
 *   After sending/receiving data, you have to deallocate (munmap) this memory.
 */
void *AxiDMA::allocBufferMem(size_t buff_size, uint32_t buffer_base_address) {
    auto *data = (uint8_t *)mmap(nullptr, buff_size, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd, buffer_base_address);
    if (data == MAP_FAILED)
        return nullptr;

    return data;
}


/**
 * @brief Check work state of Tx channel (mm2s)
 * @param none
 *
 * @return true  - if Tx channel is halted
 * @return false - if Tx channel is running
 */
bool AxiDMA::checkTxHalt() {
    int delay = RESET_TIMEOUT;
    while(!isTxWork()) {
        delay--;
        if (delay == 0)
            return true;
    }

    return false;
}


/**
 * @brief Check work state of Rx channel (s2mm)
 * @param none
 *
 * @return true  - if Rx channel is halted
 * @return false - if Rx channel is running
 */
bool AxiDMA::checkRxHalt() {
    int delay = RESET_TIMEOUT;
    while(!isRxWork()) {
        delay--;
        if (delay == 0)
            return true;
    }

    return false;
}


/**
 * @brief Wait interrupts and clear it (for MM2S)
 * @param none
 *
 * @return  DMA_OK             - DMA transaction was complete successfully
 * @return -ERR_DMA_TX_TIMEOUT - timeout of data transfer
 * @return -ERR_DMA_TX_IRQ     - get interrupt of error
 * @note Used std::this_thread::sleep_for for delay.
 */
inline int AxiDMA::waitTxComplete() {
    /*size_t timeout_10s = 10000000; // 10 second (-1)
    volatile uint32_t tx_irq = getTxIRQ();
    while (!tx_irq && (timeout_10s > 0)) { // search interrupt register
        tx_irq = getTxIRQ();
        //if (tx_irq == 0)
         //   std::this_thread::sleep_for(std::chrono::microseconds(10));
        timeout_10s -= 1;
    }

    int status = ackTxIRQ(tx_irq);
    if ((status != DMA_OK) && (timeout_10s > 0)) {
        printf("tx_irq_err: %x | %d s\r\n", status, timeout_10s); // debug
        return -ERR_DMA_TX_IRQ;
    } else if (timeout_10s <= 0 && (tx_irq != DMASR_IOCIRQ_MASK))
        return -ERR_DMA_TX_TIMEOUT;

    return DMA_OK;*/
    constexpr int wait_time   = 3; //10;
    constexpr uint64_t second = 1000000; // 1 second in microseconds
    size_t timeout_10s        = second * wait_time; // 10 second (-1)
    volatile uint32_t tx_irq  = getTxIRQ();

    for (auto i = timeout_10s; i > 0; i -= 100) {
        tx_irq = getTxIRQ();
        if (tx_irq != 0)
            break;
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    int status = ackTxIRQ(tx_irq);
    if (status == DMA_OK)
        return DMA_OK;
    else if (status != -ERR_DMA_TX_TIMEOUT) {
        printf("tx_irq_err: %x | %d s\r\n", status, timeout_10s); // debug
        return -ERR_DMA_TX_IRQ;
    } else
        return -ERR_DMA_TX_TIMEOUT;
}


/**
 * @brief Wait interrupts and clear it (for S2MM)
 * @param none
 *
 * @return  DMA_OK             - DMA transaction was complete successfully
 * @return -ERR_DMA_RX_TIMEOUT - timeout of data transfer
 * @return -ERR_DMA_RX_IRQ     - get interrupt of error
 * @note Used std::this_thread::sleep_for for delay.
 */
inline int AxiDMA::waitRxComplete() {
    constexpr int wait_time   = 3; //10;
    constexpr uint64_t second = 1000000; // 1 second in microseconds
    size_t timeout_10s        = second * wait_time; // 10 second (-1)
    volatile uint32_t rx_irq  = getRxIRQ();

    for (auto i = timeout_10s; i > 0; i -= 100) {
        rx_irq = getRxIRQ();
        if (rx_irq != 0)
            break;
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    int status = ackRxIRQ(rx_irq);
    if (status == DMA_OK)
        return DMA_OK;
    else if (status != -ERR_DMA_RX_TIMEOUT) {
        printf("rx_irq_err: %x\r\n", status); // debug
        return -ERR_DMA_RX_IRQ;
    } else
        return -ERR_DMA_RX_TIMEOUT;
}


/**
 * @brief Wait until complete transferring in direct mode
 * @param none
 *
 * @return  DMA_OK             -
 * @return -ERR_DMA_TX_IDLEOUT -
 */
inline int AxiDMA::waitTxDirectComplete() {
    size_t timeout_10s = 10000000; // 10 second (-1)
    volatile uint32_t tx_irq = getTxStatus() & DMASR_IDLE_MASK;
    while (!tx_irq && (timeout_10s > 0)) { // search interrupt register
        tx_irq = getTxStatus() & DMASR_IDLE_MASK;
        /*if (tx_irq == 0)
            std::this_thread::sleep_for(std::chrono::microseconds(10));*/
        timeout_10s -= 1;
    }

    if (timeout_10s <= 0 && ((tx_irq & DMASR_IDLE_MASK) == 0)) {
        return -ERR_DMA_TX_IDLEOUT;
    }

    return DMA_OK;
}


/**
 * @brief Wait until complete transferring in direct mode
 * @param none
 *
 * @return  DMA_OK             -
 * @return -ERR_DMA_RX_IDLEOUT -
 */
inline int AxiDMA::waitRxDirectComplete() {
    constexpr int wait_time   = 10;
    constexpr uint64_t second = 1000000; // 1 second in microseconds
    size_t timeout_10s        = second * wait_time; // 10 second (-1)
    volatile uint32_t rx_irq = getRxStatus() & DMASR_IDLE_MASK;

    while (!rx_irq && (timeout_10s > 0)) { // search interrupt register
        rx_irq = getRxStatus() & DMASR_IDLE_MASK;
        /*if (rx_irq == 0)
            std::this_thread::sleep_for(std::chrono::microseconds(5));*/
        timeout_10s -= wait_time * 5;
    }

    if (timeout_10s <= 0 && ((rx_irq & DMASR_IDLE_MASK) == 0)) {
        return -ERR_DMA_RX_IDLEOUT;
    }

    return DMA_OK;
}
