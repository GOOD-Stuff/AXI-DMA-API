//
// Created by vldmr on 25.06.19.
//
#include "AxiDmaDescriptors.h"


AxiDmaDescriptors::AxiDmaDescriptors(bool chan) {
    if (chan) {
        chain_phys_baseaddr = RX_BASEADDR;
        buffer_phys_baseaddr = RX_BUFFER_BASE;
        isRx = chan;
    } else {
        chain_phys_baseaddr = TX_BASEADDR;
        buffer_phys_baseaddr = TX_BUFFER_BASE;
    }
}


AxiDmaDescriptors::~AxiDmaDescriptors() {
    munmap(chain_virt_baseaddr, _chain_size);
    close(fd);
    isRx           = false;
    _chain_size    = 0;
    bytes_per_desc = BD_OPTIMAL_SIZE;
    bd_count       = 0;
}


/**
 * @brief Set count of bytes per descriptor
 * @param[in] value - count of bytes per descriptor
 *
 * @return none
 * @note By default, bytes per descriptor is equal to 3 MByte
 */
void AxiDmaDescriptors::SetBytesPerDesc(uint32_t value) {
    if (value == 0) value = BD_OPTIMAL_SIZE;
    bytes_per_desc = value;
}


/**
 * @brief Get count of descriptors
 * @param none
 *
 * @return count of descriptors
 */
int AxiDmaDescriptors::GetCountDescriptors() {
    return bd_count;
}


/**
 * @brief Get base address of descriptor chain
 * @param none
 *
 * @return base (start/head) address of descriptor chain
 */
uint32_t AxiDmaDescriptors::GetHeadDescriptorAddr() {
    return chain_phys_baseaddr;
}


/**
 * @brief Get tail address of descriptor chain
 * @param none
 *
 * @return tail (end) addres of descriptor chain
 */
uint32_t AxiDmaDescriptors::GetTailDescriptorAddr() {
    return getTailMem(chain_phys_baseaddr, bd_count);
}


/**
 * @brief Get base address of buffer
 * @param none
 *
 * @return buffer base address
 */
uint32_t AxiDmaDescriptors::GetBufferBaseAddr() {
    return buffer_phys_baseaddr;
}


/**
 * @brief Initialize chain of descripors
 * @param[in] buffer_addr - address of buffer (for 0x08 register in descriptor)
 * @param[in] buffer_size - the size of transferring data (in bytes)
 *
 * @return  RING_OK     - all descriptors was initialize successfully
 * @return -ERR_BDCNT   - can't calculate count of descriptors
 * @return -ERR_SIZE    - pass not aligned size of data
 * @return -ERR_OPEN_FD - can't get access to /dev/mem
 * @return -ERR_MAP     - can't mmap to SG descriptor
 * @note Calculate count of descriptors by passing @buffer_size.
 *   For @buffer_addr prefer to use @GetBufferBaseAddr().
 */
int AxiDmaDescriptors::InitDescriptors(uint32_t buffer_addr, size_t buffer_size) {
    int status = prepareChain(buffer_size);
    if (status != RING_OK)
        return status;

    auto *curr_descriptor = (descr_t *)getHeadOfDescriptors();
    auto headmem_addr     = reinterpret_cast<std::uintptr_t>(getHeadOfDescriptors());

    uint32_t _buffer_addr 	   = buffer_addr;
    uint32_t next_address_phys = chain_phys_baseaddr;
    uint32_t tailmem_addr      = getTailMem(headmem_addr, bd_count);

    if (!isRx)
        setSof(curr_descriptor);

    for (auto descrIndex = 0; descrIndex < (bd_count - 1); descrIndex++) {
        next_address_phys = getNextAddress(next_address_phys);

        setNextDescrAddr(curr_descriptor, next_address_phys);
        setBufferAddr   (curr_descriptor, buffer_addr);
        setStatus       (curr_descriptor, 0x00);
        setLength       (curr_descriptor, bytes_per_desc);
        setHeadMem      (curr_descriptor, headmem_addr);
        setTailMem      (curr_descriptor, tailmem_addr);
        setAlignMem     (curr_descriptor, BD_MIN_ALIGNMENT); // ??

        _buffer_addr    += bytes_per_desc;
        curr_descriptor = (descr_t *)getNextAddress(curr_descriptor);
    }

    setNextDescrAddr(curr_descriptor, chain_phys_baseaddr);
    setBufferAddr   (curr_descriptor, _buffer_addr);
    setStatus       (curr_descriptor, 0x00);
    setLength       (curr_descriptor, (remainder_size == 0) ? bytes_per_desc
                                                            : remainder_size);
    setHeadMem      (curr_descriptor, headmem_addr);     // app0
    setTailMem      (curr_descriptor, tailmem_addr);     // app1
    setAlignMem     (curr_descriptor, BD_MIN_ALIGNMENT); // app2

    if (!isRx)
        setEof(curr_descriptor);

    return RING_OK;
}


/**
 * @brief Process descriptor chain. Calculate transmitted data
 * @param none
 *
 * @return size - count of transmitted bytes
 */
size_t AxiDmaDescriptors::ProcessDescriptors() {
    int proc_descrs = countProcessedDescs();

    size_t size = 0;
    auto *curr_descriptor = (descr_t *)getHeadOfDescriptors();
    for (auto proc_descr_cnt = 0; proc_descr_cnt < proc_descrs; proc_descr_cnt++) {
        if (isSof(curr_descriptor))
            size = getTransferredLen(curr_descriptor);
        else if (isIof(curr_descriptor))
            size += getTransferredLen(curr_descriptor);
        else if (isEof(curr_descriptor) && !isSof(curr_descriptor))
            size += getTransferredLen(curr_descriptor);

        freeDescriptor(curr_descriptor);
        curr_descriptor = (descr_t *)getNextAddress(curr_descriptor);
    }

    return size;
}


/**
 * @brief Prepare chain of descriptors
 * @param[in] buffer_size - the size of buffer for transmit
 *
 * @return RING_OK - chain of descriptors is ready
 * @return -ERR_BDCNT   - can't calculate count of descriptors
 * @return -ERR_SIZE    - pass not aligned size of data
 * @return -ERR_OPEN_FD - can't get access to /dev/mem
 * @return -ERR_MAP     - can't mmap to SG descriptor
 */
int AxiDmaDescriptors::prepareChain(size_t buffer_size) {
    int bd_cnt = calcDescrCount(bytes_per_desc, buffer_size);
    int status = setDescrCount(bd_cnt);
    if (status != RING_OK)
        return status;

    uint32_t desc_space_size = bd_count * BD_MIN_ALIGNMENT;
    status = setChainSize(desc_space_size);
    if (status != RING_OK)
        return status;

    status = allocDescriptorMem();
    if (status != RING_OK)
        return status;

    return RING_OK;
}


/**
 * @brief Calculate count of descriptors
 * @param[in] bytes_per_descr - count bytes for 1 descriptor
 * @param[in] buffer_size     - size of input buffer of data
 *
 * @return count - count of descriptors
 */
int AxiDmaDescriptors::calcDescrCount(size_t bytes_per_descr, size_t buffer_size) {
    int tail_descriptor = 0;

    remainder_size = buffer_size % bytes_per_descr;
    if (remainder_size != 0)
        tail_descriptor++;

    int count = int((buffer_size / bytes_per_descr) + tail_descriptor);
    //count = (count == 0) ? 1 : count;

    return count;
}


/**
 * @brief Save count of descriptors
 * @param[in] descriptors_count - count of descriptors
 *
 * @return  RING_OK   - save was successful
 * @return -ERR_BDCNT - incorrect count of descriptors
 */
int AxiDmaDescriptors::setDescrCount(int descriptors_count) {
    if (descriptors_count < 1)
        return -ERR_BDCNT;

    bd_count = descriptors_count;
    return RING_OK;
}


/**
 * @brief Set size of descriptors chain
 * @param[in] chain_size - size of chain
 *
 * @return  RING_OK  - size set correct
 * @return -ERR_SIZE - incorrect size
 */
int AxiDmaDescriptors::setChainSize(size_t chain_size) {
    if (chain_size < BD_MIN_ALIGNMENT)
        return -ERR_SIZE;

    if (chain_size % BD_MIN_ALIGNMENT != 0)
        return -ERR_SIZE;

    _chain_size = chain_size;
    return RING_OK;
}


/**
 * @brief Allocate (memory mapped) memory for descriptors chain
 * @param none
 *
 * @return  RING_OK     - allocate was successful
 * @return -ERR_OPEN_FD - can't get access to /dev/mem
 * @return -ERR_MAP     - can't mmap to descriptor chain
 */
int AxiDmaDescriptors::allocDescriptorMem() {
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
        return -ERR_OPEN_FD;

    chain_virt_baseaddr = (uint32_t *) mmap(nullptr, _chain_size, PROT_READ | PROT_WRITE,
                                             MAP_SHARED, fd, chain_phys_baseaddr);
    if (chain_virt_baseaddr == MAP_FAILED)
        return -ERR_MAP;

    clearMemory();
    return RING_OK;
}


/**
 * @brief Clear (set 0x00) descriptors chain
 * @param none
 *
 * @return none
 */
void AxiDmaDescriptors::clearMemory() {
    auto *bdring_base = (uint32_t *)getHeadOfDescriptors();
    memset(bdring_base, 0x00, _chain_size);
}


/**
 * @brief Get pointer to base address of descriptor's chain
 * @param none
 *
 * @return pointer to descriptor's chain head
 */
void *AxiDmaDescriptors::getHeadOfDescriptors() {
    return chain_virt_baseaddr;
}


/**
 * @brief Get address of next descriptor
 * @param[in] current_address - the current address of descriptor
 *
 * @return address of next descriptor in chain
 */
uint32_t AxiDmaDescriptors::getNextAddress(uint32_t current_address) {
    return current_address + BD_MIN_ALIGNMENT;
}


/**
 * @brief Get pointer to address of next descriptor
 * @param[in] current_descriptor - current destriptor in chain
 *
 * @return pointer to the next descriptor in chain
 */
void *AxiDmaDescriptors::getNextAddress(void *current_descriptor) {
    auto current_addr = reinterpret_cast<std::uintptr_t >(current_descriptor);
    current_addr     += BD_MIN_ALIGNMENT;

    return reinterpret_cast<void *>(current_addr);
}


/**
 * @brief Return tail address of descriptors chain
 * @param[in] headmem_addr - value of head address of descriptors chain
 * @param[in] count_descr  - count of descriptors in chain
 *
 * @return tail address of descriptors chain
 */
uint32_t AxiDmaDescriptors::getTailMem(std::uintptr_t headmem_addr, int count_descr) {
    return uint32_t (headmem_addr + ((count_descr - 1) * BD_MIN_ALIGNMENT));
}


/**
 * @brief Get control (0x18) register of descriptor
 * @param[in] curr_descriptor - current descriptor
 *
 * @return value of control register
 */
uint32_t AxiDmaDescriptors::getControl(descr_t *curr_descriptor) {
    return curr_descriptor->control;
}


/**
 * @brief Get status (0x1C) register of descriptor
 * @param[in] curr_descriptor - current descriptor
 *
 * @return value of status register
 */
uint32_t AxiDmaDescriptors::getStatus(descr_t *curr_descriptor) {
    return curr_descriptor->status;
}


/**
 * @brief Write into APP0 (0x20) register head address of descriptors chain
 * @param[in] curr_descriptor - current descriptor
 * @param[in] headmem         - head address of descriptor chain
 *
 * @return none
 */
void AxiDmaDescriptors::setHeadMem(descr_t *curr_descriptor, uint32_t headmem) {
    setApp0(curr_descriptor, headmem);
}


/**
 * @brief Write into APP1 (0x24) register tail address of descriptors chain
 * @param[in] curr_descriptor - current descriptor
 * @param[in] tailmem         - tail address of descriptor chain
 *
 * @return none
 */
void AxiDmaDescriptors::setTailMem(descr_t *curr_descriptor, uint32_t tailmem) {
    setApp1(curr_descriptor, tailmem);
}


/**
 * @brief Write into APP2 (0x28) register alignment value
 * @param[in] curr_descriptor - current descriptor
 * @param[in] alignmem        - alignment value which used like step between
 *   descriptors in the chain
 *
 * @return none
 */
void AxiDmaDescriptors::setAlignMem(descr_t *curr_descriptor, uint32_t alignmem) {
    setApp2(curr_descriptor, alignmem);
}


/**
 * @brief Set address of next descriptor in nxtdesc (0x00) register
 * @param[in] curr_descriptor - current descriptor
 * @param[in] next_address    - address of the next descriptor
 *
 * @return none
 */
void AxiDmaDescriptors::setNextDescrAddr(descr_t *curr_descriptor,
                                             uint32_t next_address) {
    curr_descriptor->nextdesc = next_address;
}


/**
 * @brief Set address of next buffer in buffer_address (0x08) register
 * @param[in] curr_descriptor - current descriptor
 * @param[in] buff_address    - address of the next buffer
 *
 * @return none
 */
void AxiDmaDescriptors::setBufferAddr(descr_t *curr_descriptor,
                                          uint32_t buff_address) {
    curr_descriptor->buffer_addr = buff_address;
}


/**
 * @brief Set value into control (0x18) register
 * @param[in] curr_descriptor - current descriptor
 * @param[in] control_reg     - value for writing
 *
 * @return none
 */
void AxiDmaDescriptors::setControl(descr_t *curr_descriptor, uint32_t control_reg) {
    curr_descriptor->control = control_reg;
}


/**
 * @brief Set value into status (0x1C) register
 * @param[in] curr_descriptor - current descriptor
 * @param[in] status_reg      - value for writing
 *
 * @return none
 */
void AxiDmaDescriptors::setStatus(descr_t *curr_descriptor, uint32_t status_reg) {
    curr_descriptor->status = status_reg;
}


/**
 * @brief Set length of transferring data into control (0x18) register
 * @param[in] curr_descriptor - current descriptor
 * @param[in] length          - value for writing
 *
 * @return  RING_OK       - length was set
 * @return -ERR_BDMAX_LEN - incorrect length of data
 */
int AxiDmaDescriptors::setLength(descr_t *curr_descriptor, size_t length) {
    if (length > BD_MAX_LEN || length == 0)
        return -ERR_BDMAX_LEN;

    uint32_t control_reg = getControl(curr_descriptor) & ~BD_MAX_LEN;
    control_reg |= length;
    setControl(curr_descriptor, control_reg);

    return RING_OK;
}


/**
 * @brief Write data into APP0 (0x20) register
 * @param[in] curr_descr - current descriptor
 * @param[in] data       - data for writing
 *
 * @return none
 */
void AxiDmaDescriptors::setApp0(descr_t *curr_descr, uint32_t data) {
    curr_descr->app_0 = data;
}


/**
 * @brief Write data into APP1 (0x24) register
 * @param[in] curr_descr - current descriptor
 * @param[in] data       - data for writing
 *
 * @return none
 */
void AxiDmaDescriptors::setApp1(descr_t *curr_descr, uint32_t data) {
    curr_descr->app_1 = data;
}


/**
 * @brief Write data into APP2 (0x28) register
 * @param[in] curr_descr - current descriptor
 * @param[in] data       - data for writing
 *
 * @return none
 */
void AxiDmaDescriptors::setApp2(descr_t *curr_descr, uint32_t data) {
    curr_descr->app_2 = data;
}


/**
 * @brief Set bit of Start of Frame (SoF) in control register
 * @param[in] curr_descr - current descriptor
 *
 * @return none
 */
void AxiDmaDescriptors::setSof(descr_t *curr_descr) {
    curr_descr->control |= BD_STATUS_SOF;
}


/**
 * @brief Set bit of End of Frame (EoF) in control register
 * @param[in] curr_descr - current descriptor
 *
 * @return none
 */
void AxiDmaDescriptors::setEof(descr_t *curr_descr) {
    curr_descr->control |= BD_STATUS_EOF;
}


/**
 * @brief Get length of tranferred data
 * @param[in] curr_descr - current descriptor
 *
 * @return length of transferred data
 */
uint32_t AxiDmaDescriptors::getTransferredLen(descr_t *curr_descr) {
    return (getStatus(curr_descr) & BD_MAX_LEN);
}


/**
 * @brief Get complete bit
 * @param[in] curr_descr - current descriptor
 *
 * @return true  - if descriptor is complete
 * @return false - if descriptor not complete
 */
bool AxiDmaDescriptors::isCompleted(descr_t *curr_descr) {
    return (getStatus(curr_descr) & BD_STATUS_COMP) != 0;
}


/**
 * @brief Get SoF bit
 * @param[in] curr_descr - current descriptor
 *
 * @return true  - if descriptor is SoF
 * @return false - if descriptor not SoF
 */
bool AxiDmaDescriptors::isSof(descr_t *curr_descr) {
    return (curr_descr->status & BD_STATUS_SOF) != 0;
}


/**
 * @brief Get Intermediate of Frame (IoF) bit
 * @param[in] curr_descr - current descriptor
 *
 * @return true  - if descriptor is IoF
 * @return false - if descriptor not IoF
 * @note IoF - descriptor which not Start and End but complete
 */
bool AxiDmaDescriptors::isIof(descr_t *curr_descr) {
    return (isCompleted(curr_descr) && !isSof(curr_descr) && !isEof(curr_descr));
}


/**
 * @brief Get EoF bit
 * @param[in] curr_descr - current descriptor
 *
 * @return true  - if descriptor is EoF
 * @return false - if descriptor not EoF
 */
bool AxiDmaDescriptors::isEof(descr_t *curr_descr) {
    return (curr_descr->status & BD_STATUS_EOF) != 0;
}


/**
 * @brief Free descriptor (doesn't free memory)
 * @param[in] curr_descr - current descriptor
 *
 * @return none
 */
void AxiDmaDescriptors::freeDescriptor(descr_t *curr_descr) {
    curr_descr->control = 0;
    curr_descr->status  = 0;
}


/**
 * @brief Count completed descriptors
 * @param none
 *
 * @return count of processed (completed) descriptors
 */
int AxiDmaDescriptors::countProcessedDescs() {
    int processed_descrs  = 0;
    auto *curr_descriptor = (descr_t *)getHeadOfDescriptors();
    while(isCompleted(curr_descriptor)) {
        processed_descrs++;
        curr_descriptor = (descr_t *)getNextAddress(curr_descriptor);
    }

    return processed_descrs;
}


/**
 * @brief Show values of descriptors in the chain
 * @param none
 *
 * @return Used for debug
 */
void AxiDmaDescriptors::DebugDescs() {
    auto *curr_desc = (descr_t *)getHeadOfDescriptors();
    for (auto i = 0; i < bd_count; i++) {
        printf("%d\tNEXT DESC: 0x%08x\r\n", i, curr_desc->nextdesc);
        printf("%d\tBUFF ADDR: 0x%08x\r\n", i, curr_desc->buffer_addr);
        printf("%d\tCTRL REGR: 0x%08x\r\n", i, curr_desc->control);
        printf("%d\tSTTS REGR: 0x%08x\r\n", i, curr_desc->status);
        printf("%d\tAPP0 FLD:  0x%08x\r\n", i, curr_desc->app_0);
        printf("%d\tAPP1 FLD:  0x%08x\r\n", i, curr_desc->app_1);
        printf("%d\tAPP2 FLD:  0x%08x\r\n", i, curr_desc->app_2);
        printf("\r\n");
        curr_desc = (descr_t *)getNextAddress(curr_desc);
    }
}