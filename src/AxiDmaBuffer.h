//
// Created by vldmr on 25.06.19.
//

#ifndef AXIDMA_API_AXIDMABUFFER_H
#define AXIDMA_API_AXIDMABUFFER_H


#include <vector>
#include <cstdlib>
#include <exception>
#include <stdexcept>


/**
 * @class Used like container/wrapper for data which need to transfer via AXI DMA
 */
class AxiDmaBuffer {
public:
    explicit AxiDmaBuffer() noexcept;

    void         PushBack(uint8_t value);
    uint8_t      PopBack ();

    void         CopyFrom(uint8_t *src, size_t len);
    void         CopyInto(uint8_t *dst) const;

    size_t       GetSize  () const;
    const uint8_t *ToArray();
    uint8_t      At       (size_t index) const;
    void         Clear    ();

    virtual ~AxiDmaBuffer();
private:
    std::vector<uint8_t> _data;
};


#endif //AXIDMA_API_AXIDMABUFFER_H
