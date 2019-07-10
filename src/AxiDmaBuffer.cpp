//
// Created by vldmr on 25.06.19.
//

#include "AxiDmaBuffer.h"


AxiDmaBuffer::AxiDmaBuffer() noexcept {
    _data.clear();
}


AxiDmaBuffer::~AxiDmaBuffer() {
    _data.clear();
}


/**
 * @brief Add @value to AxiDmaBuffer container
 * @param[in] value - value for adding
 *
 * @return none
 */
void AxiDmaBuffer::PushBack(uint8_t value) {
    _data.push_back(value);
}


/**
 * @brief Get last value from AxiDmaBuffer container and remove it
 * @param none
 *
 * @return val - last element of AxiDmaBuffer container
 */
uint8_t AxiDmaBuffer::PopBack() {
    auto it = _data.rbegin();
    uint8_t val = *it;
    _data.erase(_data.end()-1);

    return val;
}


/**
 * @brief Copy (add back) from container @src into AxiDmaBuffer
 * @param[in] src - source array
 * @param[in] len - lenth of src (in bytes)
 *
 * @return none
 */
void AxiDmaBuffer::CopyFrom(uint8_t *src, size_t len) {
    if (src == nullptr) throw std::runtime_error("src is nullptr");
    if (len == 0)       throw std::runtime_error("len is 0");
    std::copy(src, src + len, std::back_inserter(_data));
}


/**
 * @brief Copy into container @dst from AxiDmaBuffer
 * @param[out] dst - destination array
 *
 * @return none
 * @note @dst must have enough memory for copy
 */
void AxiDmaBuffer::CopyInto(uint8_t *dst) const {
    if (dst == nullptr) throw std::runtime_error("dst is nullptr");
    std::copy(_data.begin(), _data.end(), dst);
}


/**
 * @brief Retun size of AxiDmaBuffer container
 * @param none
 *
 * @return size of container
 */
size_t AxiDmaBuffer::GetSize() const {
    return _data.size();
}


/**
 * @brief Return pointer to data in container
 * @param none
 *
 * @return pointer to data
 */
const uint8_t *AxiDmaBuffer::ToArray() {
    return _data.data();
}


/**
 * @brief Return element of container by index
 * @param[in] index - index of element
 *
 * @return element from AxiDmaBuffer container
 */
inline uint8_t AxiDmaBuffer::At(size_t index) const {
    return _data.at(index);
}


/**
 * @brief Clear AxiDmaBuffer container
 * @param none
 *
 * @return none
 */
void AxiDmaBuffer::Clear() {
    _data.clear();
}