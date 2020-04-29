//
// Created by vldmr on 25.06.19.
//

#include "AxiDmaBuffer.h"


AxiDmaBuffer::AxiDmaBuffer(const std::vector<uint8_t>& data) noexcept {
    m_data = data;
}


AxiDmaBuffer::AxiDmaBuffer(const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0)
        throw std::runtime_error("<E> Input data is empty");
    m_data.reserve(len);
    std::copy(data, data + len, std::back_inserter(m_data));
}


void AxiDmaBuffer::Append(const std::vector<uint8_t>& vec) {
    std::copy(vec.begin(), vec.end(), std::back_inserter(m_data));
}


void AxiDmaBuffer::Append(std::vector<uint8_t>&& vec) {
    m_data.insert(m_data.end(), make_move_iterator(vec.begin()), make_move_iterator(vec.end()));
}


/**
 * @brief Add @value to AxiDmaBuffer container
 * @param[in] value - value for adding
 *
 * @return none
 */
void AxiDmaBuffer::PushBack(uint8_t value) {
    m_data.emplace_back(value);
}


/**
 * @brief Get last value from AxiDmaBuffer container and remove it
 * @param none
 *
 * @return val - last element of AxiDmaBuffer container
 */
uint8_t AxiDmaBuffer::PopBack() noexcept {
    if (m_data.size() == 0)
        return 0;

    auto it = m_data.rbegin();
    uint8_t val = *it;
    m_data.erase(m_data.end()-1);

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
    std::copy(src, src + len, std::back_inserter(m_data));
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
    std::copy(m_data.begin(), m_data.end(), dst);
}


/**
 * @brief Retun size of AxiDmaBuffer container
 * @param none
 *
 * @return size of container
 */
size_t AxiDmaBuffer::GetSize() const {
    return m_data.size();
}


/**
 * @brief Return pointer to data in container
 * @param none
 *
 * @return pointer to data
 */
const uint8_t *AxiDmaBuffer::ToArray() const {
    return m_data.data();
}


/**
 * @brief Return element of container by index
 * @param[in] index - index of element
 *
 * @return element from AxiDmaBuffer container
 */
uint8_t AxiDmaBuffer::At(size_t index) const {
    return m_data.at(index);
}


/**
 * @brief Clear AxiDmaBuffer container
 * @param none
 *
 * @return none
 */
void AxiDmaBuffer::Clear() {
    m_data.clear();
}