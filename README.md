# AXI-DMA-API
Simple API for working with AXI DMA on Zynq-7000 without using driver on Linux.  
For correct work of this API you must to disable support of AXI DMA in kernel
configurations.
Like this (option `DMA Engine support` must be without asteriks `*`):

![Image](https://i.ibb.co/ZV7XDtm/ex1.png)


## Example
One of examples of transferring data between Zynq-7010 and Kintex-7.
Send command to Kintex and receive result (status) to Zynq:
```
#include <AxiDMA.h>
#include <AxiDmaBuffer.h>


int main() {
  AxiDmaBuffer *dynamic_tx_buffer = new AxiDmaBuffer();
  AxiDmaBuffer non_dynamic_rx_buffer;
  size_t statuses_len = 15; // size of waiting data

  // Set TX buffer: send command for getting statuses (i.e. from Zynq to Kintex)
  dynamic_tx_buffer->PushBack(0x01); // pnum
  dynamic_tx_buffer->PushBack(0x02); // pnum
  dynamic_tx_buffer->PushBack(0x0D); // command

  AxiDMA *dma_dev = new AxiDMA();
  int rs = dma_dev->Transf(dynamic_tx_buffer, &non_dynamic_rx_buffer, statuses_len);
  if (rs != 0) {
    delete dma_dev;
    delete dynamic_tx_buffer;
    return rs;
  }

  auto *rx_buff = new uint8_t[non_dynamic_rx_buffer.GetSize()];
  std::cout << "Get " << non_dynamic_rx_buffer.GetSize() << " bytes. So: "
            << std::endl;

  for (int i = 0; i < non_dynamic_rx_buffer.GetSize(); i++)
    printf("%d) 0x%02x\r\n", i, rx_buff[i]);

  delete rx_buff;
  delete dma_dev;
  delete dynamic_tx_buffer;
  return 0;
}
```

Other examples you could find in `examples` directory.

### Note
Work with AXI DMA in SG (scatter gather) mode.  
**API is more slowly than using driver, but you can control all transaction how you need.**  
**Used C++14.**  
Project work with Zynq-7010. Linux kernel version of 4.14. Vivado version of 2018.02.
In the branch [original](https://github.com/GOOD-Stuff/AXI-DMA-API/tree/original) contains original repository of AXI DMA API from which was started
this project.
