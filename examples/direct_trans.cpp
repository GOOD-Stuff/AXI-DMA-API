#include <iostream>
#include <AxiDMA.h>
#include <AxiDmaBuffer.h>

/**
* Our AXI DMA IP in Vivado design is configure in non SG mode
*/
int main() {
  AxiDMA dma;
  AxiDmaBuffer tx, rx;

  tx.PushBack(0x00);
  tx.PushBack(0x01);
  tx.PushBack(0x02);
  tx.PushBack(0x03);
  int len = dma.DirectSend(&tx);
  if (len < 0) {
    std::cerr << "Can't send data: " << len << std::endl;
    return -1;
  }
  std::cout << "Send " << len << " bytes" << std::endl;


  len = dma.DirectRecv(&rx, 171); // receive 171 bytes
  if (len < 0) {
    std::cerr << "Can't recv data: " << len << std::endl;
    return -1;
  }

  std::cout << "Receive " << len << " bytes" << std::endl;
  auto *arr = rx.ToArray();
  for (auto i = 0; i < len; i++) {
    printf("%d) 0x%02x\r\n", i, arr[i]);
  }

  return 0;
}
