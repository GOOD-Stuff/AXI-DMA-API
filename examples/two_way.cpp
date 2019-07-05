#include <iostream>
#include <AxiDMA.h>
#include <AxiDmaBuffer.h>


/*
  Generate and send some command, receive answer and show this.
*/
int main() {
  AxiDmaBuffer tx, rx;
  AxiDMA dma;

  while(1) {
      tx.Clear();
      tx.PushBack(0x00);
      tx.PushBack(0x00);
      tx.PushBack(0x0F);
      tx.PushBack(0x00);

      int len = dma.Transf(&tx, &rx, 171);
      cout << "Recv " << len << " bytes" << endl;

      if (len > 0) {
          auto *arr = rx.ToArray();
          for (auto i = 0; i < len; i++)
              printf("%d) 0x%02x\r\n", i, arr[i]);
      }
      rx.Clear();

      if (len < 170)
          return -1;
  }

  return 0;
}
