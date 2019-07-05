#include <AxiDMA.h>
#include <AxiDmaBuffer.h>


void ReadFromFile(char *, size_t len);


int main() {
  char arr[128];
  ReadFromFile(arr, sizeof(arr));

  AxiDmaBuffer tx;
  tx.CopyInto((uint8_t *)arr, sizeof(arr));

  AxiDMA dma_dev;
  if (dma_dev.Send(&tx) != 0)
    return -1;

  return 0;
}


/**
 * @brief Stumb, just for generating data
 * @param[out] array - array with data
 * @param[in]  len   - length (in bytes) of array
*/
void ReadFromFile(char *array, size_t len) {
  memset(array, 0xDE, len);
}
