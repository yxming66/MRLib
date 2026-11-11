#include "board/spi_endpoints.h"
#include "bsp/spi.h"
#include "spi.h"

int Board_RegisterSpiEndpoints(void) {
  // SPI2 是本示例 IMU 总线：8 位主机、双线全双工、关闭 CRC。
  // 本板 SPI1 为单向发送，不能直接登记到当前 BSP_SPI 接口。
  const BSP_SPI_Endpoint_t endpoints[] = {{"spi2", &hspi2}};
  return BSP_SPI_Configure(endpoints, sizeof(endpoints) / sizeof(endpoints[0])) == BSP_OK;
}
