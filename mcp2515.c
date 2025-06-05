#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcp2515.h"
#include "soc/spi_periph.h"
#include <string.h>

ERROR_t MCP2515_setupSpi(MCP2515 MCP2515_Object, spi_host_device_t HOST,
                         gpio_num_t MISO, gpio_num_t MOSI, gpio_num_t CLK) {

  static bool bus_initialized[SOC_SPI_PERIPH_NUM] = {false};

  if (!bus_initialized[HOST]) {
    spi_bus_config_t bus_cfg = {
        .miso_io_num = MISO,
        .mosi_io_num = MOSI,
        .sclk_io_num = CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    esp_err_t ret = spi_bus_initialize(HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
      return ret;
    bus_initialized[HOST] = true;
  }

  spi_device_interface_config_t dev_cfg = {.mode = 0,
                                           .clock_speed_hz = SPI_CLOCK,
                                           .spics_io_num =
                                               MCP2515_Object->cs_pin,
                                           .queue_size = 4};

  return spi_bus_add_device(HOST, &dev_cfg, &MCP2515_Object->spi);
}

MCP2515 MCP2515_init(gpio_num_t cs_pin, gpio_num_t int_pin) {

  // MEMORY ALLOCATIONS FOR MCP2515 STRUCTURE
  MCP2515 MCP2515_Object = (MCP2515)malloc(sizeof(MCP2515_t[1]));
  if (MCP2515_Object == NULL) {
    ESP_LOGE(TAG_MCP2515, "Couldn't initialize MCP2515_Object. (NULL pointer)");
    return NULL;
  }
  MCP2515_Object->TXB_ptr = NULL;
  MCP2515_Object->RXB_ptr = NULL;
  MCP2515_Object->TXB_ptr = (TXBn_REGS)malloc(sizeof(TXBn_REGS_t[N_TXBUFFERS]));
  MCP2515_Object->RXB_ptr = (RXBn_REGS)malloc(sizeof(RXBn_REGS_t[N_RXBUFFERS]));
  if (MCP2515_Object->TXB_ptr == NULL || MCP2515_Object->RXB_ptr == NULL) {
    ESP_LOGE(TAG_MCP2515, "Couldn't initialize MCP2515_Object->(TXB_ptr || "
                          "RXB_ptr). (NULL pointer)");
    return NULL;
  }

  // SPI INITIALIZATION
  MCP2515_Object->cs_pin = cs_pin;
  MCP2515_Object->int_pin = int_pin;

  // TXBn and RXBn REGISTER INITIALIZATION
  MCP2515_Object->TXB_ptr[0].CTRL = MCP_TXB0CTRL;
  MCP2515_Object->TXB_ptr[0].DATA = MCP_TXB0DATA;
  MCP2515_Object->TXB_ptr[0].SIDH = MCP_TXB0SIDH;

  MCP2515_Object->TXB_ptr[1].CTRL = MCP_TXB1CTRL;
  MCP2515_Object->TXB_ptr[1].DATA = MCP_TXB1DATA;
  MCP2515_Object->TXB_ptr[1].SIDH = MCP_TXB1SIDH;

  MCP2515_Object->TXB_ptr[2].CTRL = MCP_TXB2CTRL;
  MCP2515_Object->TXB_ptr[2].DATA = MCP_TXB2DATA;
  MCP2515_Object->TXB_ptr[2].SIDH = MCP_TXB2SIDH;

  MCP2515_Object->RXB_ptr[0].CTRL = MCP_RXB0CTRL;
  MCP2515_Object->RXB_ptr[0].DATA = MCP_RXB0DATA;
  MCP2515_Object->RXB_ptr[0].SIDH = MCP_RXB0SIDH;
  MCP2515_Object->RXB_ptr[0].CANINTF_RXnIF = CANINTF_RX0IF;

  MCP2515_Object->RXB_ptr[1].CTRL = MCP_RXB1CTRL;
  MCP2515_Object->RXB_ptr[1].DATA = MCP_RXB1DATA;
  MCP2515_Object->RXB_ptr[1].SIDH = MCP_RXB1SIDH;
  MCP2515_Object->RXB_ptr[1].CANINTF_RXnIF = CANINTF_RX1IF;

  MCP2515_Object->ERROR = ERROR_OK;

  return MCP2515_Object;
}

ERROR_t MCP2515_reset(MCP2515 MCP2515_Object) {
  // startSPI();
  // SPI.transfer(INSTRUCTION_RESET);
  // endSPI();

  spi_transaction_t trans = {};

  trans.length = 8;
  trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  trans.tx_data[0] = INSTRUCTION_RESET;

  esp_err_t ret = spi_device_transmit(MCP2515_Object->spi, &trans);

  if (ret != ESP_OK) {
    printf("spi_device_transmit failed\n");
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);

  uint8_t zeros[14];
  memset(zeros, 0, sizeof(zeros));
  MCP2515_setRegisters(MCP2515_Object, MCP_TXB0CTRL, zeros, 14);
  MCP2515_setRegisters(MCP2515_Object, MCP_TXB1CTRL, zeros, 14);
  MCP2515_setRegisters(MCP2515_Object, MCP_TXB2CTRL, zeros, 14);

  MCP2515_setRegister(MCP2515_Object, MCP_RXB0CTRL, 0);
  MCP2515_setRegister(MCP2515_Object, MCP_RXB1CTRL, 0);

  MCP2515_setRegister(MCP2515_Object, MCP_CANINTE,
                      CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF |
                          CANINTF_MERRF);

  // receives all valid messages using either Standard or Extended Identifiers
  // that meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for
  // RXB1
  MCP2515_modifyRegister(MCP2515_Object, MCP_RXB0CTRL,
                         RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT |
                             RXB0CTRL_FILHIT_MASK,
                         RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
  MCP2515_modifyRegister(MCP2515_Object, MCP_RXB1CTRL,
                         RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                         RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

  // clear filters and masks
  // do not filter any standard frames for RXF0 used by RXB0
  // do not filter any extended frames for RXF1 used by RXB1
  const RXF_t filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
  for (int i = 0; i < 6; i++) {
    const bool ext = (i == 1);
    ERROR_t result = MCP2515_setFilter(MCP2515_Object, filters[i], ext, 0);
    if (result != ERROR_OK) {
      return result;
    }
  }

  MASK_t masks[] = {MASK0, MASK1};
  for (int i = 0; i < 2; i++) {
    ERROR_t result = MCP2515_setFilterMask(MCP2515_Object, masks[i], true, 0);
    if (result != ERROR_OK) {
      return result;
    }
  }

  return ERROR_OK;
}

uint8_t MCP2515_readRegister(MCP2515 MCP2515_Object, const REGISTER_t reg) {
  // startSPI();
  // SPI.transfer(INSTRUCTION_READ);
  // SPI.transfer(reg);
  // uint8_t ret = SPI.transfer(0x00);
  // endSPI();
  //
  // return ret;

  spi_transaction_t trans = {};

  trans.length = 24;
  trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  trans.tx_data[0] = INSTRUCTION_READ;
  trans.tx_data[1] = reg;
  trans.tx_data[2] = 0x00;

  esp_err_t ret = spi_device_transmit(MCP2515_Object->spi, &trans);
  if (ret != ESP_OK) {
    printf("spi_device_transmit failed\n");
  }

  return trans.rx_data[2];
}

void MCP2515_readRegisters(MCP2515 MCP2515_Object, const REGISTER_t reg,
                           uint8_t values[], const uint8_t n) {
  // startSPI();
  // SPI.transfer(INSTRUCTION_READ);
  // SPI.transfer(reg);
  // // mcp2515 has auto-increment of address-pointer
  // for (uint8_t i=0; i<n; i++) {
  //     values[i] = SPI.transfer(0x00);
  // }
  // endSPI();

  uint8_t rx_data[n + 2];
  uint8_t tx_data[n + 2];

  tx_data[0] = INSTRUCTION_READ;
  tx_data[1] = reg;

  spi_transaction_t trans = {};

  trans.length = ((2 + ((size_t)n)) * 8);
  trans.rx_buffer = rx_data;
  trans.tx_buffer = tx_data;

  esp_err_t ret = spi_device_transmit(MCP2515_Object->spi, &trans);
  if (ret != ESP_OK) {
    printf("spi_device_transmit failed\n");
  }

  for (uint8_t i = 0; i < n; i++) {
    values[i] = rx_data[i + 2];
  }
}

void MCP2515_setRegister(MCP2515 MCP2515_Object, const REGISTER_t reg,
                         const uint8_t value) {
  // startSPI();
  // SPI.transfer(INSTRUCTION_WRITE);
  // SPI.transfer(reg);
  // SPI.transfer(value);
  // endSPI();

  spi_transaction_t trans = {};
  trans.length = 24;
  trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  trans.tx_data[0] = INSTRUCTION_WRITE;
  trans.tx_data[1] = reg;
  trans.tx_data[2] = value;

  esp_err_t ret = spi_device_transmit(MCP2515_Object->spi, &trans);
  if (ret != ESP_OK) {
    printf("spi_device_transmit failed\n");
  }
}

void MCP2515_setRegisters(MCP2515 MCP2515_Object, const REGISTER_t reg,
                          const uint8_t values[], const uint8_t n) {
  // startSPI();
  // SPI.transfer(INSTRUCTION_WRITE);
  // SPI.transfer(reg);
  // for (uint8_t i=0; i<n; i++) {
  //     SPI.transfer(values[i]);
  // }
  // endSPI();

  uint8_t data[n + 2];

  data[0] = INSTRUCTION_WRITE;
  data[1] = reg;

  for (uint8_t i = 0; i < n; i++) {
    data[i + 2] = values[i];
  }

  spi_transaction_t trans = {};

  trans.length = ((2 + ((size_t)n)) * 8);
  trans.tx_buffer = data;

  esp_err_t ret = spi_device_transmit(MCP2515_Object->spi, &trans);
  if (ret != ESP_OK) {
    printf("spi_device_transmit failed\n");
  }
}

void MCP2515_modifyRegister(MCP2515 MCP2515_Object, const REGISTER_t reg,
                            const uint8_t mask, const uint8_t data) {
  // startSPI();
  // SPI.transfer(INSTRUCTION_BITMOD);
  // SPI.transfer(reg);
  // SPI.transfer(mask);
  // SPI.transfer(data);
  // endSPI();

  spi_transaction_t trans = {};

  trans.length = 32;
  trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  trans.tx_data[0] = INSTRUCTION_BITMOD;
  trans.tx_data[1] = reg;
  trans.tx_data[2] = mask;
  trans.tx_data[3] = data;

  esp_err_t ret = spi_device_transmit(MCP2515_Object->spi, &trans);
  if (ret != ESP_OK) {
    printf("spi_device_transmit failed\n");
  }
}

uint8_t MCP2515_getStatus(MCP2515 MCP2515_Object) {
  // startSPI();
  // SPI.transfer(INSTRUCTION_READ_STATUS);
  // uint8_t i = SPI.transfer(0x00);
  // endSPI();
  //
  // return i;

  spi_transaction_t trans = {};

  trans.length = 16;
  trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  trans.tx_data[0] = INSTRUCTION_READ_STATUS;
  trans.tx_data[1] = 0x00;

  esp_err_t ret = spi_device_transmit(MCP2515_Object->spi, &trans);
  if (ret != ESP_OK) {
    printf("spi_device_transmit failed\n");
  }

  return trans.rx_data[1];
}

ERROR_t MCP2515_setConfigMode(MCP2515 MCP2515_Object) {
  return MCP2515_setMode(MCP2515_Object, CANCTRL_REQOP_CONFIG);
}

ERROR_t MCP2515_setListenOnlyMode(MCP2515 MCP2515_Object) {
  return MCP2515_setMode(MCP2515_Object, CANCTRL_REQOP_LISTENONLY);
}

ERROR_t MCP2515_setSleepMode(MCP2515 MCP2515_Object) {
  return MCP2515_setMode(MCP2515_Object, CANCTRL_REQOP_SLEEP);
}

ERROR_t MCP2515_setLoopbackMode(MCP2515 MCP2515_Object) {
  return MCP2515_setMode(MCP2515_Object, CANCTRL_REQOP_LOOPBACK);
}

ERROR_t MCP2515_setOneShotMode(MCP2515 MCP2515_Object, bool set) {
  uint8_t data = 0;
  if (set)
    data = 1U << 3;
  MCP2515_modifyRegister(MCP2515_Object, MCP_CANCTRL, 1U << 3, data);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  bool modeMatch = false;
  for (int i = 0; i < 10; i++) {
    uint8_t ctrlR = MCP2515_readRegister(MCP2515_Object, MCP_CANCTRL);
    modeMatch = (ctrlR & (1U << 3)) == data;
    if (modeMatch)
      break;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  return modeMatch ? ERROR_OK : ERROR_FAIL;
}

ERROR_t MCP2515_setNormalMode(MCP2515 MCP2515_Object) {
  return MCP2515_setMode(MCP2515_Object, CANCTRL_REQOP_NORMAL);
}

ERROR_t MCP2515_setMode(MCP2515 MCP2515_Object,
                        const CANCTRL_REQOP_MODE_t mode) {
  MCP2515_modifyRegister(MCP2515_Object, MCP_CANCTRL, CANCTRL_REQOP, mode);

  bool modeMatch = false;

  for (int i = 0; i < 10; i++) {
    uint8_t newmode = MCP2515_readRegister(MCP2515_Object, MCP_CANSTAT);
    newmode &= CANSTAT_OPMOD;

    modeMatch = newmode == mode;

    if (modeMatch) {
      break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  return modeMatch ? ERROR_OK : ERROR_FAIL;
}

ERROR_t MCP2515_setBitrate(MCP2515 MCP2515_Object, const CAN_SPEED_t canSpeed,
                           CAN_CLOCK_t canClock) {

  ERROR_t ERROR_t = MCP2515_setConfigMode(MCP2515_Object);
  if (ERROR_t != ERROR_OK) {
    return ERROR_FAIL;
  }

  uint8_t set, cfg1, cfg2, cfg3;
  set = 1;
  switch (canClock) {
  case (MCP_8MHZ):
    switch (canSpeed) {
    case (CAN_5KBPS): //   5KBPS
      cfg1 = MCP_8MHz_5kBPS_CFG1;
      cfg2 = MCP_8MHz_5kBPS_CFG2;
      cfg3 = MCP_8MHz_5kBPS_CFG3;
      break;

    case (CAN_10KBPS): //  10KBPS
      cfg1 = MCP_8MHz_10kBPS_CFG1;
      cfg2 = MCP_8MHz_10kBPS_CFG2;
      cfg3 = MCP_8MHz_10kBPS_CFG3;
      break;

    case (CAN_20KBPS): //  20KBPS
      cfg1 = MCP_8MHz_20kBPS_CFG1;
      cfg2 = MCP_8MHz_20kBPS_CFG2;
      cfg3 = MCP_8MHz_20kBPS_CFG3;
      break;

    case (CAN_31K25BPS): //  31.25KBPS
      cfg1 = MCP_8MHz_31k25BPS_CFG1;
      cfg2 = MCP_8MHz_31k25BPS_CFG2;
      cfg3 = MCP_8MHz_31k25BPS_CFG3;
      break;

    case (CAN_33KBPS): //  33.333KBPS
      cfg1 = MCP_8MHz_33k3BPS_CFG1;
      cfg2 = MCP_8MHz_33k3BPS_CFG2;
      cfg3 = MCP_8MHz_33k3BPS_CFG3;
      break;

    case (CAN_40KBPS): //  40Kbps
      cfg1 = MCP_8MHz_40kBPS_CFG1;
      cfg2 = MCP_8MHz_40kBPS_CFG2;
      cfg3 = MCP_8MHz_40kBPS_CFG3;
      break;

    case (CAN_50KBPS): //  50Kbps
      cfg1 = MCP_8MHz_50kBPS_CFG1;
      cfg2 = MCP_8MHz_50kBPS_CFG2;
      cfg3 = MCP_8MHz_50kBPS_CFG3;
      break;

    case (CAN_80KBPS): //  80Kbps
      cfg1 = MCP_8MHz_80kBPS_CFG1;
      cfg2 = MCP_8MHz_80kBPS_CFG2;
      cfg3 = MCP_8MHz_80kBPS_CFG3;
      break;

    case (CAN_100KBPS): // 100Kbps
      cfg1 = MCP_8MHz_100kBPS_CFG1;
      cfg2 = MCP_8MHz_100kBPS_CFG2;
      cfg3 = MCP_8MHz_100kBPS_CFG3;
      break;

    case (CAN_125KBPS): // 125Kbps
      cfg1 = MCP_8MHz_125kBPS_CFG1;
      cfg2 = MCP_8MHz_125kBPS_CFG2;
      cfg3 = MCP_8MHz_125kBPS_CFG3;
      break;

    case (CAN_200KBPS): // 200Kbps
      cfg1 = MCP_8MHz_200kBPS_CFG1;
      cfg2 = MCP_8MHz_200kBPS_CFG2;
      cfg3 = MCP_8MHz_200kBPS_CFG3;
      break;

    case (CAN_250KBPS): // 250Kbps
      cfg1 = MCP_8MHz_250kBPS_CFG1;
      cfg2 = MCP_8MHz_250kBPS_CFG2;
      cfg3 = MCP_8MHz_250kBPS_CFG3;
      break;

    case (CAN_500KBPS): // 500Kbps
      cfg1 = MCP_8MHz_500kBPS_CFG1;
      cfg2 = MCP_8MHz_500kBPS_CFG2;
      cfg3 = MCP_8MHz_500kBPS_CFG3;
      break;

    case (CAN_1000KBPS): //   1Mbps
      cfg1 = MCP_8MHz_1000kBPS_CFG1;
      cfg2 = MCP_8MHz_1000kBPS_CFG2;
      cfg3 = MCP_8MHz_1000kBPS_CFG3;
      break;

    default:
      set = 0;
      break;
    }
    break;

  case (MCP_16MHZ):
    switch (canSpeed) {
    case (CAN_5KBPS): //   5Kbps
      cfg1 = MCP_16MHz_5kBPS_CFG1;
      cfg2 = MCP_16MHz_5kBPS_CFG2;
      cfg3 = MCP_16MHz_5kBPS_CFG3;
      break;

    case (CAN_10KBPS): //  10Kbps
      cfg1 = MCP_16MHz_10kBPS_CFG1;
      cfg2 = MCP_16MHz_10kBPS_CFG2;
      cfg3 = MCP_16MHz_10kBPS_CFG3;
      break;

    case (CAN_20KBPS): //  20Kbps
      cfg1 = MCP_16MHz_20kBPS_CFG1;
      cfg2 = MCP_16MHz_20kBPS_CFG2;
      cfg3 = MCP_16MHz_20kBPS_CFG3;
      break;

    case (CAN_33KBPS): //  33.333Kbps
      cfg1 = MCP_16MHz_33k3BPS_CFG1;
      cfg2 = MCP_16MHz_33k3BPS_CFG2;
      cfg3 = MCP_16MHz_33k3BPS_CFG3;
      break;

    case (CAN_40KBPS): //  40Kbps
      cfg1 = MCP_16MHz_40kBPS_CFG1;
      cfg2 = MCP_16MHz_40kBPS_CFG2;
      cfg3 = MCP_16MHz_40kBPS_CFG3;
      break;

    case (CAN_50KBPS): //  50Kbps
      cfg1 = MCP_16MHz_50kBPS_CFG1;
      cfg2 = MCP_16MHz_50kBPS_CFG2;
      cfg3 = MCP_16MHz_50kBPS_CFG3;
      break;

    case (CAN_80KBPS): //  80Kbps
      cfg1 = MCP_16MHz_80kBPS_CFG1;
      cfg2 = MCP_16MHz_80kBPS_CFG2;
      cfg3 = MCP_16MHz_80kBPS_CFG3;
      break;

    case (CAN_83K3BPS): //  83.333Kbps
      cfg1 = MCP_16MHz_83k3BPS_CFG1;
      cfg2 = MCP_16MHz_83k3BPS_CFG2;
      cfg3 = MCP_16MHz_83k3BPS_CFG3;
      break;

    case (CAN_100KBPS): // 100Kbps
      cfg1 = MCP_16MHz_100kBPS_CFG1;
      cfg2 = MCP_16MHz_100kBPS_CFG2;
      cfg3 = MCP_16MHz_100kBPS_CFG3;
      break;

    case (CAN_125KBPS): // 125Kbps
      cfg1 = MCP_16MHz_125kBPS_CFG1;
      cfg2 = MCP_16MHz_125kBPS_CFG2;
      cfg3 = MCP_16MHz_125kBPS_CFG3;
      break;

    case (CAN_200KBPS): // 200Kbps
      cfg1 = MCP_16MHz_200kBPS_CFG1;
      cfg2 = MCP_16MHz_200kBPS_CFG2;
      cfg3 = MCP_16MHz_200kBPS_CFG3;
      break;

    case (CAN_250KBPS): // 250Kbps
      cfg1 = MCP_16MHz_250kBPS_CFG1;
      cfg2 = MCP_16MHz_250kBPS_CFG2;
      cfg3 = MCP_16MHz_250kBPS_CFG3;
      break;

    case (CAN_500KBPS): // 500Kbps
      cfg1 = MCP_16MHz_500kBPS_CFG1;
      cfg2 = MCP_16MHz_500kBPS_CFG2;
      cfg3 = MCP_16MHz_500kBPS_CFG3;
      break;

    case (CAN_1000KBPS): //   1Mbps
      cfg1 = MCP_16MHz_1000kBPS_CFG1;
      cfg2 = MCP_16MHz_1000kBPS_CFG2;
      cfg3 = MCP_16MHz_1000kBPS_CFG3;
      break;

    default:
      set = 0;
      break;
    }
    break;

  case (MCP_20MHZ):
    switch (canSpeed) {
    case (CAN_33KBPS): //  33.333Kbps
      cfg1 = MCP_20MHz_33k3BPS_CFG1;
      cfg2 = MCP_20MHz_33k3BPS_CFG2;
      cfg3 = MCP_20MHz_33k3BPS_CFG3;
      break;

    case (CAN_40KBPS): //  40Kbps
      cfg1 = MCP_20MHz_40kBPS_CFG1;
      cfg2 = MCP_20MHz_40kBPS_CFG2;
      cfg3 = MCP_20MHz_40kBPS_CFG3;
      break;

    case (CAN_50KBPS): //  50Kbps
      cfg1 = MCP_20MHz_50kBPS_CFG1;
      cfg2 = MCP_20MHz_50kBPS_CFG2;
      cfg3 = MCP_20MHz_50kBPS_CFG3;
      break;

    case (CAN_80KBPS): //  80Kbps
      cfg1 = MCP_20MHz_80kBPS_CFG1;
      cfg2 = MCP_20MHz_80kBPS_CFG2;
      cfg3 = MCP_20MHz_80kBPS_CFG3;
      break;

    case (CAN_83K3BPS): //  83.333Kbps
      cfg1 = MCP_20MHz_83k3BPS_CFG1;
      cfg2 = MCP_20MHz_83k3BPS_CFG2;
      cfg3 = MCP_20MHz_83k3BPS_CFG3;
      break;

    case (CAN_100KBPS): // 100Kbps
      cfg1 = MCP_20MHz_100kBPS_CFG1;
      cfg2 = MCP_20MHz_100kBPS_CFG2;
      cfg3 = MCP_20MHz_100kBPS_CFG3;
      break;

    case (CAN_125KBPS): // 125Kbps
      cfg1 = MCP_20MHz_125kBPS_CFG1;
      cfg2 = MCP_20MHz_125kBPS_CFG2;
      cfg3 = MCP_20MHz_125kBPS_CFG3;
      break;

    case (CAN_200KBPS): // 200Kbps
      cfg1 = MCP_20MHz_200kBPS_CFG1;
      cfg2 = MCP_20MHz_200kBPS_CFG2;
      cfg3 = MCP_20MHz_200kBPS_CFG3;
      break;

    case (CAN_250KBPS): // 250Kbps
      cfg1 = MCP_20MHz_250kBPS_CFG1;
      cfg2 = MCP_20MHz_250kBPS_CFG2;
      cfg3 = MCP_20MHz_250kBPS_CFG3;
      break;

    case (CAN_500KBPS): // 500Kbps
      cfg1 = MCP_20MHz_500kBPS_CFG1;
      cfg2 = MCP_20MHz_500kBPS_CFG2;
      cfg3 = MCP_20MHz_500kBPS_CFG3;
      break;

    case (CAN_1000KBPS): //   1Mbps
      cfg1 = MCP_20MHz_1000kBPS_CFG1;
      cfg2 = MCP_20MHz_1000kBPS_CFG2;
      cfg3 = MCP_20MHz_1000kBPS_CFG3;
      break;

    default:
      set = 0;
      break;
    }
    break;

  default:
    set = 0;
    break;
  }

  if (set) {
    MCP2515_setRegister(MCP2515_Object, MCP_CNF1, cfg1);
    MCP2515_setRegister(MCP2515_Object, MCP_CNF2, cfg2);
    MCP2515_setRegister(MCP2515_Object, MCP_CNF3, cfg3);
    return ERROR_OK;
  } else {
    return ERROR_FAIL;
  }
}

ERROR_t MCP2515_setClkOut(MCP2515 MCP2515_Object, const CAN_CLKOUT_t divisor) {
  if (divisor == CLKOUT_DISABLE) {
    /* Turn off CLKEN */
    MCP2515_modifyRegister(MCP2515_Object, MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

    /* Turn on CLKOUT for SOF */
    MCP2515_modifyRegister(MCP2515_Object, MCP_CNF3, CNF3_SOF, CNF3_SOF);
    return ERROR_OK;
  }

  /* Set the prescaler (CLKPRE) */
  MCP2515_modifyRegister(MCP2515_Object, MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

  /* Turn on CLKEN */
  MCP2515_modifyRegister(MCP2515_Object, MCP_CANCTRL, CANCTRL_CLKEN,
                         CANCTRL_CLKEN);

  /* Turn off CLKOUT for SOF */
  MCP2515_modifyRegister(MCP2515_Object, MCP_CNF3, CNF3_SOF, 0x00);
  return ERROR_OK;
}

void MCP2515_prepareId(uint8_t *buffer, const bool ext, const uint32_t id) {
  uint16_t canid = (uint16_t)(id & 0x0FFFF);

  if (ext) {
    buffer[MCP_EID0] = (uint8_t)(canid & 0xFF);
    buffer[MCP_EID8] = (uint8_t)(canid >> 8);
    canid = (uint16_t)(id >> 16);
    buffer[MCP_SIDL] = (uint8_t)(canid & 0x03);
    buffer[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
    buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
    buffer[MCP_SIDH] = (uint8_t)(canid >> 5);
  } else {
    buffer[MCP_SIDH] = (uint8_t)(canid >> 3);
    buffer[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
    buffer[MCP_EID0] = 0;
    buffer[MCP_EID8] = 0;
  }
}

ERROR_t MCP2515_setFilterMask(MCP2515 MCP2515_Object, const MASK_t mask,
                              const bool ext, const uint32_t ulData) {
  ERROR_t res = MCP2515_setConfigMode(MCP2515_Object);
  if (res != ERROR_OK) {
    return res;
  }

  uint8_t tbufdata[4];
  MCP2515_prepareId(tbufdata, ext, ulData);

  REGISTER_t reg;
  switch (mask) {
  case MASK0:
    reg = MCP_RXM0SIDH;
    break;
  case MASK1:
    reg = MCP_RXM1SIDH;
    break;
  default:
    return ERROR_FAIL;
  }

  MCP2515_setRegisters(MCP2515_Object, reg, tbufdata, 4);

  return ERROR_OK;
}

ERROR_t MCP2515_setFilter(MCP2515 MCP2515_Object, const RXF_t num,
                          const bool ext, const uint32_t ulData) {
  ERROR_t res = MCP2515_setConfigMode(MCP2515_Object);
  if (res != ERROR_OK) {
    return res;
  }

  REGISTER_t reg;

  switch (num) {
  case RXF0:
    reg = MCP_RXF0SIDH;
    break;
  case RXF1:
    reg = MCP_RXF1SIDH;
    break;
  case RXF2:
    reg = MCP_RXF2SIDH;
    break;
  case RXF3:
    reg = MCP_RXF3SIDH;
    break;
  case RXF4:
    reg = MCP_RXF4SIDH;
    break;
  case RXF5:
    reg = MCP_RXF5SIDH;
    break;
  default:
    return ERROR_FAIL;
  }

  uint8_t tbufdata[4];
  MCP2515_prepareId(tbufdata, ext, ulData);
  MCP2515_setRegisters(MCP2515_Object, reg, tbufdata, 4);

  return ERROR_OK;
}

ERROR_t MCP2515_sendMessage(MCP2515 MCP2515_Object, const TXBn_t txbn,
                            const CAN_FRAME frame) {
  if (frame->can_dlc > CAN_MAX_DLEN) {
    return ERROR_FAILTX;
  }

  const TXBn_REGS txbuf = &MCP2515_Object->TXB_ptr[txbn];

  uint8_t data[13];

  bool ext = (frame->can_id & CAN_EFF_FLAG);
  bool rtr = (frame->can_id & CAN_RTR_FLAG);
  uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

  MCP2515_prepareId(data, ext, id);

  data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

  memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

  MCP2515_setRegisters(MCP2515_Object, txbuf->SIDH, data, 5 + frame->can_dlc);

  MCP2515_modifyRegister(MCP2515_Object, txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

  uint8_t ctrl = MCP2515_readRegister(MCP2515_Object, txbuf->CTRL);
  if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) {
    return ERROR_FAILTX;
  }
  return ERROR_OK;
}

ERROR_t MCP2515_sendMessageAfterCtrlCheck(MCP2515 MCP2515_Object,
                                          const CAN_FRAME frame) {
  if (frame->can_dlc > CAN_MAX_DLEN) {
    return ERROR_FAILTX;
  }

  TXBn_t txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};

  for (int i = 0; i < N_TXBUFFERS; i++) {
    const TXBn_REGS txbuf = &MCP2515_Object->TXB_ptr[txBuffers[i]];
    uint8_t ctrlval = MCP2515_readRegister(MCP2515_Object, txbuf->CTRL);
    if ((ctrlval & TXB_TXREQ) == 0) {
      return MCP2515_sendMessage(MCP2515_Object, txBuffers[i], frame);
    }
  }

  return ERROR_ALLTXBUSY;
}

static int currentBuffer = 0;

ERROR_t MCP2515_sendMessageRoundRobin(MCP2515 can, const CAN_FRAME frame) {
  for (int retries = 0; retries < 50; retries++) {
    for (int offset = 0; offset < 3; offset++) {
      int idx = (currentBuffer + offset) % 3;
      const TXBn_REGS txbuf = &can->TXB_ptr[idx];
      uint8_t ctrlval = MCP2515_readRegister(can, txbuf->CTRL);

      if ((ctrlval & TXB_TXREQ) == 0) {
        ERROR_t result = MCP2515_sendMessage(can, idx, frame);
        if (result == ERROR_OK) {
          const uint8_t txIfBits[3] = {0x04, 0x08, 0x10};
          MCP2515_modifyRegister(can, MCP_CANINTF, txIfBits[idx], 0);

          currentBuffer = (idx + 1) % 3;
        } else {
          ESP_LOGE("MCP2515", "MCP2515_sendMessage failed at TXB%d: %d", idx,
                   result);
        }
        uint8_t intf = MCP2515_readRegister(can, MCP_CANINTF);
        printf("CANINTF: 0x%02X\n", intf);

        return result;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }

  return ERROR_ALLTXBUSY;
}

ERROR_t MCP2515_readMessage(MCP2515 MCP2515_Object, const RXBn_t rxbn,
                            const CAN_FRAME frame) {
  const RXBn_REGS rxb = &MCP2515_Object->RXB_ptr[rxbn];

  uint8_t tbufdata[5];

  MCP2515_readRegisters(MCP2515_Object, rxb->SIDH, tbufdata, 5);

  uint32_t id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

  if ((tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) == TXB_EXIDE_MASK) {
    id = (id << 2) + (tbufdata[MCP_SIDL] & 0x03);
    id = (id << 8) + tbufdata[MCP_EID8];
    id = (id << 8) + tbufdata[MCP_EID0];
    id |= CAN_EFF_FLAG;
  }

  uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
  if (dlc > CAN_MAX_DLEN) {
    return ERROR_FAIL;
  }

  uint8_t ctrl = MCP2515_readRegister(MCP2515_Object, rxb->CTRL);
  if (ctrl & RXBnCTRL_RTR) {
    id |= CAN_RTR_FLAG;
  }

  frame->can_id = id;
  frame->can_dlc = dlc;

  MCP2515_readRegisters(MCP2515_Object, rxb->DATA, frame->data, dlc);

  MCP2515_modifyRegister(MCP2515_Object, MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

  return ERROR_OK;
}

ERROR_t MCP2515_readMessageAfterStatCheck(MCP2515 MCP2515_Object,
                                          const CAN_FRAME frame) {
  ERROR_t rc;
  uint8_t stat = MCP2515_getStatus(MCP2515_Object);

  if (stat & STAT_RX0IF) {
    rc = MCP2515_readMessage(MCP2515_Object, RXB0, frame);
  } else if (stat & STAT_RX1IF) {
    rc = MCP2515_readMessage(MCP2515_Object, RXB1, frame);
  } else {
    rc = ERROR_NOMSG;
  }

  return rc;
}

bool MCP2515_checkReceive(MCP2515 MCP2515_Object) {
  uint8_t res = MCP2515_getStatus(MCP2515_Object);
  if (res & STAT_RXIF_MASK) {
    return true;
  } else {
    return false;
  }
}

bool MCP2515_checkError(MCP2515 MCP2515_Object) {
  uint8_t eflg = MCP2515_getErrorFlags(MCP2515_Object);

  if (eflg & EFLG_ERRORMASK) {
    return true;
  } else {
    return false;
  }
}

uint8_t MCP2515_getErrorFlags(MCP2515 MCP2515_Object) {
  return MCP2515_readRegister(MCP2515_Object, MCP_EFLG);
}

void MCP2515_clearRXnOVRFlags(MCP2515 MCP2515_Object) {
  MCP2515_modifyRegister(MCP2515_Object, MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR,
                         0);
}

uint8_t MCP2515_getInterrupts(MCP2515 MCP2515_Object) {
  return MCP2515_readRegister(MCP2515_Object, MCP_CANINTF);
}

void MCP2515_clearInterrupts(MCP2515 MCP2515_Object) {
  MCP2515_setRegister(MCP2515_Object, MCP_CANINTF, 0);
}

uint8_t MCP2515_getInterruptMask(MCP2515 MCP2515_Object) {
  return MCP2515_readRegister(MCP2515_Object, MCP_CANINTE);
}

void MCP2515_clearTXInterrupts(MCP2515 MCP2515_Object) {
  MCP2515_modifyRegister(MCP2515_Object, MCP_CANINTF,
                         (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void MCP2515_clearRXnOVR(MCP2515 MCP2515_Object) {
  uint8_t eflg = MCP2515_getErrorFlags(MCP2515_Object);
  if (eflg != 0) {
    MCP2515_clearRXnOVRFlags(MCP2515_Object);
    MCP2515_clearInterrupts(MCP2515_Object);
    // modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
  }
}

void MCP2515_clearMERR(MCP2515 MCP2515_Object) {
  // modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
  // clearInterrupts();
  MCP2515_modifyRegister(MCP2515_Object, MCP_CANINTF, CANINTF_MERRF, 0);
}

void MCP2515_clearERRIF(MCP2515 MCP2515_Object) {
  // modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
  // clearInterrupts();
  MCP2515_modifyRegister(MCP2515_Object, MCP_CANINTF, CANINTF_ERRIF, 0);
}
