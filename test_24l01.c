#include <inttypes.h>
#include <string.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"

#include "nrf24l01p.h"


/*
  nRF24L01 pinout:

  Rx:
    PF2  SCK        GND *1 2. VCC
    PF3  CSN        PB0 .3 4. PF3
    PF0  MISO       PF2 .5 6. PF1
    PF1  MOSI       PF0 .7 8. PF4
    PB0  CE
    PF4  IRQ

  Tx:
    PA2  SCK        GND *1 2. VCC
    PA3  CSN        PA6 .3 4. PA3
    PA4  MISO       PA2 .5 6. PA5
    PA5  MOSI       PA4 .7 8. PA7
    PA6  CE
    PA7  IRQ
*/


/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


static void
serial_output_hexdig(uint32_t dig)
{
  ROM_UARTCharPut(UART0_BASE, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


static void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


static void
serial_output_str(const char *str)
{
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
}


 __attribute__ ((unused))
static void
println_uint32(uint32_t val)
{
  char buf[12];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


static void
config_ssi_gpio(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  /* PF0 is special (NMI), needs unlock to be re-assigned to SSI1. */
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

  ROM_GPIOPinConfigure(GPIO_PF2_SSI1CLK);
  ROM_GPIOPinConfigure(GPIO_PF0_SSI1RX);
  ROM_GPIOPinConfigure(GPIO_PF1_SSI1TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_1);
  /* CSN pin, high initially */
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
  /* CE pin, low initially */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
  ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
  /* IRQ pin as input. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

  /* Config Tx on SSI0, PA2-PA7 */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
  ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
  /* CSN pin, high initially */
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
  /* CE pin, low initially */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
  /* IRQ pin as input. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
}


static void
config_spi(uint32_t base)
{
  /*
    Configure the SPI for correct mode to read from nRF24L01+.

    We need CLK inactive low, so SPO=0.
    We need to setup and sample on the leading, rising CLK edge, so SPH=0.

    The datasheet says up to 10MHz SPI is possible, depending on load
    capacitance. Let's go with a slightly cautious 8MHz, which should be
    aplenty.
  */

  ROM_SSIDisable(base);
  ROM_SSIConfigSetExpClk(base, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 8000000, 8);
  ROM_SSIEnable(base);
}


static uint32_t udma_control_block[256] __attribute__ ((aligned(1024)));
static void
config_udma_for_spi(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  ROM_uDMAEnable();
  ROM_uDMAControlBaseSet(udma_control_block);

  ROM_SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);
  ROM_SSIDMAEnable(SSI0_BASE, SSI_DMA_RX);
  ROM_IntEnable(INT_SSI0);

  ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(UDMA_CH11_SSI0TX);
  ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0RX, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(UDMA_CH10_SSI0RX);
  ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0RX, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_DST_INC_8 | UDMA_SRC_INC_NONE |
                            UDMA_ARB_4);
}


static void
bzero(uint8_t *buf, uint32_t len)
{
  while (len > 0)
  {
    *buf++ = 0;
    --len;
  }
}


static inline void
csn_low(uint32_t csn_base, uint32_t csn_pin)
{
  ROM_GPIOPinWrite(csn_base, csn_pin, 0);
}


static inline void
csn_high(uint32_t csn_base, uint32_t csn_pin)
{
  ROM_GPIOPinWrite(csn_base, csn_pin, csn_pin);
}


static inline void
ce_low(uint32_t ce_base, uint32_t ce_pin)
{
  ROM_GPIOPinWrite(ce_base, ce_pin, 0);
}


static inline void
ce_high(uint32_t ce_base, uint32_t ce_pin)
{
  ROM_GPIOPinWrite(ce_base, ce_pin, ce_pin);
}


static void
ssi_cmd(uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len,
        uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint32_t i;
  uint32_t data;

  /* Take CSN low to initiate transfer. */
  csn_low(csn_base, csn_pin);

  for (i = 0; i < len; ++i)
  {
    ROM_SSIDataPut(ssi_base, sendbuf[i]);
    while (ROM_SSIBusy(ssi_base))
      ;
    ROM_SSIDataGet(ssi_base, &data);
    recvbuf[i] = data;
  }

  /* Take CSN high to complete transfer. */
  csn_high(csn_base, csn_pin);

#ifdef SSI_DEBUG_OUTPUT
  /* For debug, output the data sent and received. */
  serial_output_str("Tx: ");
  for (i = 0; i < len; ++i)
  {
    serial_output_hexbyte(sendbuf[i]);
    if ((i %8) == 7)
      serial_output_str(" ");
  }
  serial_output_str("\r\nRx: ");
  for (i = 0; i < len; ++i)
  {
    serial_output_hexbyte(recvbuf[i]);
    if ((i %8) == 7)
      serial_output_str(" ");
  }
  serial_output_str("\r\n");
#endif
}


/*
  Asynchronous SSI transfer to nRF24L01+ using uDMA.
  Performs a transaction over SPI.
  First call nrf_async_start(). While nrf_async_start() or nrf_async_cont()
  returns zero, call nrf_async_cont() for each SSI interrupt event (either
  signifying DMA or transfer done).
*/
struct nrf_async_dma {
  uint32_t ssi_base;
  uint32_t dma_rx_chan;
  uint32_t dma_tx_chan;
  uint32_t csn_base;
  uint32_t csn_pin;
  uint8_t dma_rx_running;
  uint8_t dma_tx_running;
  uint8_t ssi_active;
};


static int32_t
nrf_async_dma_start(struct nrf_async_dma *a, uint8_t *recvbuf, uint8_t *sendbuf,
                    uint32_t len, uint32_t ssi_base, uint32_t dma_rx_chan,
                    uint32_t dma_tx_chan, uint32_t csn_base, uint32_t csn_pin)
{
  a->ssi_base = ssi_base;
  a->dma_rx_chan = dma_rx_chan;
  a->dma_tx_chan = dma_tx_chan;
  a->csn_base = csn_base;
  a->csn_pin = csn_pin;
  ROM_uDMAChannelTransferSet(dma_rx_chan | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                             (void *)(ssi_base + SSI_O_DR), recvbuf, len);
  ROM_uDMAChannelTransferSet(dma_tx_chan | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                             sendbuf, (void *)(ssi_base + SSI_O_DR), len);
  a->dma_rx_running = 1;
  a->dma_tx_running = 1;
  a->ssi_active = 1;
  /* Take CSN low to initiate transfer. */
  csn_low(csn_base, csn_pin);
  /* Empty any left-over stuff in the RX FIFO. */
  while (HWREG(ssi_base + SSI_O_SR) & SSI_SR_RNE)
    (void)(HWREG(ssi_base + SSI_O_DR));
  ROM_uDMAChannelEnable(dma_rx_chan);
  ROM_uDMAChannelEnable(dma_tx_chan);
  return 0;
}


static int32_t
nrf_async_dma_cont(struct nrf_async_dma *a)
{
  /*
    There are no pending interrupt requests to clear here.
    The only interrupt we handle is SSI_TXFF (TX FIFO empty), and that
    cannot be cleared (except by putting stuff in the FIFO). Rather, we
    unmask and mask it as appropriate.
  */
  if (a->dma_tx_running && !ROM_uDMAChannelIsEnabled(a->dma_tx_chan))
  {
    a->dma_tx_running = 0;
    /*
      Enable interrupt at end of transfer.

      Things did not work for me unless I delayed setting EOT to here (rather
      that doing it up-front). My guess is that setting EOT prevents not only
      the interrupt at half-full fifo, but also the dma request, causing send
      to stall, but I did not investigate fully.

      Also, let's clear the TX interrupt, if not I seemed to get a spurious
      interrupt, probably due to the fifo being half-full at this point.

      Note that then I also need to check for EOT already now in this
      interrupt activation, to avoid a race where EOT could have occured
      already due to delayed interrupt execution.
    */
    HWREG(a->ssi_base + SSI_O_CR1) |= SSI_CR1_EOT;
    ROM_SSIIntClear(a->ssi_base, SSI_TXFF);
    HWREG(a->ssi_base + SSI_O_IM) |= SSI_TXFF;
  }
  if (!ROM_SSIBusy(a->ssi_base))
  {
    a->ssi_active = 0;
    /*
      Now that the transfer is completely done, set the TX interrupt back
      to disabled and trigger at 1/2 full fifo.
    */
    HWREG(a->ssi_base + SSI_O_IM) &= ~SSI_TXFF;
    HWREG(a->ssi_base + SSI_O_CR1) &= ~SSI_CR1_EOT;
  }
  if (a->dma_rx_running && !ROM_uDMAChannelIsEnabled(a->dma_rx_chan))
    a->dma_rx_running = 0;
  if (a->dma_tx_running || a->ssi_active || a->dma_rx_running)
    return 0;
  else
  {
    /* Take CSN high to complete transfer. */
    csn_high(a->csn_base, a->csn_pin);
    return 1;
  }
}


/*
  Asynchronous SSI transfer to nRF24L01+ using only fifo (no dma).
  Performs a transaction over SPI <= 8 bytes.
  First call nrf_async_start(). While nrf_async_start() or nrf_async_cont()
  returns zero, call nrf_async_cont() for each SSI interrupt event (only
  transfer done can occur).
*/
struct nrf_async_cmd {
  uint32_t ssi_base;
  uint32_t csn_base;
  uint32_t csn_pin;
  uint8_t *recvbuf;
  uint8_t ssi_active;
};


/*
  Start a command, max 8 bytes.
  recvbuf must remain valid for the duration of the operation, or it can be
  NULL to ignore anything received.
  sendbuf can be discarded once nrf_async_cmd_start() returns.
*/
static int32_t
nrf_async_cmd_start(struct nrf_async_cmd *a,
                    uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len,
                    uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  if (len > 8)
    return -1;
  a->ssi_base = ssi_base;
  a->csn_base = csn_base;
  a->csn_pin = csn_pin;
  a->recvbuf = recvbuf;
  a->ssi_active = 1;
  /* Take CSN low to initiate transfer. */
  csn_low(csn_base, csn_pin);
  /* Empty any left-over stuff in the RX FIFO. */
  while (HWREG(ssi_base + SSI_O_SR) & SSI_SR_RNE)
    (void)(HWREG(ssi_base + SSI_O_DR));

  /* Set up so we get an interrupt when last bit has been transmitted. */
  HWREG(ssi_base + SSI_O_CR1) |= SSI_CR1_EOT;
  ROM_SSIIntClear(ssi_base, SSI_TXFF);
  HWREG(ssi_base + SSI_O_IM) |= SSI_TXFF;

  while (len > 0)
  {
    HWREG(ssi_base + SSI_O_DR) = *sendbuf++;
    --len;
  }

  return 0;
}


static int32_t
nrf_async_cmd_cont(struct nrf_async_cmd *a)
{
  uint8_t *p;

  if (!ROM_SSIBusy(a->ssi_base))
  {
    a->ssi_active = 0;
    /*
      Now that the transfer is completely done, set the TX interrupt back
      to disabled and trigger at 1/2 full fifo.
    */
    HWREG(a->ssi_base + SSI_O_IM) &= ~SSI_TXFF;
    HWREG(a->ssi_base + SSI_O_CR1) &= ~SSI_CR1_EOT;

    /* Take CSN high to complete transfer. */
    csn_high(a->csn_base, a->csn_pin);

    /* Empty the receive fifo (and return the data if so requested. */
    p = a->recvbuf;
    while (HWREG(a->ssi_base + SSI_O_SR) & SSI_SR_RNE)
      if (p)
        *p++ = HWREG(a->ssi_base + SSI_O_DR);
    return 1;
  }
  else
    return 0;
}


static void
nrf_rx(uint8_t *data, uint32_t len,
       uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > 32)
    len = 32;
  sendbuf[0] = nRF_R_RX_PAYLOAD;
  bzero(&sendbuf[1], len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
  memcpy(data, &recvbuf[1], len);
}


__attribute__((unused))
static void
nrf_tx_nack(uint8_t *data, uint32_t len,
            uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > 32)
    len = 32;
  sendbuf[0] = nRF_W_TX_PAYLOAD_NOACK;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_flush_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_TX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_flush_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_RX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg_n(uint8_t reg, const uint8_t *data, uint32_t len,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[6], recvbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg(uint8_t reg, uint8_t val,
              uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg_n(reg, &val, 1, ssi_base, csn_base, csn_pin);
}


static int32_t
nrf_write_reg_n_start(struct nrf_async_cmd *a, uint8_t reg, const uint8_t *data,
                      uint8_t *recvbuf, uint32_t len, uint32_t ssi_base,
                      uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[8];
  if (len > 7)
    len = 7;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  return nrf_async_cmd_start(a, recvbuf, sendbuf, len+1, ssi_base,
                             csn_base, csn_pin);
}


static int32_t
nrf_write_reg_start(struct nrf_async_cmd *a, uint8_t reg, uint8_t val,
                    uint8_t *recvbuf,
                    uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  return nrf_write_reg_n_start(a, reg, &val, recvbuf, 1,
                               ssi_base, csn_base, csn_pin);
}


static void
nrf_read_reg_n(uint8_t reg, uint8_t *out, uint32_t len,
               uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_R_REGISTER | reg;
  bzero(&sendbuf[1], len);
  ssi_cmd(out, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static uint8_t
nrf_read_reg(uint8_t reg, uint8_t *status_ptr,
             uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t recvbuf[2];
  nrf_read_reg_n(reg, recvbuf, 2, ssi_base, csn_base, csn_pin);
  if (status_ptr)
    *status_ptr = recvbuf[0];
  return recvbuf[1];
}


/*
  Configure nRF24L01+ as Rx or Tx.
    channel - radio frequency channel to use, 0 <= channel <= 127.
    power - nRF_RF_PWR_<X>DBM, <X> is 0, 6, 12, 18 dBm.
*/
static void
nrf_init_config(uint8_t is_rx, uint32_t channel, uint32_t power,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  static const uint8_t addr[3] = { 0xe7, 0xe7, 0xe7 };

  if (is_rx)
    nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  else
    nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Disable auto-ack, saving 9 bits/packet. Else 0x3f. */
  nrf_write_reg(nRF_EN_AA, 0, ssi_base, csn_base, csn_pin);
  /* Enable only pipe 0. */
  nrf_write_reg(nRF_EN_RXADDR, nRF_ERX_P0, ssi_base, csn_base, csn_pin);
  /* 3 byte adresses. */
  nrf_write_reg(nRF_SETUP_AW, nRF_AW_3BYTES, ssi_base, csn_base, csn_pin);
  /* Disable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, 0, ssi_base, csn_base, csn_pin);
  nrf_write_reg(nRF_RF_CH, channel, ssi_base, csn_base, csn_pin);
  /* Use 2Mbps, and set transmit power. */
  nrf_write_reg(nRF_RF_SETUP, nRF_RF_DR_HIGH | power,
                ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_RX_ADDR_P0, addr, 3, ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_TX_ADDR, addr, 3, ssi_base, csn_base, csn_pin);
  /* Set payload size for pipe 0. */
  if (is_rx)
    nrf_write_reg(nRF_RX_PW_P0, 32, ssi_base, csn_base, csn_pin);
  else
    nrf_write_reg(nRF_RX_PW_P0, 8, ssi_base, csn_base, csn_pin);
  /* Disable pipe 1-5. */
  nrf_write_reg(nRF_RX_PW_P1, 0, ssi_base, csn_base, csn_pin);
  /* Disable dynamic payload length. */
  nrf_write_reg(nRF_DYNDP, 0, ssi_base, csn_base, csn_pin);
  /* Allow disabling acks. */
  nrf_write_reg(nRF_FEATURE, nRF_EN_DYN_ACK, ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


static void
config_interrupts(void)
{
  ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_LOW_LEVEL);
  ROM_GPIOPinIntEnable(GPIO_PORTA_BASE, GPIO_PIN_7);
  ROM_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_LOW_LEVEL);
  ROM_GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
  ROM_IntMasterEnable();
  ROM_IntEnable(INT_GPIOA);
  ROM_IntEnable(INT_GPIOF);
}


void
IntHandlerGPIOa(void)
{
  uint32_t irq_status = HWREG(GPIO_PORTA_BASE + GPIO_O_MIS) & 0xff;
  if (irq_status & GPIO_PIN_7)
  {
    /* Tx IRQ. */

    /*
      Clear the interrupt request and disable further interrupts until we can
      clear the request from the device over SPI.
    */
    HWREG(GPIO_PORTA_BASE + GPIO_O_IM) &= ~GPIO_PIN_7 & 0xff;
    HWREG(GPIO_PORTA_BASE + GPIO_O_ICR) = GPIO_PIN_7;

    serial_output_str("Tx: IRQ: TX_DS\r\n");
  }
}


void
IntHandlerGPIOf(void)
{
  uint32_t irq_status = HWREG(GPIO_PORTF_BASE + GPIO_O_MIS) & 0xff;
  if (irq_status & GPIO_PIN_4)
  {
    /* Rx IRQ. */

    /*
      Clear the interrupt request and disable further interrupts until we can
      clear the request from the device over SPI.
    */
    HWREG(GPIO_PORTF_BASE + GPIO_O_IM) &= ~GPIO_PIN_4 & 0xff;
    HWREG(GPIO_PORTF_BASE + GPIO_O_ICR) = GPIO_PIN_4;

    serial_output_str("Rx: IRQ: RX_DR\r\n");
  }
}


struct nrf_async_dma transmit_packet_state;
volatile uint8_t transmit_packet_running = 0;
struct nrf_async_cmd nrf_cmd_state;
volatile uint8_t nrf_cmd_running = 0;

void
IntHandlerSSI0(void)
{
  if (transmit_packet_running && nrf_async_dma_cont(&transmit_packet_state))
    transmit_packet_running = 0;
  else if (nrf_cmd_running && nrf_async_cmd_cont(&nrf_cmd_state))
    nrf_cmd_running = 0;
}


void
IntHandlerSSI1(void)
{
  serial_output_str("%");
}


static void
transmit_packet(uint8_t startval,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin,
                uint32_t ce_base, uint32_t ce_pin)
{
  uint8_t buf[33];
  uint8_t recvbuf[33];
  uint8_t recvbuf2[2];
  uint32_t i;

  buf[0] = nRF_W_TX_PAYLOAD_NOACK;
  for (i = 0; i < 32; ++i)
    buf[i+1] = startval + i;
  recvbuf[0] = 0;
  transmit_packet_running =
    !nrf_async_dma_start(&transmit_packet_state, recvbuf, buf, 33, ssi_base,
                         UDMA_CHANNEL_SSI0RX, UDMA_CHANNEL_SSI0TX,
                         csn_base, csn_pin);
  while (transmit_packet_running)
    ;
  ce_high(ce_base, ce_pin);

  while (ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))
    ;
  ce_low(ce_base, ce_pin);
  nrf_cmd_running =
    !nrf_write_reg_start(&nrf_cmd_state, nRF_STATUS, nRF_TX_DS, recvbuf2,
                         ssi_base, csn_base, csn_pin);
  while (nrf_cmd_running)
    ;
  serial_output_str("Tx: status at send: ");
  serial_output_hexbyte(recvbuf[0]);
  serial_output_str(", at write_reg: ");
  serial_output_hexbyte(recvbuf2[0]);
  serial_output_str("\r\n");
}


static void
receive_packet(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin,
               uint32_t ce_base, uint32_t ce_pin)
{
  uint8_t buf[32];
  uint32_t i;

  while (ROM_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4))
    ;

  nrf_rx(buf, 32, ssi_base, csn_base, csn_pin);
  nrf_write_reg(nRF_STATUS, nRF_RX_DR, ssi_base, csn_base, csn_pin);

  serial_output_str("Rx packet: ");
  for (i = 0; i < 32; ++i)
    serial_output_hexbyte(buf[i]);
  serial_output_str("\r\n");
}


int main()
{
  uint8_t status;
  uint8_t val;

  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  ROM_FPULazyStackingEnable();

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

  config_ssi_gpio();
  config_spi(SSI0_BASE);
  config_spi(SSI1_BASE);

  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  ROM_SysCtlDelay(MCU_HZ/3/10);

  serial_output_str("Tx: Setting up...\r\n");
  config_interrupts();
  config_udma_for_spi();
  nrf_init_config(0 /* Tx */, 2, nRF_RF_PWR_18DBM,
                  SSI0_BASE, GPIO_PORTA_BASE, GPIO_PIN_3);
  serial_output_str("Tx: Read CONFIG=0x");
  val = nrf_read_reg(nRF_CONFIG, &status,
                     SSI0_BASE, GPIO_PORTA_BASE, GPIO_PIN_3);
  serial_output_hexbyte(val);
  serial_output_str(" status=0x");
  serial_output_hexbyte(status);
  serial_output_str("\r\n");

  serial_output_str("Rx: Setting up...\r\n");
  nrf_init_config(1 /* Rx */, 2, nRF_RF_PWR_18DBM,
                  SSI1_BASE, GPIO_PORTF_BASE, GPIO_PIN_3);
  serial_output_str("Rx: Read CONFIG=0x");
  val = nrf_read_reg(nRF_CONFIG, &status,
                     SSI1_BASE, GPIO_PORTF_BASE, GPIO_PIN_3);
  serial_output_hexbyte(val);
  serial_output_str(" status=0x");
  serial_output_hexbyte(status);
  serial_output_str("\r\n");
  serial_output_str("Done!\r\n");

  /* Set Rx in receive mode. */
  ce_high(GPIO_PORTB_BASE, GPIO_PIN_0);
  /* Wait a bit to make sure Rx is ready. */
  ROM_SysCtlDelay(MCU_HZ/3/100);

  transmit_packet(42, SSI0_BASE, GPIO_PORTA_BASE, GPIO_PIN_3,
                  GPIO_PORTA_BASE, GPIO_PIN_6);
  serial_output_str("Tx: Sent packet\r\n");

  serial_output_str("Rx: Waiting for packet...\r\n");
  receive_packet(SSI1_BASE, GPIO_PORTF_BASE, GPIO_PIN_3,
                 GPIO_PORTB_BASE, GPIO_PIN_0);

  for(;;)
    ;
}
