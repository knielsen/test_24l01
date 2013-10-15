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

#include "nrf24l01p.h"


/*
  nRF24L01 pinout:

    PA2  SCK    -> PF2                  GND *1 2. VCC
    PA3  CSN    -> PF3                  PB0 .3 4. PA3
    PA4  MISO   -> PF0                  PA2 .5 6. PA5
    PA5  MOSI   -> PF1                  PA4 .7 8. PF4
    PB0  CE
    PF4  IRQ
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
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
  ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
  /* CE pin, low initially */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
  ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
  /* CSN pin, high initially */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
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


static void
bzero(uint8_t *buf, uint32_t len)
{
  while (len > 0)
  {
    *buf++ = 0;
    --len;
  }
}


/*
  Set the DC register on TLC5940 through SSI.
  Then read out the TLC5940 status register and check that DC is correct.
*/
static void
ssi_cmd(uint32_t ssi_base, uint8_t *recvbuf, const uint8_t *sendbuf,
        uint32_t len)
{
  uint32_t i;
  uint32_t data;

  /* Take CSN low to initiate transfer. */
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);

  for (i = 0; i < len; ++i)
  {
    ROM_SSIDataPut(ssi_base, sendbuf[i]);
    while (ROM_SSIBusy(ssi_base))
      ;
    ROM_SSIDataGet(ssi_base, &data);
    recvbuf[i] = data;
  }

  /* Take CSN high to complete transfer. */
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

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
}


int main()
{
  uint8_t recvbuf[32];
  uint8_t sendbuf[32];

  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

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
  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  ROM_SysCtlDelay(MCU_HZ/3/10);
  serial_output_str("Write CONFIG...\r\n");
  sendbuf[0] = nRF_W_REGISTER | nRF_CONFIG;
  sendbuf[1] = nRF_MASK_RX_DR|nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP;
  ssi_cmd(SSI0_BASE, recvbuf, sendbuf, 2);
  serial_output_str("Read CONFIG...\r\n");
  bzero(sendbuf, 2);
  sendbuf[0] = nRF_R_REGISTER | nRF_CONFIG;
  ssi_cmd(SSI0_BASE, recvbuf, sendbuf, 2);
  serial_output_str("Done!\r\n");

  for(;;)
    ;
}
