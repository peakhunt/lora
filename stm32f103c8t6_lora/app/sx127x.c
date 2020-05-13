#include <string.h>
#include "app_common.h"
#include "sx127x.h"
#include "event_dispatcher.h"
#include "event_list.h"
#include "spi.h"
#include "math_util.h"


/////////////////////////////////////////////////////////////////////////////
//
// internal definitions 
//
/////////////////////////////////////////////////////////////////////////////
#define SX127X_SPI_TIMEOUT        1000

typedef struct
{
  GPIO_TypeDef*     port;
  uint16_t          pin;
} sx127x_pin_def_t;

typedef struct
{
  sx127x_instance_t   instance;
  SPI_HandleTypeDef*  hspi;
  sx127x_pin_def_t    gpio_cs;
  sx127x_pin_def_t    gpio_rst;
  uint16_t            irqn;
  uint32_t            event;
  uint32_t            fosc;       			// oscillator HZ
  enum sx127x_pa      pa;
  enum sx127x_opmode  opmode;
  uint8_t             initialized;
  bool                loraregmap;

  sx127x_tx_complete  tx_cb;
  sx127x_rx_complete  rx_cb;
} sx127x_ctx_t;

static sx127x_ctx_t   _sx127x_instances[SX127X_NUM_INSTANCE] = 
{
  {
    .instance   = sx127x_instance_0,
    .hspi       = &hspi1,
    .gpio_cs    = {
      .port = LORA_CS_GPIO_Port,
      .pin  = LORA_CS_Pin,
    },
    .gpio_rst   = {
      .port     = LORA_RESET_GPIO_Port,
      .pin      = LORA_RESET_Pin,
    },
    .irqn       = LORA_IRQ_EXTI_IRQn,
    .event      = DISPATCH_EVENT_SX127X_IRQ,
    .fosc       = 32000000,
    .pa         = SX127X_PA_PABOOST,
    .opmode     = SX127X_OPMODE_STANDBY,
    .loraregmap = false,
  },
};

/////////////////////////////////////////////////////////////////////////////
//
// utility macros & prototypes
//
/////////////////////////////////////////////////////////////////////////////
#define BIT(nr) (1UL << (nr))

static void __sx127x_init(sx127x_ctx_t* sx127x);
static void __sx127x_set_modulation(sx127x_ctx_t* sx127x, enum sx127x_modulation modulation);
static enum sx127x_modulation __sx127x_get_modulation(sx127x_ctx_t* sx127x);
static void __sx127x_set_opmode(sx127x_ctx_t* sx127x, enum sx127x_opmode opmode, bool retain);
static enum sx127x_opmode __sx127x_get_opmode(sx127x_ctx_t* sx127x);
static void __sx127x_set_carrier_freq(sx127x_ctx_t* sx127x, uint64_t freq);
static uint32_t __sx127x_get_carrier_freq(sx127x_ctx_t* sx127x);
static void __sx127x_set_sf(sx127x_ctx_t* sx127x, int32_t sf);
static int32_t __sx127x_get_sf(sx127x_ctx_t* sx127x);
static int32_t __sx127x_get_bw(sx127x_ctx_t* sx127x);
static int32_t __sx127x_get_coding_rate(sx127x_ctx_t* sx127x);
static int32_t __sx127x_get_implicitheadermodeon(sx127x_ctx_t* sx127x);
static void __sx127x_set_paoutput(sx127x_ctx_t* sx127x, enum sx127x_pa paoutput);
static enum sx127x_pa __sx127x_get_paoutput(sx127x_ctx_t* sx127x);

static void __sx127x_set_output_power(sx127x_ctx_t* sx127x, int32_t power);
static int32_t __sx127x_get_output_power(sx127x_ctx_t* sx127x);

static int32_t __sx127x_tx(sx127x_ctx_t* sx127x, uint8_t* pkt, uint32_t len);

/////////////////////////////////////////////////////////////////////////////
//
// internal register definitions
//
/////////////////////////////////////////////////////////////////////////////

#define SX127X_LORAREG_MASK                           BIT(8)
#define SX127X_LORAREG(addr)                          (addr | SX127X_LORAREG_MASK)
#define SX127X_FSKOOKREG_MASK                         BIT(9)
#define SX127X_FSKOOKREG(addr)                        (addr | SX127X_FSKOOK_MASK)

#define SX127X_REGADDR(reg)                           (reg & 0x7f)
#define SX127X_WRITEADDR(addr)                        (addr | (1 << 7))

#define SX127X_REG_FIFO                               0x00

#define SX127X_REG_OPMODE                             0x01
#define SX127X_REG_OPMODE_LONGRANGEMODE               BIT(7)
#define SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE       (BIT(6) | BIT(5))
#define SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_FSK   0
#define SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_OOK   BIT(5)
#define SX127X_REG_OPMODE_LOWFREQUENCYMODEON          BIT(3)
#define SX127X_REG_OPMODE_MODE                        (BIT(2) | BIT(1) | BIT(0))
#define SX127X_REG_OPMODE_MODE_SLEEPMODE              0
#define SX127X_REG_OPMODE_MODE_STDBYMODE              BIT(0)

#define SX127X_REG_FSKOOK_BITRATEMSB                  SX127X_FSKOOKREG(0x02)
#define SX127X_REG_FSKOOK_BITRATELSB                  SX127X_FSKOOKREG(0x03)
#define SX127X_REG_FSKOOK_FDEVMSB                     SX127X_FSKOOKREG(0x04)
#define SX127X_REG_FSKOOK_FDEVLSB                     SX127X_FSKOOKREG(0x05)
#define SX127X_REG_FRFMSB                             0x06
#define SX127X_REG_FRFMLD                             0x07
#define SX127X_REG_FRFLSB                             0x08

#define SX127X_REG_PACONFIG                           0x09
#define SX127X_REG_PACONFIG_PASELECT                  BIT(7)
#define SX127X_REG_PACONFIG_MAXPOWER                  (BIT(6) | BIT(5) | BIT(4))
#define SX127X_REG_PACONFIG_MAXPOWER_SHIFT            4
#define SX127X_REG_PACONFIG_OUTPUTPOWER               (BIT(3) | BIT(2) | BIT(1) | BIT(0))

#define SX127X_REG_PARAMP                             0x0a
#define SX127X_REG_OCP                                0x0b
#define SX127X_REG_LNA                                0x0c
#define SX127X_REG_FSKOOK_RXCONFIG                    SX127X_FSKOOKREG(0x0d)
#define SX127X_REG_LORA_FIFOADDRPTR                   SX127X_LORAREG(0x0d)
#define SX127X_REG_FSKOOK_RSSICONFIG                  SX127X_FSKOOKREG(0x0e)
#define SX127X_REG_LORA_FIFOTXBASEADDR                SX127X_LORAREG(0x0e)
#define SX127X_REG_FSKOOK_RSSICOLLISION               SX127X_FSKOOKREG(0x0f)
#define SX127X_REG_LORA_RXBASEADDR                    SX127X_LORAREG(0x0f)
#define SX127X_REG_FSKOOK_RSSTHRESH                   SX127X_FSKOOKREG(0x10)
#define SX127X_REG_LORA_RXCURRENTADDR                 SX127X_LORAREG(0x10)
#define SX127X_REG_FSKOOK_RSSIVALUE                   SX127X_FSKOOKREG(0x11)
#define SX127X_REG_LORA_IRQMASKFLAGS                  SX127X_LORAREG(0x11)
#define SX127X_REG_FSKOOK_RXBW                        SX127X_FSKOOKREG(0x12)

#define SX127X_REG_LORA_IRQFLAGS                      SX127X_LORAREG(0x12)
#define SX127X_REG_LORA_IRQFLAGS_RXTIMEOUT            BIT(7)
#define SX127X_REG_LORA_IRQFLAGS_RXDONE               BIT(6)
#define SX127X_REG_LORA_IRQFLAGS_PAYLOADCRCERROR      BIT(5)
#define SX127X_REG_LORA_IRQFLAGS_TXDONE               BIT(3)
#define SX127X_REG_LORA_IRQFLAGS_CADDONE              BIT(2)
#define SX127X_REG_LORA_IRQFLAGS_CADDETECTED          BIT(0)

#define SX127X_REG_FSKOOK_AFCBW                       SX127X_FSKOOKREG(0x13)
#define SX127X_REG_LORA_RXNBBYTES                     SX127X_LORAREG(0x13)
#define SX127X_REG_FSKOOK_OOKPEAK                     SX127X_FSKOOKREG(0x14)
#define SX127X_REG_LORA_RXHEADERCNTVALUEMSB           SX127X_LORAREG(0x14)
#define SX127X_REG_FSKOOK_OOKFIX                      SX127X_FSKOOKREG(0x15)
#define SX127X_REG_LORA_RXHEADERCNTVALUELSB           SX127X_LORAREG(0x15)
#define SX127X_REG_FSKOOK_OOKAVG                      SX127X_FSKOOKREG(0x16)
#define SX127X_REG_LORA_RXPACKETCNTVALUEMSB           SX127X_LORAREG(0x16)
#define SX127X_REG_LORA_RXPACKETCNTVALUELSB           SX127X_LORAREG(0x17)
#define SX127X_REG_LORA_MODEMSTAT                     SX127X_LORAREG(0x18)
#define SX127X_REG_LORA_PKTSNRVALUE                   SX127X_LORAREG(0x19)
#define SX127X_REG_FSKOOK_AFCFEI                      SX127X_FSKOOKREG(0x1a)
#define SX127X_REG_LORA_PKTRSSIVALUE                  SX127X_LORAREG(0x1a)
#define SX127X_REG_FSKOOK_AFCMSB                      SX127X_FSKOOKREG(0x1b)
#define SX127X_REG_LORA_RSSIVALUE                     SX127X_LORAREG(0x1b)
#define SX127X_REG_FSKOOK_AFCLSB                      SX127X_FSKOOKREG(0x1c)
#define SX127X_REG_LORA_HOPCHANNEL                    SX127X_LORAREG(0x1c)
#define SX127X_REG_FSKOOK_FEIMSB                      SX127X_FSKOOKREG(0x1d)

#define SX127X_REG_LORA_MODEMCONFIG1                  SX127X_LORAREG(0x1d)
#define SX127X_REG_LORA_MODEMCONFIG1_BW               (BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define SX127X_REG_LORA_MODEMCONFIG1_BW_SHIFT         4
#define SX127X_REG_LORA_MODEMCONFIG1_BW_MAX           9
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE       (BIT(3) | BIT(2) | BIT(1))
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_SHIFT 1
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MIN   1
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MAX   6
#define SX127X_REG_LORA_MODEMCONFIG1_IMPLICITHEADERMODEON   BIT(0)

#define SX127X_REG_FSKOOK_FEILSB                      SX127X_FSKOOKREG(0x1e)

#define SX127X_REG_LORA_MODEMCONFIG2                  SX127X_LORAREG(0x1e)
#define SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR        (BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT  4
#define SX127X_REG_LORA_MODEMCONFIG2_RXPAYLOADCRCON         BIT(2)

#define SX127X_REG_FSKOOK_PREAMBLEDETECT              SX127X_FSKOOKREG(0x1f)
#define SX127X_REG_LORA_SYMBTIMEOUTLSB                SX127X_LORAREG(0x1f)
#define SX127X_REG_FSKOOK_RXTIMEOUT1                  SX127X_FSKOOKREG(0x20)
#define SX127X_REG_LORA_PREAMBLEMSB                   SX127X_LORAREG(0x20)
#define SX127X_REG_FSKOOK_RXTIMEOUT2                  SX127X_FSKOOKREG(0x21)
#define SX127X_REG_LORA_PREAMBLELSB                   SX127X_LORAREG(0x21)
#define SX127X_REG_FSKOOK_RXTIMEOUT3                  SX127X_FSKOOKREG(0x22)
#define SX127X_REG_LORA_PAYLOADLENGTH                 SX127X_LORAREG(0x22)
#define SX127X_REG_FSKOOK_RXDELAY                     SX127X_FSKOOKREG(0x23)
#define SX127X_REG_LORA_MAXPAYLOADLENGTH              SX127X_LORAREG(0x23)
#define SX127X_REG_FSKOOK_OSC                         SX127X_FSKOOKREG(0x24)
#define SX127X_REG_LORA_HOPPERIOD                     SX127X_LORAREG(0x24)
#define SX127X_REG_FSKOOK_PREAMBLEMSB                 SX127X_FSKOOKREG(0x25)
#define SX127X_REG_LORA_FIFORXBYTEADDR                SX127X_LORAREG(0x25)
#define SX127X_REG_FSKOOK_PREAMBLELSB                 SX127X_FSKOOKREG(0x26)

#define SX127X_REG_LORA_MODEMCONFIG3                  SX127X_LORAREG(0x26)
#define SX127X_REG_LORA_MODEMCONFIG3_LOWDATARATEOPTIMIZE    BIT(3)

#define SX127X_REG_FSKOOK_SYNCCONFIG                  SX127X_FSKOOKREG(0x27)
#define SX127X_REG_FSKOOK_SYNCVALUE1                  SX127X_FSKOOKREG(0x28)
#define SX127X_REG_LORA_FEIMSB                        SX127X_LORAREG(0x28)
#define SX127X_REG_FSKOOK_SYNCVALUE2                  SX127X_FSKOOKREG(0x29)
#define SX127X_REG_LORA_FEIMID                        SX127X_LORAREG(0x29)
#define SX127X_REG_FSKOOK_SYNCVALUE3                  SX127X_FSKOOKREG(0x2a)
#define SX127X_REG_LORA_FEILSB                        SX127X_LORAREG(0x29)
#define SX127X_REG_FSKOOK_SYNCVALUE4                  SX127X_FSKOOKREG(0x2b)
#define SX127X_REG_FSKOOK_SYNCVALUE5                  SX127X_FSKOOKREG(0x2c)
#define SX127X_REG_LORA_RSSIWIDEBAND                  SX127X_LORAREG(0x2c)
#define SX127X_REG_FSKOOK_SYNCVALUE6                  SX127X_FSKOOKREG(0x2d)
#define SX127X_REG_FSKOOK_SYNCVALUE7                  SX127X_FSKOOKREG(0x2e)
#define SX127X_REG_FSKOOK_SYNCVALUE8                  SX127X_FSKOOKREG(0x2f)
#define SX127X_REG_FSKOOK_PACKETCONFIG1               SX127X_FSKOOKREG(0x30)
#define SX127X_REG_FSKOOK_PACKETCONFIG2               SX127X_FSKOOKREG(0x31)

#define SX127X_REG_LORA_DETECTOPTIMIZATION            SX127X_LORAREG(0x31)
#define SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE          (BIT(2) | BIT(1) | BIT(0))
#define SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF7SF12  0x03
#define SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF6      0x05

#define SX127X_REG_FSKOOK_PAYLOADLENGTH               SX127X_FSKOOKREG(0x32)
#define SX127X_REG_FSKOOK_NODEADRS                    SX127X_FSKOOKREG(0x33)
#define SX127X_REG_LORA_INVERTIQ                      SX127X_LORAREG(0x33)
#define SX127X_REG_LORA_INVERTIQ_INVERTIQ             BIT(6)

#define SX127X_REG_FSKOOK_BROADCASTADRS               SX127X_FSKOOKREG(0x34)
#define SX127X_REG_FSKOOK_FIFOTHRESH                  SX127X_FSKOOKREG(0x35)
#define SX127X_REG_FSKOOK_SEQCONFIG1                  SX127X_FSKOOKREG(0x36)
#define SX127X_REG_FSKOOK_SEQCONFIG2                  SX127X_FSKOOKREG(0x37)
#define SX127X_REG_LORA_DETECTIONTHRESHOLD            SX127X_LORAREG(0x37)
#define SX127X_REG_FSKOOK_TIMERRESOL                  SX127X_FSKOOKREG(0x38)
#define SX127X_REG_FSKOOK_TIMER1COEF                  SX127X_FSKOOKREG(0x39)
#define SX127X_REG_LORA_SYNCWORD                      SX127X_LORAREG(0x39)
#define SX127X_REG_FSKOOK_TIMER2COEF                  SX127X_FSKOOKREG(0x3a)
#define SX127X_REG_FSKOOK_IMAGECAL                    SX127X_FSKOOKREG(0x3b)
#define SX127X_REG_FSKOOK_TEMP                        SX127X_FSKOOKREG(0x3c)
#define SX127X_REG_FSKOOK_LOWBAT                      SX127X_FSKOOKREG(0x3d)
#define SX127X_REG_FSKOOK_IRQFLAGS1                   SX127X_FSKOOKREG(0x3e)
#define SX127X_REG_FSKOOK_IRQFLAGS2                   SX127X_FSKOOKREG(0x3f)

#define SX127X_REG_DIOMAPPING1                        0x40
#define SX127X_REG_DIOMAPPING1_DIO0                   (BIT(7) | BIT(6))
#define SX127X_REG_DIOMAPPING1_DIO0_RXDONE            0
#define SX127X_REG_DIOMAPPING1_DIO0_TXDONE            (BIT(6))
#define SX127X_REG_DIOMAPPING1_DIO0_CADDONE           (BIT(7))

#define SX127X_REG_DIOMAPPING2                        0x41
#define SX127X_REG_VERSION                            0x42
#define SX127X_REG_FSKOOK_PLLHOP                      SX127X_FSKOOKREG(0x44)
#define SX127X_REG_TCXO                               0x4b
#define SX127X_REG_PADAC                              0x4d
#define SX127X_REG_FORMERTEMP                         0x5b
#define SX127X_REG_FSKOOK_BITRATEFRAC                 SX127X_FSKOOKREG(0x5d)
#define SX127X_REG_AGCREF                             0x61
#define SX127X_REG_AGCTHRESH1                         0x62
#define SX127X_REG_AGCTHRESH2                         0x63
#define SX127X_REG_AGCTHRESH3                         0x64
#define SX127X_REG_PLL                                0x70

/////////////////////////////////////////////////////////////////////////////
//
// register access via SPI
//
/////////////////////////////////////////////////////////////////////////////
static inline sx127x_ctx_t*
sx127x_get(sx127x_instance_t instance)
{
  return &_sx127x_instances[instance];
}

static inline int32_t
spi_write_then_read(sx127x_ctx_t* sx127x, uint8_t* tx, uint16_t ntx, uint8_t* rx, uint16_t nrx)
{
  HAL_GPIO_WritePin(sx127x->gpio_cs.port, sx127x->gpio_cs.pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(sx127x->hspi, tx, ntx, SX127X_SPI_TIMEOUT);
  HAL_SPI_Receive(sx127x->hspi, rx, nrx, SX127X_SPI_TIMEOUT);

  HAL_GPIO_WritePin(sx127x->gpio_cs.port, sx127x->gpio_cs.pin, GPIO_PIN_SET);

  return 0;
}

static inline int32_t
spi_write(sx127x_ctx_t* sx127x, uint8_t* tx, uint16_t ntx)
{
  HAL_GPIO_WritePin(sx127x->gpio_cs.port, sx127x->gpio_cs.pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(sx127x->hspi, tx, ntx, SX127X_SPI_TIMEOUT);

  HAL_GPIO_WritePin(sx127x->gpio_cs.port, sx127x->gpio_cs.pin, GPIO_PIN_SET);

  return 0;
}

static inline void
spi_write_pkt(sx127x_ctx_t* sx127x, uint8_t addr, uint8_t* data, uint16_t len)
{
  HAL_GPIO_WritePin(sx127x->gpio_cs.port, sx127x->gpio_cs.pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(sx127x->hspi, &addr, 1, SX127X_SPI_TIMEOUT);
  HAL_SPI_Transmit(sx127x->hspi, data, len, SX127X_SPI_TIMEOUT);

  HAL_GPIO_WritePin(sx127x->gpio_cs.port, sx127x->gpio_cs.pin, GPIO_PIN_SET);
}

static int32_t
sx127x_reg_read(sx127x_ctx_t* sx127x, uint16_t reg, uint8_t* result)
{
  uint8_t addr = reg & 0xff;
  int32_t ret;
  
  ret = spi_write_then_read(sx127x, &addr, 1, result, 1);
  return ret;
}

#if 0
static int32_t
sx127x_reg_read16(sx127x_ctx_t* sx127x, uint16_t reg, uint16_t* result)
{
  uint8_t addr = reg & 0xff,
          buf[2];
  int32_t ret;

  ret = spi_write_then_read(sx127x, &addr, 1, buf, 2);

  *result = (buf[0] << 8) | buf[1];

  return ret;
}
#endif

static int32_t
sx127x_reg_read24(sx127x_ctx_t* sx127x, uint16_t reg, uint32_t* result)
{
  uint8_t addr = reg & 0xff,
          buf[3];
  int32_t ret;
  
  ret = spi_write_then_read(sx127x, &addr, 1, buf, 3);
  *result = (buf[0] << 16) | (buf[1] << 8) | buf[0];
  return ret;
}

static int32_t
sx127x_reg_write(sx127x_ctx_t* sx127x, uint16_t reg, uint8_t value)
{
  uint8_t addr = SX127X_REGADDR(reg),
          buff[2];
  int32_t ret;

  buff[0] = SX127X_WRITEADDR(addr);
  buff[1] = value;

  ret = spi_write(sx127x, buff, 2);
  return ret;
}

static int32_t
sx127x_reg_write24(sx127x_ctx_t* sx127x, uint16_t reg, uint32_t value)
{
  uint8_t addr = SX127X_REGADDR(reg),
          buff[4];
  int32_t ret;

  buff[0] = SX127X_WRITEADDR(addr);
  buff[1] = (value >> 16) & 0xff;
  buff[2] = (value >> 8) & 0xff;
  buff[3] = value & 0xff;

  ret = spi_write(sx127x, buff, sizeof(buff));

  return ret;
}

static int
sx127x_readpkt(sx127x_ctx_t* sx127x, void *buffer, uint8_t *len)
{
  uint8_t   addr = SX127X_REG_FIFO,
            pktstart,
            rxbytes;
  int32_t   ret;

  ret = sx127x_reg_read(sx127x, SX127X_REG_LORA_RXCURRENTADDR, &pktstart);
  ret = sx127x_reg_read(sx127x, SX127X_REG_LORA_RXNBBYTES, &rxbytes);

  ret = sx127x_reg_write(sx127x, SX127X_REG_LORA_FIFOADDRPTR, pktstart);
  ret = spi_write_then_read(sx127x, &addr, 1, buffer, rxbytes);

  *len = rxbytes;
  return ret;
}

static int sx127x_writepkt(sx127x_ctx_t *sx127x, void *buffer, uint8_t len)
{
  uint8_t addr = SX127X_WRITEADDR(SX127X_REGADDR(SX127X_REG_FIFO));
  int32_t ret;

  ret = sx127x_reg_write(sx127x, SX127X_REG_LORA_FIFOTXBASEADDR, 0);
  ret = sx127x_reg_write(sx127x, SX127X_REG_LORA_FIFOADDRPTR, 0);
  ret = sx127x_reg_write(sx127x, SX127X_REG_LORA_PAYLOADLENGTH, len);

  spi_write_pkt(sx127x, addr, buffer, len);

  ret = sx127x_reg_write(sx127x, SX127X_REG_LORA_FIFOADDRPTR, 0);

  return ret;
}

/////////////////////////////////////////////////////////////////////////////
//
// IRQ handler
//
/////////////////////////////////////////////////////////////////////////////
static void
__sx127x_irq_handle(sx127x_instance_t instance, sx127x_ctx_t* sx127x)
{
  uint8_t         irqflags,
                  len,
                  snr,
                  rssi;
  static uint8_t  buf[128];
  uint32_t        fei;
  sx127x_pkt_t    pkt;

  sx127x_reg_read(sx127x, SX127X_REG_LORA_IRQFLAGS, &irqflags);

  if(irqflags & SX127X_REG_LORA_IRQFLAGS_RXDONE)
  {
    // reading packet
    memset(&pkt, 0, sizeof(pkt));

    sx127x_readpkt(sx127x, buf, &len);

    sx127x_reg_read(sx127x, SX127X_REG_LORA_PKTSNRVALUE, &snr);
    sx127x_reg_read(sx127x, SX127X_REG_LORA_PKTRSSIVALUE, &rssi);
    sx127x_reg_read24(sx127x, SX127X_REG_LORA_FEIMSB, &fei);

    pkt.hdrlen      = sizeof(pkt);
    pkt.payloadlen  = len;
    pkt.len         = pkt.hdrlen + pkt.payloadlen;
    pkt.snr         = (int16_t)(snr << 2) / 4 ;
    pkt.rssi        = -157 + rssi; //TODO fix this for the LF port

    if(irqflags & SX127X_REG_LORA_IRQFLAGS_PAYLOADCRCERROR)
    {
      // CRC Error for received payload
      pkt.crcfail = 1;
    }

    if(sx127x->rx_cb != NULL)
    {
      sx127x->rx_cb(instance, &pkt, buf);
    }
  }
  else if(irqflags & SX127X_REG_LORA_IRQFLAGS_TXDONE)
  {
    // TX complete

    /* after tx the chip goes back to standby so restore the user selected mode if it wasn't standby */
    if(sx127x->opmode != SX127X_OPMODE_STANDBY)
    {
      // restoring opmode
      __sx127x_set_opmode(sx127x, sx127x->opmode, false);
    }

    if(sx127x->tx_cb != NULL)
    {
      sx127x->tx_cb(instance);
    }
  }
  else if(irqflags & SX127X_REG_LORA_IRQFLAGS_CADDONE)
  {
    if(irqflags & SX127X_REG_LORA_IRQFLAGS_CADDETECTED)
    {
      // CAD done, detected activity
    }
    else
    {
      // CAD done, nothing detected
    }
  }
  else
  {
    // XXX
    // unhandled interrupt stats
  }
  sx127x_reg_write(sx127x, SX127X_REG_LORA_IRQFLAGS, 0xff);
}

static void
sx127x_irq_handler(uint32_t event)
{
  for(int i = 0; i < sizeof(_sx127x_instances)/sizeof(sx127x_ctx_t); i++)
  {
    if(_sx127x_instances[i].event == event)
    {
      __sx127x_irq_handle((sx127x_instance_t)i, &_sx127x_instances[i]);
      return;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
//
// internal core
//
/////////////////////////////////////////////////////////////////////////////
static void
__sx127x_init(sx127x_ctx_t* sx127x)
{
  uint8_t version;

  HAL_NVIC_DisableIRQ(sx127x->irqn);

  // reset the chip
  HAL_Delay(20);
  HAL_GPIO_WritePin(sx127x->gpio_rst.port, sx127x->gpio_rst.pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(sx127x->gpio_rst.port, sx127x->gpio_rst.pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(sx127x->gpio_rst.port, sx127x->gpio_rst.pin, GPIO_PIN_SET);
  HAL_Delay(20);

  // check chip revision for verification purpose
  sx127x_reg_read(sx127x, SX127X_REG_VERSION, &version);
  if(version != 0x12)
  {
    // XXX error indication
    sx127x->initialized = false;
    while(1)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(1000);
    }
    return;
  }

  sx127x->initialized = true;
  HAL_NVIC_EnableIRQ(sx127x->irqn);
}

static void
__sx127x_set_modulation(sx127x_ctx_t* sx127x, enum sx127x_modulation modulation)
{
  uint8_t opmode;

  sx127x_reg_read(sx127x, SX127X_REG_OPMODE, &opmode);

  // LoRa mode bit can only be changed in sleep mode
  if(opmode & SX127X_REG_OPMODE_MODE)
  {
    opmode &= ~SX127X_REG_OPMODE_MODE;
    sx127x_reg_write(sx127x, SX127X_REG_OPMODE, opmode);
  }

  switch(modulation)
  {
  case SX127X_MODULATION_FSK:
  case SX127X_MODULATION_OOK:
    opmode &= ~SX127X_REG_OPMODE_LONGRANGEMODE;
    sx127x->loraregmap = false;
    break;
  
  case SX127X_MODULATION_LORA:
    opmode |= SX127X_REG_OPMODE_LONGRANGEMODE;
    sx127x->loraregmap = true;
    break;

  default:
    return;
  }
  sx127x_reg_write(sx127x, SX127X_REG_OPMODE, opmode);
}

static enum sx127x_modulation
__sx127x_get_modulation(sx127x_ctx_t* sx127x)
{
  uint8_t opmode;
  uint8_t mod;

  sx127x_reg_read(sx127x, SX127X_REG_OPMODE, &opmode);

  if(opmode & SX127X_REG_OPMODE_LONGRANGEMODE)
  {
    return SX127X_MODULATION_LORA;
  }
  else
  {
    mod = opmode & SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE;
    if(mod == SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_FSK)
    {
      return SX127X_MODULATION_FSK;
    }
    else if(mod == SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_OOK)
    {
      return SX127X_MODULATION_OOK;
    }
  }
  return SX127X_MODULATION_INVALID;
}

static void
__sx127x_set_opmode(sx127x_ctx_t* sx127x, enum sx127x_opmode mode, bool retain)
{
  uint8_t   opmode,
            diomapping1;

  (void)sx127x_reg_read(sx127x, SX127X_REG_OPMODE, &opmode);

  if(mode < SX127X_OPMODE_SLEEP || mode > SX127X_OPMODE_CAD)
  {
    // invalid opmode
    return;
  }
  else if((opmode & SX127X_REG_OPMODE_LONGRANGEMODE) && (mode == SX127X_OPMODE_RX))
  {
    // opmode not valid in LoRa mode
    return;
  }
  else if(!(opmode & SX127X_REG_OPMODE_LONGRANGEMODE) && (mode > SX127X_OPMODE_RX))
  {
    // opmode not valid in FSK/OOK mode
    return;
  }
  else
  {
    if(retain)
    {
      sx127x->opmode = mode;
    }
    sx127x_reg_read(sx127x, SX127X_REG_DIOMAPPING1, &diomapping1);
    diomapping1 &= ~SX127X_REG_DIOMAPPING1_DIO0;

    switch(mode)
    {
    case SX127X_OPMODE_CAD:
      diomapping1 |= SX127X_REG_DIOMAPPING1_DIO0_CADDONE;
      break;

    case SX127X_OPMODE_TX:
      diomapping1 |= SX127X_REG_DIOMAPPING1_DIO0_TXDONE;
      break;

    case SX127X_OPMODE_RX:
    case SX127X_OPMODE_RXCONTINUOS:
    case SX127X_OPMODE_RXSINGLE:
      diomapping1 |= SX127X_REG_DIOMAPPING1_DIO0_RXDONE;
      break;
    
    default:
      break;
    }

    opmode &= ~SX127X_REG_OPMODE_MODE;
    if(mode > SX127X_OPMODE_RX)
    {
      mode -= 1;
    }
    opmode |= mode;
    sx127x_reg_write(sx127x, SX127X_REG_DIOMAPPING1, diomapping1);
    sx127x_reg_write(sx127x, SX127X_REG_OPMODE, opmode);
  }
}

static enum sx127x_opmode
__sx127x_get_opmode(sx127x_ctx_t* sx127x)
{
  uint8_t opmode, mode;

  sx127x_reg_read(sx127x, SX127X_REG_OPMODE, &opmode);
  mode = opmode & SX127X_REG_OPMODE_MODE;

  //
  // FIXME check what it really means
  //
  if(mode > 4 && (opmode & SX127X_REG_OPMODE_LONGRANGEMODE))
  {
    mode += 1;
  }

  return mode;
}

static void
__sx127x_set_carrier_freq(sx127x_ctx_t* sx127x, uint64_t freq)
{
  uint8_t opmode, newopmode;
  uint32_t freq32;

  sx127x_reg_read(sx127x, SX127X_REG_OPMODE, &opmode);

  newopmode = opmode & ~SX127X_REG_OPMODE_LOWFREQUENCYMODEON;

  if(freq < 700000000)
  {
    newopmode |= SX127X_REG_OPMODE_LOWFREQUENCYMODEON;
  }

  if(newopmode != opmode)
  {
    sx127x_reg_write(sx127x, SX127X_REG_OPMODE, newopmode);
  }

  freq *= 524288;

  // XXX
  // do_div(freq, sx127x->fosc);
  //sx127x_reg_write24(sx127x, SX127X_REG_FRFMSB, freq);
  //
  freq32 = freq / sx127x->fosc;

  sx127x_reg_write24(sx127x, SX127X_REG_FRFMSB, freq32);
}

static uint32_t
__sx127x_get_carrier_freq(sx127x_ctx_t* sx127x)
{
  uint8_t msb, mld, lsb;
  uint32_t frf;
  uint32_t freq;

  sx127x_reg_read(sx127x, SX127X_REG_FRFMSB, &msb);
  sx127x_reg_read(sx127x, SX127X_REG_FRFMLD, &mld);
  sx127x_reg_read(sx127x, SX127X_REG_FRFLSB, &lsb);

  frf = (msb << 16) | (mld << 8) | lsb;
  freq = ((uint64_t)sx127x->fosc * frf) / 524288;

  return freq;
}

static void
__sx127x_set_sf(sx127x_ctx_t* sx127x, int32_t sf)
{
  uint8_t r;

  // set the spreading factor
  sx127x_reg_read(sx127x, SX127X_REG_LORA_MODEMCONFIG2, &r);
  r &= ~SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR;
  r |= sf << SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT;
  sx127x_reg_write(sx127x, SX127X_REG_LORA_MODEMCONFIG2, r);

  // set the detection optimization magic number depending on the spreading factor
  sx127x_reg_read(sx127x, SX127X_REG_LORA_DETECTOPTIMIZATION, &r);
  r &= ~SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE;
  if(sf == 6)
  {
    r |= SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF6;
  }
  else
  {
    r |= SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF7SF12;
  }
  sx127x_reg_write(sx127x, SX127X_REG_LORA_DETECTOPTIMIZATION, r);

  // set the low data rate bit
  sx127x_reg_read(sx127x, SX127X_REG_LORA_MODEMCONFIG3, &r);
  r |= SX127X_REG_LORA_MODEMCONFIG3_LOWDATARATEOPTIMIZE;
  sx127x_reg_write(sx127x, SX127X_REG_LORA_MODEMCONFIG3, r);
}

static int32_t
__sx127x_get_sf(sx127x_ctx_t* sx127x)
{
  uint8_t config2;
  int32_t sf;

  sx127x_reg_read(sx127x, SX127X_REG_LORA_MODEMCONFIG2, &config2);
  sf = config2 >> SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT;

  return sf;
}

static int32_t
__sx127x_get_bw(sx127x_ctx_t* sx127x)
{
  uint8_t config1;
  int32_t bw;

  sx127x_reg_read(sx127x, SX127X_REG_LORA_MODEMCONFIG1, &config1);
  bw = config1 >> SX127X_REG_LORA_MODEMCONFIG1_BW_SHIFT;

  return bw;
}

static int32_t
__sx127x_get_coding_rate(sx127x_ctx_t* sx127x)
{
  uint8_t config1;
  int32_t cr;

  sx127x_reg_read(sx127x, SX127X_REG_LORA_MODEMCONFIG1, &config1);
  cr = (config1 & SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE) >> SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_SHIFT;
  return cr;
}

static int32_t
__sx127x_get_implicitheadermodeon(sx127x_ctx_t* sx127x)
{
  uint8_t config1;
  int32_t hdrmodeon;

  sx127x_reg_read(sx127x, SX127X_REG_LORA_MODEMCONFIG1, &config1);
  hdrmodeon = config1 & SX127X_REG_LORA_MODEMCONFIG1_IMPLICITHEADERMODEON;

  return hdrmodeon;
}

static void
__sx127x_set_paoutput(sx127x_ctx_t* sx127x, enum sx127x_pa paoutput)
{
  uint8_t paconfig;

  sx127x_reg_read(sx127x, SX127X_REG_PACONFIG, &paconfig);

  switch(paoutput)
  {
  case SX127X_PA_RFO:
    paconfig &= ~SX127X_REG_PACONFIG_PASELECT;
    sx127x->pa = SX127X_PA_RFO;
    break;

  case SX127X_PA_PABOOST:
    paconfig |= SX127X_REG_PACONFIG_PASELECT;
    sx127x->pa = SX127X_PA_PABOOST;
    break;
  }

  sx127x_reg_write(sx127x, SX127X_REG_PACONFIG, paconfig);
}

static enum sx127x_pa
__sx127x_get_paoutput(sx127x_ctx_t* sx127x)
{
  uint8_t paconfig, idx;

  sx127x_reg_read(sx127x, SX127X_REG_PACONFIG, &paconfig);

  idx = (paconfig & SX127X_REG_PACONFIG_PASELECT) ? 1 : 0;

  if(idx == 0)
  {
    return SX127X_PA_RFO;
  }
  return SX127X_PA_PABOOST;
}

static void
__sx127x_set_output_power(sx127x_ctx_t* sx127x, int32_t power)
{
  // FIXME not yet implemented
}

static int32_t
__sx127x_get_output_power(sx127x_ctx_t* sx127x)
{
  uint8_t paconfig;
  int32_t maxoutputpower = 17;
  int32_t outputpower;

  sx127x_reg_read(sx127x, SX127X_REG_PACONFIG, &paconfig);
  
  if(!(paconfig & SX127X_REG_PACONFIG_PASELECT))
  {
    maxoutputpower = ((paconfig & SX127X_REG_PACONFIG_MAXPOWER) >> SX127X_REG_PACONFIG_MAXPOWER_SHIFT);
  }

  outputpower = maxoutputpower - (15 - (paconfig & SX127X_REG_PACONFIG_OUTPUTPOWER));

  return outputpower;
}

static int32_t
__sx127x_tx(sx127x_ctx_t* sx127x, uint8_t* pkt, uint32_t len)
{
  __sx127x_set_opmode(sx127x, SX127X_OPMODE_STANDBY, false);

  sx127x_writepkt(sx127x, pkt, len);

  __sx127x_set_opmode(sx127x, SX127X_OPMODE_TX, false);

  return len;
}

/////////////////////////////////////////////////////////////////////////////
//
// public interfaces
//
/////////////////////////////////////////////////////////////////////////////
void
sx127x_init(void)
{
  event_register_handler(sx127x_irq_handler, DISPATCH_EVENT_SX127X_IRQ);

  for(int i = 0; i < sizeof(_sx127x_instances)/sizeof(sx127x_ctx_t); i++)
  {
    __sx127x_init(&_sx127x_instances[i]);
  }
}

void
sx127x_set_callback(sx127x_instance_t instance, sx127x_tx_complete tx, sx127x_rx_complete rx)
{
  sx127x_get(instance)->tx_cb = tx;
  sx127x_get(instance)->rx_cb = rx;
}

void
sx127x_set_modulation(sx127x_instance_t instance, enum sx127x_modulation mod)
{
  __sx127x_set_modulation(sx127x_get(instance), mod);
}

enum sx127x_modulation
sx127x_get_modulation(sx127x_instance_t instance)
{
  return __sx127x_get_modulation(sx127x_get(instance));
}

void
sx127x_set_opmode(sx127x_instance_t instance, enum sx127x_opmode opmode)
{
  __sx127x_set_opmode(sx127x_get(instance), opmode, true);
}

enum sx127x_opmode
sx127x_get_opmode(sx127x_instance_t instance)
{
  return __sx127x_get_opmode(sx127x_get(instance));
}

void
sx127x_set_carrier_freq(sx127x_instance_t instance, uint64_t freq)
{
  __sx127x_set_carrier_freq(sx127x_get(instance), freq);
}

uint32_t
sx127x_get_carrier_freq(sx127x_instance_t instance)
{
  return __sx127x_get_carrier_freq(sx127x_get(instance));
}

void
sx127x_set_sf(sx127x_instance_t instance, int32_t sf)
{
  __sx127x_set_sf(sx127x_get(instance), sf);
}

int32_t
sx127x_get_sf(sx127x_instance_t instance)
{
  return __sx127x_get_sf(sx127x_get(instance));
}

int32_t
sx127x_get_bw(sx127x_instance_t instance)
{
  return __sx127x_get_bw(sx127x_get(instance));
}

int32_t
sx127x_get_coding_rate(sx127x_instance_t instance)
{
  return __sx127x_get_coding_rate(sx127x_get(instance));
}

int32_t
sx127x_get_implicitheadermodeon(sx127x_instance_t instance)
{
  return __sx127x_get_implicitheadermodeon(sx127x_get(instance));
}

void
sx127x_set_paoutput(sx127x_instance_t instance, enum sx127x_pa paoutput)
{
  __sx127x_set_paoutput(sx127x_get(instance), paoutput);
}

enum sx127x_pa
sx127x_get_paoutput(sx127x_instance_t instance)
{
  return __sx127x_get_paoutput(sx127x_get(instance));
}

void
sx127x_set_output_power(sx127x_instance_t instance, int32_t power)
{
  __sx127x_set_output_power(sx127x_get(instance), power);
}

int32_t
sx127x_get_output_power(sx127x_instance_t instance)
{
  return __sx127x_get_output_power(sx127x_get(instance));
}

bool
sx127x_get_initialized(sx127x_instance_t instance)
{
  return sx127x_get(instance)->initialized;
}

int32_t
sx127x_tx(sx127x_instance_t instance, uint8_t* pkt, uint32_t len)
{
  return __sx127x_tx(sx127x_get(instance), pkt, len);
}
