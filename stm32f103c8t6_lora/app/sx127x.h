#ifndef __SX127X_DEF_H__
#define __SX127X_DEF_H__

#include "app_common.h"
#include <stdint.h>

typedef enum
{
  sx127x_instance_0 = 0,
} sx127x_instance_t;

#define SX127X_NUM_INSTANCE   (sx127x_instance_0 + 1)

enum sx127x_modulation
{
  SX127X_MODULATION_FSK,
  SX127X_MODULATION_OOK,
  SX127X_MODULATION_LORA,
  SX127X_MODULATION_INVALID
};

enum sx127x_opmode
{
  SX127X_OPMODE_SLEEP,
  SX127X_OPMODE_STANDBY,
  SX127X_OPMODE_FSTX,
  SX127X_OPMODE_TX,
  SX127X_OPMODE_FSRX,
  SX127X_OPMODE_RX,
  SX127X_OPMODE_RXCONTINUOS,
  SX127X_OPMODE_RXSINGLE,
  SX127X_OPMODE_CAD
};

enum sx127x_pa
{
  SX127X_PA_RFO,
  SX127X_PA_PABOOST
};

typedef struct
{
  size_t len;
  size_t hdrlen;
  size_t payloadlen;

  int16_t   snr;
  int16_t   rssi;
  uint32_t  fei;
  uint8_t   crcfail;
} sx127x_pkt_t;

//
// callback type defunitions
//
typedef void (*sx127x_tx_complete)(sx127x_instance_t instance);
typedef void (*sx127x_rx_complete)(sx127x_instance_t instance, sx127x_pkt_t* pkt, uint8_t* buf);

extern void sx127x_init(void);

extern void sx127x_set_callback(sx127x_instance_t instance, sx127x_tx_complete tx, sx127x_rx_complete rx);

extern void sx127x_set_modulation(sx127x_instance_t instance, enum sx127x_modulation mod);
extern enum sx127x_modulation sx127x_get_modulation(sx127x_instance_t instance);

extern void sx127x_set_opmode(sx127x_instance_t instance, enum sx127x_opmode opmode);
extern enum sx127x_opmode sx127x_get_opmode(sx127x_instance_t instance);

extern void sx127x_set_carrier_freq(sx127x_instance_t instance, uint64_t freq);
extern uint32_t sx127x_get_carrier_freq(sx127x_instance_t instance);

extern void sx127x_set_sf(sx127x_instance_t instance, int32_t sf);
extern int32_t sx127x_get_sf(sx127x_instance_t instance);

extern int32_t sx127x_get_bw(sx127x_instance_t instance);

extern int32_t sx127x_get_coding_rate(sx127x_instance_t instance);

extern int32_t sx127x_get_implicitheadermodeon(sx127x_instance_t instance);

extern void sx127x_set_paoutput(sx127x_instance_t instance, enum sx127x_pa paoutput);
extern enum sx127x_pa sx127x_get_paoutput(sx127x_instance_t instance);

extern void sx127x_set_output_power(sx127x_instance_t instance, int32_t power);
extern int32_t sx127x_get_output_power(sx127x_instance_t instance);

extern bool sx127x_get_initialized(sx127x_instance_t instance);

extern int32_t sx127x_tx(sx127x_instance_t instance, uint8_t* pkt, uint32_t len);


#endif /* !__SX127X_DEF_H__ */
