#include <string.h>
#include "sx127x.h"
#include "sx127x_test.h"
#include "event_dispatcher.h"
#include "event_list.h"

#define SX127x_MAX_PKT    4

typedef struct
{
  sx127x_pkt_t    pkt;
  uint8_t         buf[128];
} sx127x_test_pkt_t;

static ShellIntf*           _shell_if = NULL;
static sx127x_test_pkt_t    _pkt_buf[SX127x_MAX_PKT];
static int32_t              _num_pkt = 0;
static bool                 _tx_in_prog = false;

static void
hex_print(uint8_t* buf, int len)
{
  for(int i = 0; i < len; )
  {
    uint8_t chunk_size;

    chunk_size= (i + 8) <= len ? 8 : (len - i);

    for(uint8_t j = 0; j < chunk_size; j++)
    {
      shell_printf(_shell_if, " %02x", buf[i + j]);
    }
    shell_printf(_shell_if, "\r\n");

    i += chunk_size;
  }
}

static void 
sx127x_tx_complete_callback(sx127x_instance_t instance)
{
  _tx_in_prog = false;
  event_set(1 << DISPATCH_EVENT_SX127X_TX);
}

static void
sx127x_rx_complete_callback(sx127x_instance_t instance, sx127x_pkt_t* pkt, uint8_t* buf)
{
  if(_num_pkt >= SX127x_MAX_PKT)
  {
    if(_shell_if != NULL)
    {
      shell_printf(_shell_if, "\r\nsx127x_rx_complete_callback buffer overflow\r\n");
    }
  }
  else
  {
    sx127x_test_pkt_t* tpkt;

    tpkt = &_pkt_buf[_num_pkt];
    memcpy(&tpkt->pkt, pkt, sizeof(sx127x_pkt_t));
    memcpy(tpkt->buf, buf, pkt->payloadlen);

    _num_pkt++;
    event_set(1 << DISPATCH_EVENT_SX127X_RX);
  }
}

static void
sx127x_test_rx_comp(uint32_t event)
{
  sx127x_test_pkt_t* p;

  for(int i = 0; i < _num_pkt; i++)
  {
    p = &_pkt_buf[i];

    if(_shell_if != NULL)
    {
      shell_printf(_shell_if, "\r\n");
      shell_printf(_shell_if, "===== RX Packet ======\r\n");
      shell_printf(_shell_if, "len:         %d\r\n", p->pkt.len);
      shell_printf(_shell_if, "hdrlen:      %d\r\n", p->pkt.hdrlen);
      shell_printf(_shell_if, "payloadlen:  %d\r\n", p->pkt.payloadlen);
      shell_printf(_shell_if, "snr:         %d\r\n", p->pkt.snr);
      shell_printf(_shell_if, "rssi:        %d\r\n", p->pkt.rssi);
      shell_printf(_shell_if, "fei:         %ld\r\n", p->pkt.fei);
      shell_printf(_shell_if, "crcfail:     %d\r\n", p->pkt.crcfail);

      hex_print(p->buf, p->pkt.payloadlen);
    }
  }
  _num_pkt = 0;
}

static void
sx127x_test_tx_comp(uint32_t event)
{
  if(_shell_if != NULL)
  {
    shell_printf(_shell_if, "\r\nsx127x_tx_complete_callback\r\n");
  }
  _tx_in_prog = false;
}

static void sx127x_setup_radio(void)
{
  sx127x_set_paoutput(sx127x_instance_0, SX127X_PA_PABOOST);
  sx127x_set_modulation(sx127x_instance_0, SX127X_MODULATION_LORA);
  sx127x_set_carrier_freq(sx127x_instance_0, 920000000);
  sx127x_set_sf(sx127x_instance_0, 12);
  sx127x_set_opmode(sx127x_instance_0, SX127X_OPMODE_RXCONTINUOS);
}

void
sx127x_test_start(ShellIntf* intf)
{
  _shell_if   = intf;
  _num_pkt    = 0;
  _tx_in_prog = false;

  sx127x_setup_radio();
  sx127x_set_callback(sx127x_instance_0, sx127x_tx_complete_callback, sx127x_rx_complete_callback);
}

void
sx127x_test_stop(void)
{
  _shell_if = NULL;
  sx127x_set_callback(sx127x_instance_0, NULL, NULL);
}

void
sx127x_test_init(void)
{
  event_register_handler(sx127x_test_rx_comp, DISPATCH_EVENT_SX127X_RX);
  event_register_handler(sx127x_test_tx_comp, DISPATCH_EVENT_SX127X_TX);
}

void
sx127x_test_tx(void)
{
  static uint8_t    data[16] = 
  {
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x01, 0x20, 0x03, 0x40, 0x05, 0x60, 0x07, 0x80,
  };

  if(_shell_if == NULL)
  {
    return;
  }

  if(_tx_in_prog)
  {
    shell_printf(_shell_if, "tx in progress\r\n");
    return;
  }

  sx127x_tx(sx127x_instance_0, data, 16);
}
