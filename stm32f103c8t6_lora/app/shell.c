#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

#include "stm32f1xx_hal.h"

#include "app_common.h"
#include "shell.h"
#include "shell_if_usb.h"

#include "sx127x.h"
#include "sx127x_test.h"

////////////////////////////////////////////////////////////////////////////////
//
// private definitions
//
////////////////////////////////////////////////////////////////////////////////

#define SHELL_MAX_COLUMNS_PER_LINE      128
#define SHELL_COMMAND_MAX_ARGS          4

#define VERSION       "STM32F1 Shell V0.3a"

typedef void (*shell_command_handler)(ShellIntf* intf, int argc, const char** argv);

typedef struct
{
  const char*           command;
  const char*           description;
  shell_command_handler handler;
} ShellCommand;

////////////////////////////////////////////////////////////////////////////////
//
// private prototypes
//
////////////////////////////////////////////////////////////////////////////////
static void shell_command_help(ShellIntf* intf, int argc, const char** argv);
static void shell_command_version(ShellIntf* intf, int argc, const char** argv);
static void shell_command_uptime(ShellIntf* intf, int argc, const char** argv);
static void shell_command_sx127x(ShellIntf* intf, int argc, const char** argv);

////////////////////////////////////////////////////////////////////////////////
//
// private variables
//
////////////////////////////////////////////////////////////////////////////////
const uint8_t                 _welcome[] = "\r\n**** Welcome ****\r\n";
const uint8_t                 _prompt[]  = "\r\nSTM32F1> ";

static char                   _print_buffer[SHELL_MAX_COLUMNS_PER_LINE + 1];

static LIST_HEAD_DECL(_shell_intf_list);

static ShellCommand     _commands[] = 
{
  {
    "help",
    "show this command",
    shell_command_help,
  },
  {
    "version",
    "show version",
    shell_command_version,
  },
  {
    "uptime",
    "show system uptime",
    shell_command_uptime,
  },
  {
    "sx127x",
    "sx17x command",
    shell_command_sx127x,
  },
};

////////////////////////////////////////////////////////////////////////////////
//
// shell utilities
//
////////////////////////////////////////////////////////////////////////////////
static inline void
shell_prompt(ShellIntf* intf)
{
  intf->put_tx_data(intf, (uint8_t*)_prompt, sizeof(_prompt) -1);
}

////////////////////////////////////////////////////////////////////////////////
//
// shell command handlers
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_command_help(ShellIntf* intf, int argc, const char** argv)
{
  size_t i;

  shell_printf(intf, "\r\n");

  for(i = 0; i < sizeof(_commands)/sizeof(ShellCommand); i++)
  {
    shell_printf(intf, "%-20s: ", _commands[i].command);
    shell_printf(intf, "%s\r\n", _commands[i].description);
  }
}

static void
shell_command_version(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "%s\r\n", VERSION);
}

static void
shell_command_uptime(ShellIntf* intf, int argc, const char** argv)
{
  shell_printf(intf, "\r\n");
  shell_printf(intf, "System Uptime: %lu\r\n", __uptime);
}

static void
__sx127x_command_initialized(ShellIntf* intf, int argc, const char** argv)
{
  for(int i = 0; i < SX127X_NUM_INSTANCE; i++)
  {
    shell_printf(intf, "sx127x instance: %d, initialized: %s\r\n", i,
        sx127x_get_initialized((sx127x_instance_t)i) == true ? "true" : "false");
  }
}

static void
__sx127x_command_modulation(ShellIntf* intf, int argc, const char** argv)
{
  static const char* help = \
    "sx127x modulation\r\n" \
    "sx127x modulation [fsk|ook|lora]\r\n"
  ;
  static const char* modulation_string[] =
  {
    "fsk",
    "ook",
    "lora",
    "invalid",
  };

  enum sx127x_modulation modulation;
  bool set = false;

  if(argc > 2)
  {
    set = true;
    if(strcmp(argv[2], "fsk") == 0)
    {
      modulation = SX127X_MODULATION_FSK;
    }
    else if(strcmp(argv[2], "ook") == 0)
    {
      modulation = SX127X_MODULATION_OOK;
    }
    else if(strcmp(argv[2], "lora") == 0)
    {
      modulation = SX127X_MODULATION_LORA;
    }
    else
    {
      goto command_error;
    }
  }

  if(set)
  {
    sx127x_set_modulation(sx127x_instance_0, modulation);
    shell_printf(intf, "sx127x instance: %d, set modulation: to %s\r\n", 0, argv[2]);
  }
  else
  {
    modulation = sx127x_get_modulation(sx127x_instance_0);
    shell_printf(intf, "sx127x instance: %d, modulation: %s\r\n", 0, modulation_string[modulation]);
  }
  return;

command_error:
  shell_printf(intf, "invalid command\r\n");
  shell_printf(intf, "%s", help);
}

static void
__sx127x_command_opmode(ShellIntf* intf, int argc, const char** argv)
{
  static const char* help = \
    "sx127x opmode\r\n" \
    "sx127x opmode [sleep|standby|fstx|tx|fsrx|rx|rxcontinuous|rxsingle|cad]\r\n"
  ;
  static const char* opmode_string[] =
  {
    "sleep",
    "standby",
    "fstx",
    "tx",
    "fsrx",
    "rx",
    "rxcontinuous",
    "rxsingle",
    "cad",
  };

  enum sx127x_opmode opmode;
  bool set = false;

  if(argc > 2)
  {
    set = true;
    if(strcmp(argv[2], "sleep") == 0)
    {
      opmode = SX127X_OPMODE_SLEEP;
    }
    else if(strcmp(argv[2], "standby") == 0)
    {
      opmode = SX127X_OPMODE_STANDBY;
    }
    else if(strcmp(argv[2], "fstx") == 0)
    {
      opmode = SX127X_OPMODE_FSTX;
    }
    else if(strcmp(argv[2], "tx") == 0)
    {
      opmode = SX127X_OPMODE_TX;
    }
    else if(strcmp(argv[2], "fsrx") == 0)
    {
      opmode = SX127X_OPMODE_FSRX;
    }
    else if(strcmp(argv[2], "rx") == 0)
    {
      opmode = SX127X_OPMODE_RX;
    }
    else if(strcmp(argv[2], "rxcontinuous") == 0)
    {
      opmode = SX127X_OPMODE_RXCONTINUOS;
    }
    else if(strcmp(argv[2], "rxsingle") == 0)
    {
      opmode = SX127X_OPMODE_RXSINGLE;
    }
    else if(strcmp(argv[2], "cad") == 0)
    {
      opmode = SX127X_OPMODE_CAD;
    }
    else
    {
      goto command_error;
    }
  }

  if(set)
  {
    sx127x_set_opmode(sx127x_instance_0, opmode);
    shell_printf(intf, "sx127x instance: %d, set opmode: to %s\r\n", 0, argv[2]);
  }
  else
  {
    opmode = sx127x_get_opmode(sx127x_instance_0);
    shell_printf(intf, "sx127x instance: %d, opmode: %s\r\n", 0, opmode_string[opmode]);
  }

  return;

command_error:
  shell_printf(intf, "invalid command\r\n");
  shell_printf(intf, "%s", help);
}

static void
__sx127x_command_carrier_freq(ShellIntf* intf, int argc, const char** argv)
{
  bool set = false;
  uint64_t  freq;
  uint32_t  freq32;

  if(argc > 2)
  {
    set = true;
    freq = strtoull(argv[2], NULL, 10);
  }

  if(set)
  {
    sx127x_set_carrier_freq(sx127x_instance_0, freq);
    shell_printf(intf, "sx127x instance: %d, set carrier frequency: to %" PRIu64 "\r\n", 0, freq);
  }
  else
  {
    freq32 = sx127x_get_carrier_freq(sx127x_instance_0);
    shell_printf(intf, "sx127x instance: %d, carrier frequency: %ld \r\n", 0, freq32);
  }
}

static void
__sx127x_command_sf(ShellIntf* intf, int argc, const char** argv)
{
  int32_t sf;

  if(argc > 2)
  {
    sf = atoi(argv[2]);
    sx127x_set_sf(sx127x_instance_0, sf);
    shell_printf(intf, "sx127x instance: %d, set spreading factor: to %ld\r\n", 0, sf);
  }
  else
  {
    sf = sx127x_get_sf(sx127x_instance_0);
    shell_printf(intf, "sx127x instance: %d, spreading factor: %ld \r\n", 0, sf);
  }
}

static void
__sx127x_command_bw(ShellIntf* intf, int argc, const char** argv)
{
  int32_t bw;

  bw = sx127x_get_bw(sx127x_instance_0);
  shell_printf(intf, "sx127x instance: %d, bandwidth: %ld \r\n", 0, bw);
}

static void
__sx127x_command_coding_rate(ShellIntf* intf, int argc, const char** argv)
{
  int32_t coding_rate;

  coding_rate = sx127x_get_coding_rate(sx127x_instance_0);
  shell_printf(intf, "sx127x instance: %d, coding rate: %ld \r\n", 0, coding_rate);
}

static void
__sx127x_command_header_mode(ShellIntf* intf, int argc, const char** argv)
{
  int32_t header_mode;

  header_mode = sx127x_get_implicitheadermodeon(sx127x_instance_0);
  shell_printf(intf, "sx127x instance: %d, implicait header mode on: %ld \r\n", 0, header_mode);
}

static void
__sx127x_command_paoutput(ShellIntf* intf, int argc, const char** argv)
{
  static const char* help = \
    "sx127x paoutput\r\n" \
    "sx127x paoutput [rfo|boost]\r\n"
  ;
  static const char* paoutput_str[] = 
  {
    "rfo",
    "paboost",
  };
  enum sx127x_pa paoutput;
  bool set = false;

  if(argc > 2)
  {
    set = true;
    if(strcmp(argv[2], "rfo") == 0)
    {
      paoutput = SX127X_PA_RFO;
    }
    else if(strcmp(argv[2], "boost") == 0)
    {
      paoutput = SX127X_PA_PABOOST;
    }
    else
    {
      goto command_error;
    }
  }

  if(set)
  {
    sx127x_set_paoutput(sx127x_instance_0, paoutput);
    shell_printf(intf, "sx127x instance: %d, set paoutput: to %s\r\n", 0, argv[2]);
  }
  else
  {
    paoutput = sx127x_get_paoutput(sx127x_instance_0);
    shell_printf(intf, "sx127x instance: %d, paoutput: %s\r\n", 0, paoutput_str[paoutput]);
  }

  return;

command_error:
  shell_printf(intf, "invalid command\r\n");
  shell_printf(intf, "%s", help);
}

static void
__sx127x_command_output_power(ShellIntf* intf, int argc, const char** argv)
{
  int output_power;

  if(argc > 2)
  {
    output_power = atoi(argv[2]);
    sx127x_set_output_power(sx127x_instance_0, output_power);
    shell_printf(intf, "sx127x instance: %d, set output power: to %d\r\n", 0, output_power);
  }
  else
  {
    output_power = sx127x_get_output_power(sx127x_instance_0);
    shell_printf(intf, "sx127x instance: %d, output power: %d\r\n", 0, output_power);
  }
}

static void
__sx127x_command_test(ShellIntf* intf, int argc, const char** argv)
{
  static const char* help = \
    "sx127x test [start|stop|tx]\r\n" \
  ;

  if(argc != 3)
  {
    goto command_error;
  }

  if(strcmp(argv[2], "start") == 0)
  {
    sx127x_test_start(intf);
    shell_printf(intf, "sx127x test started\r\n");
  }
  else if(strcmp(argv[2], "stop") == 0)
  {
    sx127x_test_stop();
    shell_printf(intf, "sx127x test stopped\r\n");
  }
  else if(strcmp(argv[2], "tx") == 0)
  {
    sx127x_test_tx();
    shell_printf(intf, "sx127x test tx\r\n");
  }
  else
  {
    goto command_error;
  }
  return;

command_error:
  shell_printf(intf, "invalid command\r\n");
  shell_printf(intf, "%s", help);
}

static void
shell_command_sx127x(ShellIntf* intf, int argc, const char** argv)
{
  typedef struct
  {
    const char* cmd;
    void (*handler)(ShellIntf* intf, int argc, const char** argv);
  } sx127x_command_t;
  bool found = false;

  static sx127x_command_t commands[] = 
  {
    { "initialized",    __sx127x_command_initialized    },
    { "modulation",     __sx127x_command_modulation     },
    { "opmode",         __sx127x_command_opmode         },
    { "carrier_freq",   __sx127x_command_carrier_freq   },
    { "sf",             __sx127x_command_sf             },
    { "bw",             __sx127x_command_bw             },
    { "codingrate",     __sx127x_command_coding_rate    },
    { "headermode",     __sx127x_command_header_mode    },
    { "paoutput",       __sx127x_command_paoutput       },
    { "out_power",      __sx127x_command_output_power   },
    { "test",           __sx127x_command_test           },
  };

  static const char* help = 
    "\r\n"\
    "==== sx127x command list ===\r\n"
  ;

  shell_printf(intf, "\r\n");
  if(argc < 2)
  {
    goto command_error;
  }

  for(int i = 0; i < sizeof(commands)/sizeof(sx127x_command_t); i++)
  {
    if(strcmp(argv[1], commands[i].cmd) == 0)
    {
      commands[i].handler(intf, argc, argv);
      found = true;
    }
  }

  if(found)
  {
    return;
  }

command_error:
  shell_printf(intf, "sx127x invalid command\r\n");
  shell_printf(intf, "%s", help);
  for(int i = 0; i < sizeof(commands)/sizeof(sx127x_command_t); i++)
  {
    shell_printf(intf, "%s\r\n", commands[i].cmd);
  }
  return;
}

////////////////////////////////////////////////////////////////////////////////
//
// shell core
//
////////////////////////////////////////////////////////////////////////////////
static void
shell_execute_command(ShellIntf* intf, char* cmd)
{
  static const char*    argv[SHELL_COMMAND_MAX_ARGS];
  int                   argc = 0;
  size_t                i;
  char                  *s, *t;

  while((s = strtok_r(argc  == 0 ? cmd : NULL, " \t", &t)) != NULL)
  {
    if(argc >= SHELL_COMMAND_MAX_ARGS)
    {
      shell_printf(intf, "\r\nError: too many arguments\r\n");
      return;
    }
    argv[argc++] = s;
  }

  if(argc == 0)
  {
    return;
  }

  for(i = 0; i < sizeof(_commands)/sizeof(ShellCommand); i++)
  {
    if(strcmp(_commands[i].command, argv[0]) == 0)
    {
      shell_printf(intf, "\r\nExecuting %s\r\n", argv[0]);
      _commands[i].handler(intf, argc, argv);
      return;
    }
  }
  shell_printf(intf, "%s", "\r\nUnknown Command: ");
  shell_printf(intf, "%s", argv[0]);
  shell_printf(intf, "%s", "\r\n");
}


void
shell_printf(ShellIntf* intf, const char* fmt, ...)
{
  va_list   args;
  int       len;

  va_start(args, fmt);
  len = vsnprintf(_print_buffer, SHELL_MAX_COLUMNS_PER_LINE, fmt, args);
  va_end(args);

  intf->put_tx_data(intf, (uint8_t*)_print_buffer, len);
}


////////////////////////////////////////////////////////////////////////////////
//
// public interface
//
////////////////////////////////////////////////////////////////////////////////
void
shell_init(void)
{
  shell_if_usb_init();
}

void
shell_start(void)
{
  ShellIntf* intf;

  list_for_each_entry(intf, &_shell_intf_list, lh)
  {
    intf->put_tx_data(intf, (uint8_t*)_welcome, sizeof(_welcome) -1);
    shell_prompt(intf);
  }
}


void
shell_if_register(ShellIntf* intf)
{
  list_add_tail(&intf->lh, &_shell_intf_list);
}

void
shell_handle_rx(ShellIntf* intf)
{
  uint8_t   b;

  while(1)
  {
    if(intf->get_rx_data(intf, &b) == false)
    {
      return;
    }

    if(b != '\r' && intf->cmd_buffer_ndx < SHELL_MAX_COMMAND_LEN)
    {
      if(b == '\b' || b == 0x7f)
      {
        if(intf->cmd_buffer_ndx > 0)
        {
          shell_printf(intf, "%c%c%c", b, 0x20, b);
          intf->cmd_buffer_ndx--;
        }
      }
      else
      {
        shell_printf(intf, "%c", b);
        intf->cmd_buffer[intf->cmd_buffer_ndx++] = b;
      }
    }
    else if(b == '\r')
    {
      intf->cmd_buffer[intf->cmd_buffer_ndx++] = '\0';

      shell_execute_command(intf, (char*)intf->cmd_buffer);

      intf->cmd_buffer_ndx = 0;
      shell_prompt(intf);
    }
  }
}

struct list_head*
shell_get_intf_list(void)
{
  return &_shell_intf_list;
}
