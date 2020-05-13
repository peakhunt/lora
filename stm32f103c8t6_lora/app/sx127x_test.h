#ifndef __SX127X_TEST_DEF_H__
#define __SX127X_TEST_DEF_H__

#include "shell.h"

extern void sx127x_test_init(void);
extern void sx127x_test_start(ShellIntf* intf);
extern void sx127x_test_stop(void);
extern void sx127x_test_tx(void);

#endif /* !__SX127X_TEST_DEF_H__ */
