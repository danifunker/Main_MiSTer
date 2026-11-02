// DEC AlphaStation 200 Ethernet bridge: the 21040 stays in the FPGA, Main is the wire.
// OSD "Network interface" (status [23:21]): 0 eth0 (default), 1 None, 2 eth1, 3 macvlan on eth0, 4 tap0.

#ifndef DECALPHA_ENET_H
#define DECALPHA_ENET_H

void decalpha_enet_start(void);
void decalpha_enet_stop(void);
void decalpha_enet_poll(void);

#endif
