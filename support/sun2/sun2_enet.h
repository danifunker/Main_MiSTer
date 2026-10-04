#ifndef SUN2_ENET_H
#define SUN2_ENET_H

// Identity and Ethernet for the Sun-2 core.
//
// Identity: a Sun-2 takes its Ethernet address, serial number and so its
// hostid from a 32-byte ID PROM.  At core load, before the boot ROM, this
// sends the core one on ioctl index 64 -- games/Sun-2/boot1.rom if there is
// one, otherwise one made from this MiSTer's own Ethernet address (Sun's OUI
// 08:00:20 and the host NIC's low three octets, which are also the serial
// number) -- so two MiSTers on one network are two machines.
//
// Ethernet: the core has the Sun-2's 82586 in the fabric and a PHY that puts
// its frames in a DDR3 mailbox; this is the daemon on the other side, the
// same mailbox and host-side machinery as the NeXT core's (support/next/
// next_enet.cpp, support/minimig/minimig_a2065_ethernet.cpp) with the
// Sun-2's own magic.  Only one core runs at a time, so sharing is safe.
//
// The interface lives in core status bits [11:9] (the core's OSD "Network"):
//   0 Off, 1 eth0 (shared, BPF filtered), 2 eth1 (dedicated),
//   3 macvlan child of eth0, 4 tap0.

#define SUN2_ENET_STATUS_OPT "[11:9]"

void sun2_enet_start(void);
void sun2_enet_stop(void);
void sun2_enet_poll(void);

#endif
