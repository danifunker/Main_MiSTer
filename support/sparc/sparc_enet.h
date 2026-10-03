#ifndef SPARC_ENET_H
#define SPARC_ENET_H

// Ethernet bridge for the SunSparcStation core.
//
// The core keeps the LANCE (Am7990) in the FPGA, with its descriptor rings
// and its DMA through the IOMMU; this module is the wire behind it. Frames
// cross between the FPGA and this daemon through a DDR3 mailbox (layout in
// sparc_enet.cpp), and go out through the host network layer of the A2065
// module (raw sockets, the BPF MAC filter, macvlan, tap).
//
// The mode is the core's OSD "Network" option, status bits [26:24]:
//   0 Off, 1 eth0 (shared, BPF filtered), 2 eth1 (dedicated, filtered too),
//   3 macvlan child of eth0, 4 tap0.

#define SPARC_ENET_STATUS_OPT "[26:24]"

void sparc_enet_start(void);
void sparc_enet_stop(void);
void sparc_enet_poll(void);

#endif
