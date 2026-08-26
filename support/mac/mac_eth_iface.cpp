// Network side of the MacLC PDS Ethernet service — port of the standalone
// daemon's enet_iface.c (MacLC_MiSTer hps/maclc_eth), itself a lean port of
// support/minimig/minimig_a2065_ethernet.cpp. Two modes by interface name:
//   "tapN"     TUN/TAP (IFF_TAP|IFF_NO_PI): private subnet / NAT on the
//              MiSTer side; the only mode that works over WiFi.
//   anything   AF_PACKET raw socket bound to that interface, promiscuous
//   else       while open. Use a macvlan child to give the Mac its own MAC
//              on the LAN without disturbing the MiSTer's address.
// Frame filtering (our MAC / broadcast / multicast / CAM) is done by the
// SONIC model, so no BPF here. SOCK_CLOEXEC everywhere: leaked promiscuous
// sockets outlive the card and keep queueing frames (a2065 lesson).

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/if_tun.h>
#include <linux/ethtool.h>
#include <linux/sockios.h>
#include <arpa/inet.h>

#include "mac_eth.h"

static int sock_fd = -1;

int mac_eth_iface_fd(void) { return sock_fd; }

static void iface_up(const char *name)
{
	int s = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
	if (s < 0) return;
	struct ifreq ifr;
	memset(&ifr, 0, sizeof ifr);
	strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
	if (ioctl(s, SIOCGIFFLAGS, &ifr) == 0 && !(ifr.ifr_flags & IFF_UP)) {
		ifr.ifr_flags |= IFF_UP;
		ioctl(s, SIOCSIFFLAGS, &ifr);
	}
	close(s);
}

static int open_tap(const char *name)
{
	int fd = open("/dev/net/tun", O_RDWR | O_CLOEXEC | O_NONBLOCK);
	if (fd < 0) { printf("mac_eth: open /dev/net/tun failed\n"); return 0; }

	struct ifreq ifr;
	memset(&ifr, 0, sizeof ifr);
	ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
	strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
	if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
		printf("mac_eth: TUNSETIFF %s failed\n", name);
		close(fd);
		return 0;
	}
	sock_fd = fd;
	iface_up(ifr.ifr_name);
	printf("mac_eth: opened tap %s\n", ifr.ifr_name);
	return 1;
}

// The kernel's GRO (generic receive offload) coalesces back-to-back TCP
// segments into super-frames BEFORE an AF_PACKET tap sees them - so under
// any burst the tap received 2920+-byte "frames" that no real wire carries.
// The SONIC model rightly refuses frames over 1518 bytes, so every
// coalesced burst toward the guest was silently lost while solitary
// segments (retransmits, ACKs, ARP, ping) passed - Linux TCP answered the
// pattern with an RTO-backoff spiral: the 2-4 KB/s FTP crawl, root-caused
// 2026-08-26 with a two-point capture (tcpsnoop) after every deeper layer
// measured healthy. Switch GRO off on the tapped interface so frames
// arrive wire-sized; on a macvlan/vlan child GRO runs on the parent, so
// walk lower_* and switch it off there too.
static void iface_gro_off(int fd, const char *name, int depth)
{
	struct ethtool_value ev;
	struct ifreq ifr;
	memset(&ifr, 0, sizeof ifr);
	strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
	ev.cmd  = ETHTOOL_SGRO;
	ev.data = 0;
	ifr.ifr_data = (char *)&ev;
	if (ioctl(fd, SIOCETHTOOL, &ifr) == 0)
		printf("mac_eth: GRO off on %s\n", name);

	if (depth >= 2) return;   // lower_ chains are short; guard anyway
	char path[128];
	snprintf(path, sizeof path, "/sys/class/net/%s", name);
	DIR *d = opendir(path);
	if (!d) return;
	struct dirent *e;
	while ((e = readdir(d)))
		if (!strncmp(e->d_name, "lower_", 6))
			iface_gro_off(fd, e->d_name + 6, depth + 1);
	closedir(d);
}

static int open_raw(const char *name)
{
	struct ifreq ifr;
	struct sockaddr_ll addr;

	sock_fd = socket(AF_PACKET, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, htons(ETH_P_ALL));
	if (sock_fd < 0) { printf("mac_eth: raw socket failed\n"); return 0; }

	memset(&ifr, 0, sizeof ifr);
	strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
	if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) {
		printf("mac_eth: no such interface %s\n", name);
		close(sock_fd); sock_fd = -1;
		return 0;
	}
	int ifindex = ifr.ifr_ifindex;

	iface_up(name);

	memset(&addr, 0, sizeof addr);
	addr.sll_family   = AF_PACKET;
	addr.sll_protocol = htons(ETH_P_ALL);
	addr.sll_ifindex  = ifindex;
	if (bind(sock_fd, (struct sockaddr *)&addr, sizeof addr) < 0) {
		printf("mac_eth: bind %s failed\n", name);
		close(sock_fd); sock_fd = -1;
		return 0;
	}

	struct packet_mreq mreq;
	memset(&mreq, 0, sizeof mreq);
	mreq.mr_ifindex = ifindex;
	mreq.mr_type    = PACKET_MR_PROMISC;
	// Absorb bursts: the service drains on a ~1 ms poll tick, so the default
	// (~160 KB) buffer overflows if a pass runs long. Dropped frames cost far
	// more than the memory - TCP reacts to loss by collapsing its window.
	int rcvbuf = 1 << 20;
	setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof rcvbuf);

	if (setsockopt(sock_fd, SOL_PACKET, PACKET_ADD_MEMBERSHIP, &mreq, sizeof mreq) < 0)
		printf("mac_eth: promisc membership on %s failed\n", name);

	iface_gro_off(sock_fd, name, 0);

	printf("mac_eth: opened raw %s (ifindex %d, promisc)\n", name, ifindex);
	return 1;
}

int mac_eth_iface_open(const char *name)
{
	if (sock_fd >= 0) mac_eth_iface_close();
	if (!strncmp(name, "tap", 3)) return open_tap(name);
	return open_raw(name);
}

void mac_eth_iface_close(void)
{
	if (sock_fd >= 0) close(sock_fd);
	sock_fd = -1;
}

int mac_eth_iface_send(const uint8_t *frame, int len)
{
	if (sock_fd < 0) return -1;
	int n = write(sock_fd, frame, len);
	if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
		printf("mac_eth: send errno=%d\n", errno);
	return n;
}

int mac_eth_iface_recv(uint8_t *buf, int maxlen)
{
	if (sock_fd < 0) return -1;
	int n = read(sock_fd, buf, maxlen);
	if (n < 0) return -1;   // EAGAIN: nothing pending
	return n;
}

// Kernel-side drop count for this socket since the previous call (reading
// PACKET_STATISTICS resets it). Frames the kernel had nowhere to put.
int mac_eth_iface_drops(void)
{
	if (sock_fd < 0) return 0;
	struct tpacket_stats ps;
	socklen_t len = sizeof ps;
	if (getsockopt(sock_fd, SOL_PACKET, PACKET_STATISTICS, &ps, &len) < 0) return 0;
	return (int)ps.tp_drops;
}
