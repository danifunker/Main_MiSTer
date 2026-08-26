// tcpsnoop — minimal AF_PACKET TCP tracer for the MacLC eth bring-up.
// Prints one line per TCP segment involving the target IP: time, dir,
// ports, flags, seq/ack (relative once seen), payload length, window.
// Build (same toolchain as Main):
//   arm-none-linux-gnueabihf-gcc -O2 -static -o tcpsnoop tcpsnoop.c
// Run:  ./tcpsnoop 192.168.99.42 [port-filter]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <arpa/inet.h>
#include <sys/time.h>

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <ip> [port]\n", argv[0]); return 1; }
    uint32_t target = ntohl(inet_addr(argv[1]));
    int portf = argc > 2 ? atoi(argv[2]) : 0;

    int s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_IP));
    if (s < 0) { perror("socket"); return 1; }

    uint8_t buf[2048];
    struct timeval t0 = {0};
    for (;;) {
        int n = recv(s, buf, sizeof buf, 0);
        if (n < 34) continue;
        const uint8_t *ip = buf + 14;
        if ((ip[0] >> 4) != 4 || ip[9] != 6) continue;   // IPv4 TCP only
        int ihl = (ip[0] & 0xf) * 4;
        uint32_t src = (ip[12]<<24)|(ip[13]<<16)|(ip[14]<<8)|ip[15];
        uint32_t dst = (ip[16]<<24)|(ip[17]<<16)|(ip[18]<<8)|ip[19];
        if (src != target && dst != target) continue;
        const uint8_t *tcp = ip + ihl;
        int sport = (tcp[0]<<8)|tcp[1], dport = (tcp[2]<<8)|tcp[3];
        if (portf && sport != portf && dport != portf) continue;
        int doff = (tcp[12] >> 4) * 4;
        int iplen = (ip[2]<<8)|ip[3];
        int paylen = iplen - ihl - doff;
        uint32_t seq = (tcp[4]<<24)|(tcp[5]<<16)|(tcp[6]<<8)|tcp[7];
        uint32_t ack = (tcp[8]<<24)|(tcp[9]<<16)|(tcp[10]<<8)|tcp[11];
        int win = (tcp[14]<<8)|tcp[15];
        uint8_t fl = tcp[13];

        struct timeval tv; gettimeofday(&tv, NULL);
        if (!t0.tv_sec) t0 = tv;
        double t = (tv.tv_sec - t0.tv_sec) + (tv.tv_usec - t0.tv_usec) / 1e6;
        printf("%9.3f %s %5d>%-5d %c%c%c%c%c seq=%u ack=%u len=%d win=%d\n",
               t, src == target ? "GUEST>" : ">GUEST", sport, dport,
               (fl&0x02)?'S':'-', (fl&0x10)?'A':'-', (fl&0x08)?'P':'-',
               (fl&0x01)?'F':'-', (fl&0x04)?'R':'-',
               seq, ack, paylen, win);
        fflush(stdout);
    }
}
