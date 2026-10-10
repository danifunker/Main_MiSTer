// HP 9000 Model 715 (HP9K715): user_io hooks.

#ifndef HP9K_H
#define HP9K_H

#include <stdint.h>
#include "../../file_io.h"

#define HP9K_CD_SLOT 3
#define HP9K_NET_STATUS "[14:12]"

char is_hp9k();

// The CD slot is served by sun_cdrom: CUE/CHD translation, track table and CD-audio windows.
int hp9k_mount_hook(int index, const char *name, fileTYPE *f, int *writable);
void hp9k_unmount(int index);
int hp9k_sd_service(int disk, fileTYPE *f, int op, uint64_t lba, int sz, int ack);

// The 82596 LAN is bridged by sun_enet (mailbox magic HPETH001, OSD option HP9K_NET_STATUS).

#endif
