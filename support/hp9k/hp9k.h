// HP 9000 family (HP9K715; the Series 300 later): user_io hooks, each gated on the family.
// Self-contained: nothing here depends on another core family's support code.

#ifndef HP9K_H
#define HP9K_H

#include <stdint.h>
#include "../../file_io.h"

// To add a model: a value here and a row in hp9k.cpp's model table.
enum hp9k_model_t
{
	HP9K_NONE,
	HP9K_715,
};

hp9k_model_t hp9k_model();
char is_hp9k_family();

// OSD "Network" status bits, or 0 if the model has no LAN bridge.
const char *hp9k_net_status();

// CD-ROM slot with image translation, or -1.
int hp9k_cdrom_slot();

// Returns 0 to fail the mount; a translated CD clears *writable and sets f->size.
int hp9k_mount_hook(int index, const char *name, fileTYPE *f, int *writable);
void hp9k_unmount(int index);

// 0 = not ours, 1 = serviced, -1 = ours but unsupported (caller breaks the loop).
int hp9k_sd_service(int disk, fileTYPE *f, int op, uint64_t lba, int sz, int ack);

#endif
