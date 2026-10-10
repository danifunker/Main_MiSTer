// HP 9000 Model 715: the user_io hooks.

#include <stdint.h>
#include <string.h>
#include <strings.h>

#include "../../user_io.h"
#include "../../spi.h"
#include "../../file_io.h"
#include "../sun/sun_cdrom.h"
#include "hp9k.h"

char is_hp9k()
{
	return !strcasecmp(user_io_get_core_name(0), "HP9K715")
	    || !strcasecmp(user_io_get_core_name(1), "HP9K715");
}

int hp9k_mount_hook(int index, const char *name, fileTYPE *f, int *writable)
{
	if (!is_hp9k() || index != HP9K_CD_SLOT) return 1;

	int r = sun_cdrom_mount(index, name);
	if (r == SUN_CDROM_HANDLED)
	{
		*writable = 0;
		f->size = (int64_t)sun_cdrom_size(index);
	}
	else if (r == SUN_CDROM_REJECT)
	{
		FileClose(f);
		return 0;
	}
	return 1;
}

void hp9k_unmount(int index)
{
	if (is_hp9k()) sun_cdrom_unmount(index);
}

int hp9k_sd_service(int disk, fileTYPE *f, int op, uint64_t lba, int sz, int ack)
{
	(void)f;
	if (!op || !is_hp9k() || disk != HP9K_CD_SLOT) return 0;
	if (!sun_cdrom_active(disk) && lba < SUN_CDROM_AUDIO_BLK) return 0;

	static uint8_t buf[UIO_BUFFER_SIZE];
	if (!(op & 1) || sz > (int)sizeof(buf)) return -1;

	if (!sun_cdrom_window(disk, lba, buf, sz))
		sun_cdrom_fill(disk, lba, buf, sz);
	EnableIO();
	spi_w(UIO_SECTOR_RD | ack);
	spi_block_write(buf, user_io_get_width(), sz);
	DisableIO();
	return 1;
}
