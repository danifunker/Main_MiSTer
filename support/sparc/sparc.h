// SunSparcStation core (SPARCstation 5/20) -- HPS-side services.

#ifndef SPARC_H
#define SPARC_H

// The core: CONF_STR name "SunSparcStation" (both machines).
char is_sparc();

// The CD-ROM's hps_io slot (SunSparcStation.sv: SC2). CUE/CHD images are
// served as a flat disc of 2048-byte sectors by the Mac CD-ROM module.
#define SPARC_CDROM_SLOT 2

#endif
