// SunSparcStation core (SPARCstation 5/20) -- HPS-side services.

#include <strings.h>

#include "../../user_io.h"
#include "sparc.h"

char is_sparc()
{
	return !strcasecmp(user_io_get_core_name(1), "SunSparcStation");
}
