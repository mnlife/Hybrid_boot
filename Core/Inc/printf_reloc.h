#ifndef PRINTF_RELOC_H
#define PRINTF_RELOC_H
#include <stdio.h>
#ifdef DEBUG
#define _DEBUG	1
#else
#define _DEBUG	0
#endif
/*
 * Output a debug text when condition "cond" is met. The "cond" should be
 * computed by a preprocessor in the best case, allowing for the best
 * optimization.
 */

#define debug_cond(cond, fmt, args...)			\
	do {						\
		if (cond)				\
			printf(fmt, ##args);	\
	} while (0)

/* Show a message if DEBUG is defined in a file */
#define debug(fmt, args...)			\
	debug_cond(_DEBUG, fmt, ##args)

void ConfigTraceSWO(void);

#endif
