/*
 * drift_util.h - Utilitats de memòria sense dependre de libc (només C99 + libm).
 *
 * memset/memcpy es substitueixen per bucles de bytes per no enllaçar amb
 * string.h. El compilador pot emetre memcpy en còpies de structs grans; per
 * això la còpia de tracks és explícita (drift_copy).
 */
#ifndef DRIFT_UTIL_H
#define DRIFT_UTIL_H

#include <stddef.h>

static inline void drift_zero(void *p, size_t n)
{
    unsigned char *b = (unsigned char *)p;
    while (n-- > 0) {
        *b++ = 0u;
    }
}

static inline void drift_copy(void *dst, const void *src, size_t n)
{
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    while (n-- > 0) {
        *d++ = *s++;
    }
}

#endif /* DRIFT_UTIL_H */
