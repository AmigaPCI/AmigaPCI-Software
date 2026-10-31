/*
 * strtox()
 * Implementation based on strtoull() from NetBSD
 * December 31, 2025 cdh
 */

#include <stdint.h>
#include <limits.h>
#include "strtox.h"

#ifdef AMIGA
#define isspace(x) (((x) == ' ') || ((x) == '\t'))
#endif

/*
 * strtox
 * --------
 * Convert a string to an unsigned integer.
 */
int
strtox(const char *nptr, int base, unsigned int *value)
{
    const char *s;
    unsigned int acc;
    char c;
    unsigned int cutoff;
    int neg, any, cutlim;

    /*
     * See NetBSD strtoq for comments as to the logic used.
     */
    s = nptr;
    do {
        c = *s++;
    } while (isspace((unsigned char)c));
    if (c == '-') {
        neg = 1;
        c = *s++;
    } else {
        neg = 0;
        if (c == '+')
            c = *s++;
    }
    if ((base == 0 || base == 16) &&
        c == '0' && (*s == 'x' || *s == 'X') &&
        ((s[1] >= '0' && s[1] <= '9') ||
        (s[1] >= 'A' && s[1] <= 'F') ||
        (s[1] >= 'a' && s[1] <= 'f'))) {
        c = s[1];
        s += 2;
        base = 16;
    }
    if (base == 0)
        base = c == '0' ? 8 : 10;
    acc = any = 0;
    if (base < 2 || base > 36)
        goto noconv;

    cutoff = UINT_MAX / base;
    cutlim = UINT_MAX % base;
    for ( ; ; c = *s++) {
        if (c >= '0' && c <= '9')
            c -= '0';
        else if (c >= 'A' && c <= 'Z')
            c -= 'A' - 10;
        else if (c >= 'a' && c <= 'z')
            c -= 'a' - 10;
        else
            break;
        if (c >= base)
            break;
        if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim))
            any = -1;
        else {
            any = 1;
            acc *= base;
            acc += c;
        }
    }
    if (any < 0) {
        acc = UINT_MAX;
    } else if (!any) {
noconv:
        ;
    } else if (neg) {
        acc = -acc;
    }
    *value = acc;
    return ((any ? (s - 1) : nptr) - nptr);
}
