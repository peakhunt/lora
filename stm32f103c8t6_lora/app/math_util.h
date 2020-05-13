#ifndef __MATH_UTIL_DEF_H__
#define __MATH_UTIL_DEF_H__

#include <stdint.h>

#define __asmeq(x, y)       \
  ".ifnc " x "," y "; "     \
    ".ifnc " x y ",fpr11; "     \
      ".ifnc " x y ",r11fp; "   \
        ".ifnc " x y ",ipr12; "     \
          ".ifnc " x y ",r12ip; "   \
            ".err; "      \
          ".endif; "      \
        ".endif; "      \
      ".endif; "        \
    ".endif; "        \
  ".endif\n\t"

#define __xl "r0"
#define __xh "r1"

static inline uint32_t __div64_32(uint64_t *n, uint32_t base)
{
  register unsigned int __base      asm("r4") = base;
  register unsigned long long __n   asm("r0") = *n;
  register unsigned long long __res asm("r2");
  register unsigned int __rem       asm(__xh);
  asm(  __asmeq("%0", __xh)
    __asmeq("%1", "r2")
    __asmeq("%2", "r0")
    __asmeq("%3", "r4")
    "bl __do_div64"
    : "=r" (__rem), "=r" (__res)
    : "r" (__n), "r" (__base)
    : "ip", "lr", "cc");
  *n = __res;
  return __rem;
}

#define do_div(n, base) __div64_32(&(n), base)

#endif /* !__MATH_UTIL_DEF_H__ */
