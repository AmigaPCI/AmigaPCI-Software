/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2020.
 *
 * ---------------------------------------------------------------------
 *
 * Generic POSIX function emulation.
 */

#ifndef _UTILS_H
#define _UTILS_H

#define ADDR8(x)    ((uint8_t *)  ((uintptr_t)(x)))
#define ADDR16(x)   ((uint16_t *) ((uintptr_t)(x)))
#define ADDR32(x)   ((uint32_t *) ((uintptr_t)(x)))
#define VADDR8(x)    ((volatile uint8_t *)  ((uintptr_t)(x)))
#define VADDR16(x)   ((volatile uint16_t *) ((uintptr_t)(x)))
#define VADDR32(x)   ((volatile uint32_t *) ((uintptr_t)(x)))

#define BIT(x)      (1U << (x))

#define ARRAY_SIZE(x) (int)((sizeof (x) / sizeof ((x)[0])))

#define IO_BASE              0x40000000
#define BND_IO_BASE          0x42000000
#define GPIO_IDR_OFFSET      0x10  // Input Data Register offset
#define GPIO_ODR_OFFSET      0x14  // Output Data Register offset
#define BND_IO(byte, bit)    (BND_IO_BASE + ((byte) - IO_BASE) * 32 + (bit) * 4)
#define BND_ODR_TO_IDR(addr) ((addr) + (GPIO_IDR_OFFSET - GPIO_ODR_OFFSET) * 32)

/*
 * Compile-time asserts
 */
#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define CC_ASSERT(e, n) enum { \
            ASSERT_CONCAT(ASSERT_CONCAT(ASSERT_CONCAT(assert_line_, __LINE__), \
            ASSERT_CONCAT(_, __COUNTER__)), n) = 1/(!!(e)) \
        }
#define CC_ASSERT_SIZE(type, size) CC_ASSERT(sizeof (type) == (size), type)
#define CC_ASSERT_ARRAY_SIZE(type, nelem) \
        CC_ASSERT(ARRAY_SIZE(type) == (nelem), type)

/*
 * low_bit() calculates the bit position of the lowest bit set in a value.
 *           At least one bit must be set.
 */
static inline int
low_bit(uint32_t value)
{
    return (__builtin_ffs(value) - 1);
}

void reset_dfu(int in_rom);
void reset_update(void);
void reset_cpu(void);
void reset_check(void);
void get_reset_reason(void);
void show_reset_reason(void);
void identify_cpu(void);
extern char cpu_serial_str[];

extern uint8_t cold_poweron;

#endif /* _UTILS_H */
