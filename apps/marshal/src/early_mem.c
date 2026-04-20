#include <stddef.h>
#include <stdint.h>
#include <zephyr/linker/section_tags.h>

__boot_func void arch_early_memset(void *dst, int c, size_t n)
{
	volatile uint8_t *p = (volatile uint8_t *)dst;

	while (n-- > 0U) {
		*p++ = (uint8_t)c;
	}
}

__boot_func void arch_early_memcpy(void *dst, const void *src, size_t n)
{
	volatile uint8_t *d = (volatile uint8_t *)dst;
	const volatile uint8_t *s = (const volatile uint8_t *)src;

	while (n-- > 0U) {
		*d++ = *s++;
	}
}
