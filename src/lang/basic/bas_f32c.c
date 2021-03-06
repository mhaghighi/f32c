/*-
 * Copyright (c) 2013 Marko Zec
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id$
 */

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <fatfs/ff.h>

#include <dev/io.h>
#include <dev/spi.h>

#include "bas.h"


#define	SRAM_BASE	0x80000000
#define	LOADER_BASE	0x800f8000

#define	LOAD_COOKIE	0x10adc0de
#define	LOADADDR	SRAM_BASE


static void
flash_read_block(char *buf, uint32_t addr, uint32_t len)
{

	spi_start_transaction(IO_SPI_FLASH);
	spi_byte(IO_SPI_FLASH, 0x0b); /* High-speed read */
	spi_byte(IO_SPI_FLASH, addr >> 16);
	spi_byte(IO_SPI_FLASH, addr >> 8);
	spi_byte(IO_SPI_FLASH, addr);
	spi_byte(IO_SPI_FLASH, 0xff); /* dummy byte, ignored */
	spi_block_in(IO_SPI_FLASH, buf, len);
}


int
bauds(void)
{
	int bauds;

	bauds = evalint();
	check();
	if (bauds < 300 || bauds > 3000000)
		error(33);	/* argument error */
	sio_setbaud(bauds);
	normret;
}


int
bas_sleep(void)
{
	int c = 0;
	int t, target;

	RDTSC(target);

	evalreal();
	check();
	if (res.f < ZERO)
		error(33);	/* argument error */

	target += (int) (res.f * 1000.0 * freq_khz);

	do {
		RDTSC(t);
		if (t < tsc_lo)
			tsc_hi++;
		tsc_lo = t;

		if(t - target > 0)
		  break;

		c = sio_getchar(0);
#ifdef __mips__
//		asm("wait"); /* Low-power mode */
#endif
	} while (c != 3);

	if (c == 3)
		trapped = 1;
	normret;
}


int
bas_exec(void)
{
	char buf[256];
	STR st;
	uint32_t *up = (void *) SRAM_BASE;
	uint8_t *cp = (void *) LOADER_BASE;
	int res_sec, sec_size, len;

	st = stringeval();
	NULL_TERMINATE(st);
	strcpy(buf, st->strval);
	len = strlen(buf);
	if (len >= 1 && buf[1] != ':') {
		if (buf[0] == '/') {
			f_getcwd(buf, sizeof(buf));
			strcpy(&buf[2], st->strval);
		} else {
			f_getcwd(buf, sizeof(buf));
			len = strlen(buf);
			buf[len++] = '/';
			strcpy(&buf[len], st->strval);
		}
	}
	FREE_STR(st);
	check();

	if (open(buf, O_RDONLY) < 0)
		sprintf(&buf[strlen(buf)], ".bin");
	if (open(buf, O_RDONLY) < 0)
		error(15);

	*up = LOAD_COOKIE;
	strcpy((void *) &up[1], buf);

	flash_read_block((void *) cp, 0, 512);
	sec_size = (cp[0xc] << 8) + cp[0xb];
	res_sec = (cp[0xf] << 8) + cp[0xe];
	if (cp[0x1fe] != 0x55 || cp[0x1ff] != 0xaa || sec_size != 4096
	    || res_sec < 2) {
		printf("Invalid boot sector\n");
		error(15);
	}

	len = sec_size * res_sec - 512;
	flash_read_block((void *) cp, 512, len);

#ifdef __mips__
	__asm __volatile__(
		".set noreorder;"
		"mtc0 $0, $12;"		/* Mask and disable all interrupts */
		"lui $4, 0x8000;"       /* stack mask */
		"lui $5, 0x0010;"       /* top of the initial stack */
		"and $29, %0, $4;"      /* clear low bits of the stack */
		"move $31, $0;"         /* return to ROM loader when done */
		"jr %0;"
		"or $29, $29, $5;"      /* set the stack pointer */
		".set reorder;"
		:
		: "r" (cp)
	);
#else /* riscv */
	/* XXX fixme! */
#endif

	/* Actually, not reached */
	normret;
}
