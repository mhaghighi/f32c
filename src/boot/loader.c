
#include <types.h>
#include <io.h>
#include <sio.h>

typedef int mainfn_t(void);

static char *msg = "\nulxp2> ";

int
main(void)
{
	mainfn_t *bootaddr = NULL;
	int *loadaddr = NULL;
	int cur_bits = 0;
	int cur_word = 0;	/* Appease gcc -Wall */
	int c;
	char *cp;
	
	do {
		do {
			INW(c, IO_SIO);
		} while ((c & SIO_RX_BYTES) == 0);
		c = (c >> 8) & 0xff;

		OUTB(IO_SIO, c);

		if (c == '\r') {
			if (loadaddr == NULL) {
				if (bootaddr != NULL)
					bootaddr();
				else
					for (cp = msg; *cp != 0; cp++) {
						do {
							INW(c, IO_SIO);
						} while (c & SIO_TX_BUSY);
						OUTB(IO_SIO, *cp);
					}
			} else
				loadaddr = NULL;
			continue;
		}

		if ((c >= '0' && c <= '9') || (c >= 'a'  && c <= 'f')) {
			if (c >= 'a')
				c = c - 'a' + 10;
			else
				c = c - '0';
			cur_word = (cur_word << 4) | (c ^ 0x0f);
			cur_bits += 4;
			if (cur_bits == 32) {
				cur_bits = 0;
				if (loadaddr == NULL) {
					loadaddr = (int *) cur_word;
					if (bootaddr == NULL)
						bootaddr =
						    (mainfn_t *) loadaddr;
				} else {
					*loadaddr++ = cur_word;
				}
			}
		}
	} while (1);
}

