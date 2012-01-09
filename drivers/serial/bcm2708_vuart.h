#ifndef _SERIAL_VUART_H
#define _SERIAL_VUART_H

#define CBUF_LEN_LN2 8

/* circular buffer */
struct bcm2708_vuart_cbuf {
	const unsigned size_ln2;	/* should have the value CBUF_LEN_LN2 */
	unsigned in_count;	/* number of times character put in */
	unsigned out_count;	/* number of times character taken out */
	unsigned char buf[1 << CBUF_LEN_LN2];
};

#define VUART_ID 0xcafef00d

struct bcm2708_vuart {
	struct bcm2708_vuart_cbuf to_host;	/* buffer written by ARM read by VC */
	struct bcm2708_vuart_cbuf from_host;	/* buffer written by VC read by ARM */
	unsigned sentinal;	/* should have the value VUART_ID 0xcafef00d */
};

/*! Read character from circular buffer or return EOF if none available */
extern int /*char or EOF */ cbuf_rd_poll(volatile struct bcm2708_vuart_cbuf *cb);

/*! Write character to circular buffer and return TRUE else return FALSE */
extern int /*bool */ cbuf_wr_poll(volatile struct bcm2708_vuart_cbuf *cb, char ch);

#endif /* _SERIAL_VUART_H */
