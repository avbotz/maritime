#ifndef USB_H
#define USB_H

#include <stddef.h>

int setup_usb(void);
int usb_recv_ecm_packet(char *buf, size_t buf_size);
int usb_send_ecm_packet(const char *buf);

#endif /* USB_H */
