/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __USBAUDIO_CLOCK_H
#define __USBAUDIO_CLOCK_H

int snd_usb_init_sample_rate(struct snd_usb_audio *chip, int iface,
			     struct usb_host_interface *alts,
			     struct audioformat *fmt, int rate);

<<<<<<< HEAD
int snd_usb_clock_find_source(struct snd_usb_audio *chip, int protocol,
			     int entity_id, bool validate);
=======
int snd_usb_clock_find_source(struct snd_usb_audio *chip,
			      struct audioformat *fmt, bool validate);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

#endif /* __USBAUDIO_CLOCK_H */
