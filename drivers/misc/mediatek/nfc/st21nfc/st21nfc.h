// SPDX-License-Identifier: GPL-2.0-only
/*
 * NFC Controller Driver
 * Copyright (C) 2020 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 */

#define ST21NFC_MAGIC 0xEA

#define ST21NFC_NAME "st21nfc"
/*
 * ST21NFC power control via ioctl
 * ST21NFC_GET_WAKEUP :  poll gpio-level for Wakeup pin
 */
#define ST21NFC_GET_WAKEUP _IO(ST21NFC_MAGIC, 0x01)
#define ST21NFC_PULSE_RESET _IO(ST21NFC_MAGIC, 0x02)
#define ST21NFC_SET_POLARITY_RISING _IO(ST21NFC_MAGIC, 0x03)
#define ST21NFC_SET_POLARITY_FALLING _IO(ST21NFC_MAGIC, 0x04)
#define ST21NFC_SET_POLARITY_HIGH _IO(ST21NFC_MAGIC, 0x05)
#define ST21NFC_SET_POLARITY_LOW _IO(ST21NFC_MAGIC, 0x06)
#define ST21NFC_GET_POLARITY _IO(ST21NFC_MAGIC, 0x07)
#define ST21NFC_RECOVERY _IO(ST21NFC_MAGIC, 0x08)
#define ST21NFC_USE_ESE _IOW(ST21NFC_MAGIC, 0x09, unsigned int)

// Keep compatibility with older user applications.
#define ST21NFC_LEGACY_GET_WAKEUP _IOR(ST21NFC_MAGIC, 0x01, unsigned int)
#define ST21NFC_LEGACY_PULSE_RESET _IOR(ST21NFC_MAGIC, 0x02, unsigned int)
<<<<<<< HEAD
#define ST21NFC_LEGACY_SET_POLARITY_RISING _IOR(ST21NFC_MAGIC, 0x03, \
	unsigned int)
#define ST21NFC_LEGACY_SET_POLARITY_FALLING _IOR(ST21NFC_MAGIC, 0x04, \
	unsigned int)
=======
#define ST21NFC_LEGACY_SET_POLARITY_RISING \
  _IOR(ST21NFC_MAGIC, 0x03, unsigned int)
>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
#define ST21NFC_LEGACY_SET_POLARITY_HIGH _IOR(ST21NFC_MAGIC, 0x05, unsigned int)
#define ST21NFC_LEGACY_SET_POLARITY_LOW _IOR(ST21NFC_MAGIC, 0x06, unsigned int)
#define ST21NFC_LEGACY_GET_POLARITY _IOR(ST21NFC_MAGIC, 0x07, unsigned int)
#define ST21NFC_LEGACY_RECOVERY _IOR(ST21NFC_MAGIC, 0x08, unsigned int)

struct st21nfc_platform_data {
	unsigned int irq_gpio;
	unsigned int ena_gpio;
	unsigned int reset_gpio;
	unsigned int polarity_mode;
};

#define ST54SPI_CB_RESET_END 0
#define ST54SPI_CB_RESET_START 1
#define ST54SPI_CB_ESE_NOT_USED 2
#define ST54SPI_CB_ESE_USED 3
void st21nfc_register_st54spi_cb(void (*cb)(int, void *), void *data);
void st21nfc_unregister_st54spi_cb(void);
<<<<<<< HEAD
=======

#define ACCESS_OK(x,y,z) access_ok(x,y,z)

>>>>>>> 32022887f842 (Kernel: Xiaomi kernel changes for Redmi Note 11S Android S)
