#ifndef _HPSDR_H
#define _HPSDR_H 1

#include <linux/ioctl.h>

#define HPSDR_IOC_MAGIC 0xF1

//  IOCTLs for the HPSDR TX device
#define HPSDR_IOCTPREAMP	_IO(HPSDR_IOC_MAGIC, 1)
#define HPSDR_IOCQPREAMP	_IO(HPSDR_IOC_MAGIC, 2)

#endif /* hpsdr.h */
