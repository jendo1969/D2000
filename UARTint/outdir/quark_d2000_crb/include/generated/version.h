#ifndef _KERNEL_VERSION_H_
#define _KERNEL_VERSION_H_

#define ZEPHYR_VERSION_CODE 67328
#define ZEPHYR_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))

#define KERNELVERSION \
0x01070000
#define KERNEL_VERSION_NUMBER     0x010700
#define KERNEL_VERSION_MAJOR      1
#define KERNEL_VERSION_MINOR      7
#define KERNEL_PATCHLEVEL         0
#define KERNEL_VERSION_STRING     "1.7.0"

#endif /* _KERNEL_VERSION_H_ */
