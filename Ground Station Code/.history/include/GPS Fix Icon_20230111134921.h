#include <Arduino.h>
const uint16_t gpsFix[] = {
    // 'satellite-icon', 10x10px
    0x2104, 0xdefb, 0x6b4d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xc618, 0xffff, 0xffff, 0x6b4d, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x3186, 0xef7d, 0xffff, 0xffdf, 0x2104, 0x73ae, 0x632c, 0x5aeb, 0x0000, 0x0000, 0x0000, 0x3186,
    0xe71c, 0xbdd7, 0x7bcf, 0x0000, 0x0000, 0x738e, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7bcf, 0x0000, 0x0000, 0x0000, 0x7bef,
    0x0000, 0x0000, 0x18c3, 0x632c, 0x6b6d, 0x8c51, 0x1082, 0x0000, 0x73ae, 0x2104, 0x0000, 0x0000, 0x8c71, 0x0000, 0x0861, 0x9cf3,
    0x8c71, 0x7bcf, 0xbdd7, 0xffdf, 0x6b4d, 0x0000, 0x2124, 0x632c, 0x0000, 0x0020, 0x632c, 0x0000, 0xe71c, 0xffff, 0xffff, 0x6b4d,
    0x5acb, 0x94b2, 0x632c, 0x0020, 0x632c, 0x0000, 0x3186, 0xef7d, 0xffff, 0xdedb, 0x7bcf, 0x5acb, 0x2124, 0x9492, 0x10a2, 0x0000,
    0x0000, 0x3186, 0xbdf7, 0x2104};