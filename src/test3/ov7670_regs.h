#pragma once
#include <stdint.h>

#define OV7670_ADDR 0x21  // SCCB/I2C address

// Register addresses
#define REG_GAIN     0x00
#define REG_COM7     0x12
#define REG_COM3     0x0C
#define REG_COM14    0x3E
#define REG_SCALING_XSC 0x70
#define REG_SCALING_YSC 0x71
#define REG_SCALING_DCWCTR 0x72
#define REG_SCALING_PCLK_DIV 0x73
#define REG_SCALING_PCLK_DELAY 0xA2
#define REG_COM10    0x15
#define REG_CLKRC    0x11
#define REG_COM11    0x3B
#define REG_TSLB     0x3A
#define REG_COM15    0x40
#define REG_COM13    0x3D
#define REG_RGB444   0x8C
#define REG_HSTART   0x17
#define REG_HSTOP    0x18
#define REG_VSTART   0x19
#define REG_VSTOP    0x1A
#define REG_HREF     0x32
#define REG_VREF     0x03
#define REG_COM1     0x04
#define REG_AECH     0x10

// {reg, value} pairs — 0xFF,0xFF = end marker
static const uint8_t OV7670_QCIF_RGB565[][2] = {
    {REG_COM7,  0x80},  // Reset all registers
    {0xFF, 0x00},       // Delay marker (handled in code)
    {REG_CLKRC, 0x01},  // Prescaler = 2, ~10MHz with 20MHz input
    {REG_COM7,  0x04},  // RGB mode
    {REG_RGB444,0x00},  // Disable RGB444, use RGB565
    {REG_COM15, 0xD0},  // RGB565, full range
    {REG_COM3,  0x04},  // DCW enable
    {REG_COM14, 0x19},  // Manual scaling, PCLK divider=2
    {REG_SCALING_XSC,     0x3A},
    {REG_SCALING_YSC,     0x35},
    {REG_SCALING_DCWCTR,  0x11},  // Downsample by 2
    {REG_SCALING_PCLK_DIV,0xF1},  // DSP clock divider
    {REG_SCALING_PCLK_DELAY, 0x02},
    {REG_TSLB,  0x04},  // UYVY output order
    {REG_COM13, 0x88},  // Gamma enable, UV auto adjust
    {REG_COM10, 0x00},  // VSYNC/HREF polarity normal
    // QCIF window (176x144)
    {REG_HSTART,0x16},
    {REG_HSTOP, 0x04},
    {REG_HREF,  0x24},
    {REG_VSTART,0x02},
    {REG_VSTOP, 0x7A},
    {REG_VREF,  0x0A},
    {0xFF, 0xFF}  // End
};
