/* 
 * File:   mcp9808.h
 * Author: rodrigobarbosa
 *
 * Created on November 7, 2015, 3:34 PM
 */

#ifndef MCP9808_H
#define	MCP9808_H

#include <string.h>
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

// PIC LCD Frequency
#define _XTAL_FREQ  4000000

void i2c_init(void);
void i2c_start(void);
void i2c_repStart(void);
void i2c_stop(void);
void i2c_wakeSlave(char adr);
unsigned char i2c_write(unsigned char i2cWriteData);
unsigned char i2c_read(unsigned char ack);
float rdTemp(unsigned char addr);

#endif	/* MCP9808_H */