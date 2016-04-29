#include "mcp9808.h"

/********************************************************************
 FileName: mcp9808.c
 Processor: PIC18 Microcontrollers
 Complier: Microchip XC8 (for PIC18)
 Company: Microchip Technology, Inc.
 This code is developed for PIC18F45K20
 Author: Rodrigo Barbosa

/** PRIVATE PROTOTYPES *********************************************/
#include <p18f45k20.h>
unsigned char i2c_write(unsigned char i2cWriteData);
unsigned char i2c_read(unsigned char ack);
void i2c_init(void);
void i2c_start(void);
void i2c_repStart(void);
void i2c_stop(void);
void i2c_wakeSlave(char adr);
float rdTemp(unsigned char addr);


/********************************************************************
 * Function Name: rdTemp
 * Return Value: float t (temperature)
 * Parameters: addr (i2c device address)
 * Description: This function communicates with the module using the
 * correct routines and calculates the temperature base on the values
 * returned by the sensor.
********************************************************************/
float rdTemp(unsigned char addr){
    unsigned char ub=0, lb=0;
    float t=0.0;
    __delay_ms(20);
    addr&=0xFE; //Set last bit as 0 - write mode
    i2c_wakeSlave(addr); //wake the module up
    i2c_write(0x05); //write command to read the temperature registers

    addr|=0x01; //Set last bit as 1 - read mode
    i2c_wakeSlave(addr); // wake the module up
    ub=i2c_read(1); //Read the upper byte, sending an acknowledge signal of 1
    lb=i2c_read(0); //Read the lower byte, sending an acknowledge signal of 0
    i2c_stop(); //Stop i2c communication
    SSPCON2bits.RCEN=0;
    ub=ub&0x1F; //get only the 5 lower bits
    if ((ub & 0x10) == 0x10){ //calculates the temperature as indicate in the data sheet
        ub = ub & 0x0F; //Clear SIGN
        t = 256.0 - (16.0 + ub/16.0);
        }
    else //TA � 0�C
        t = (ub*16.0 + lb/16.0);
    return t;
}

/********************************************************************
 * Function Name: i2c_wakeSlave
 * Return Value: void
 * Parameters: addr (i2c device address)
 * Description: This function wakes the module up by sending the
 * device address to the bus and wait for an acknowledge signal from it.
********************************************************************/

void i2c_wakeSlave(char addr){
    SSPCON2bits.SEN=1;
    while(SSPCON2bits.SEN);
    
    do{
        PIR1bits.SSPIF=0;
        while( SSPSTATbits.BF );
        SSPBUF=addr;
        while(!PIR1bits.SSPIF);
        if(!SSPCON2bits.ACKSTAT)
           return;
        while(SSPCON2bits.RSEN);
    }while(SSPCON2bits.ACKSTAT);
}

/********************************************************************
* Function Name: i2c_init
* Return Value: void
* Parameters: void
* Description: This function sets up the SSP module.
********************************************************************/
void i2c_init(void) {
    TRISC = 0b00011000;    // TRISC 3&4 (SCL & SDA) inputs
    LATC=0b00011000;
    SSPCON1 = 0x28; // enable I2C Master mode
    SSPCON2 = 0x00; // clear control bits
    SSPSTAT = 0x80; // disable slew rate control; disable SMBus
    SSPADD = 9;
    PIR1bits.SSPIF = 0;
    PIR2bits.BCLIF = 0;
}

/********************************************************************
* Function Name: i2c_start
* Return Value: void
* Parameters: void
* Description: Send I2C Start Command
********************************************************************/
void i2c_start(void) {
    PIR1bits.SSPIF = 0; //clear flag
    while (SSPSTATbits.BF ); // wait for idle condition
    SSPCON2bits.SEN = 1; // initiate START condition
    while (!PIR1bits.SSPIF) ; // wait for a flag to be set
    PIR1bits.SSPIF = 0; // clear flag
}

/********************************************************************
* Function Name: i2c_stop
* Return Value: void
* Parameters: void
* Description: Send I2C Stop command
*
********************************************************************/
void i2c_stop(void) {
    PIR1bits.SSPIF = 0; // clear flag
    while ( SSPSTATbits.BF ) ; // wait for idle condition
    SSPCON2bits.PEN = 1; // Initiate STOP condition
    while (!PIR1bits.SSPIF) ; // wait for a flag to be set
    PIR1bits.SSPIF = 0; // clear flag
}

/********************************************************************
* Function Name: i2c_write
* Return Value: Status byte for WCOL detection.
* Parameters: Single data byte for I2C2 bus.
* Description: This routine writes a single byte to the
* I2C2 bus.
********************************************************************/
unsigned char i2c_write( unsigned char i2cWriteData ) {
    PIR1bits.SSPIF = 0; // clear interrupt
    while ( SSPSTATbits.BF ) ; // wait for idle condition
    SSPBUF = i2cWriteData; // Load SSPBUF with i2cWriteData (the value to be transmitted)
    while (!PIR1bits.SSPIF) ; // wait for a flag to be set
    PIR1bits.SSPIF = 0; // clear flag
    return ( !SSPCON2bits.ACKSTAT ); // function returns '1' if transmission is acknowledged
}
/********************************************************************
* Function Name: i2c_read
* Return Value: contents of SSP2BUF register
* Parameters: ack = 1 and nak = 0
* Description: Read a byte from I2C bus and ACK/NAK device
********************************************************************/
unsigned char i2c_read(unsigned char ack)  {
    unsigned char i2cReadData;
    PIR1bits.SSPIF = 0;// clear interrupt
    while ( SSPSTATbits.BF ) ; // wait for idle condition
    SSPCON2bits.RCEN = 1; // enable receive mode
    while (!PIR1bits.SSPIF) ; // wait for a flag to be set
    PIR1bits.SSPIF = 0;// clear flag
    if(ack)
        SSPCON2bits.ACKDT=0;
    else
        SSPCON2bits.ACKDT=1;
    SSPCON2bits.ACKEN=1;
    while(SSPCON2bits.ACKEN);
    i2cReadData = SSPBUF; // Read SSPBUF and put it in i2cReadData
    return( i2cReadData ); // return the value read from SSPBUF
} 
