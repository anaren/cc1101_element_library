#define CC1101_CONFIG_IOCFG2	0x29,// GDO2 output pin configuration
#define CC1101_CONFIG_IOCFG1	0x2E,// GDO1 output pin configuration
#define CC1101_CONFIG_IOCFG0	0x06,// GDO0 output pin configuration
#define CC1101_CONFIG_FIFOTHR	0x07,// RXFIFO and TXFIFO thresholds
#define CC1101_CONFIG_SYNC1	0xD3,// Sync word, high byte
#define CC1101_CONFIG_SYNC0	0x91,// Sync word, low byte
#define CC1101_CONFIG_PKTLEN	0x40,// Packet length
#define CC1101_CONFIG_PKTCTRL1	0x04,// Packet automation control
#define CC1101_CONFIG_PKTCTRL0	0x41,// Packet automation control
#define CC1101_CONFIG_ADDR	0x3B,// Device address
#define CC1101_CONFIG_CHANNR	0x00,// Channel number
#define CC1101_CONFIG_FSCTRL1	0x0C,// Frequency synthesizer control
#define CC1101_CONFIG_FSCTRL0	0x00,// Frequency synthesizer control
#define CC1101_CONFIG_FREQ2	0x21,// Frequency control word, high byte
#define CC1101_CONFIG_FREQ1	0x6B,// Frequency control word, middle byte
#define CC1101_CONFIG_FREQ0	0x24,// Frequency control word, low byte
#define CC1101_CONFIG_MDMCFG4	0x15,// Modem configuration
#define CC1101_CONFIG_MDMCFG3	0x75,// Modem configuration
#define CC1101_CONFIG_MDMCFG2	0x03,// Modem configuration
#define CC1101_CONFIG_MDMCFG1	0x21,// Modem configuration
#define CC1101_CONFIG_MDMCFG0	0xE5,// Modem configuration
#define CC1101_CONFIG_DEVIATN	0x71,// Modem deviation setting (when FSK modulation is enabled)
#define CC1101_CONFIG_MCSM2	0x07,// Main Radio Control State Machine configuration
#define CC1101_CONFIG_MCSM1	0x30,// Main Radio Control State Machine configuration
#define CC1101_CONFIG_MCSM0	0x18,// Main Radio Control State Machine configuration
#define CC1101_CONFIG_FOCCFG	0x1D,// Frequency Offset Compensation Configuration
#define CC1101_CONFIG_BSCFG	0x1C,// Bit synchronization Configuration
#define CC1101_CONFIG_AGCCTRL2	0x47,// AGC control
#define CC1101_CONFIG_AGCCTRL1	0x40, // AGC control
#define CC1101_CONFIG_AGCCTRL0	0xB0,// AGC control
#define CC1101_CONFIG_WOREVT1	0x00,// Not supported (worevt1)
#define CC1101_CONFIG_WOREVT0	0x00,// Not supported (worevt0)
#define CC1101_CONFIG_WORCTRL	0xF8,// Not supported (worctrl1)
#define CC1101_CONFIG_FREND1	0xB7,// Front end RX configuration
#define CC1101_CONFIG_FREND0	0x10,// Front end RX configuration
#define CC1101_CONFIG_FSCAL3	0xE9,// Frequency synthesizer calibration
#define CC1101_CONFIG_FSCAL2	0x2A,// Frequency synthesizer calibration
#define CC1101_CONFIG_FSCAL1	0x00,// Frequency synthesizer calibration
#define CC1101_CONFIG_FSCAL0	0x1F,// Frequency synthesizer calibration
#define CC1101_CONFIG_RCCTRL1	0x00,// Not supported (rcctrl1)
#define CC1101_CONFIG_RCCTRL0	0x00,// Not supported (rcctrl0)
#define CC1101_CONFIG_FSTEST	0x59,// Not supported (fstest)
#define CC1101_CONFIG_PTEST	0x7F,// Not supported (ptest)
#define CC1101_CONFIG_AGCTEST	0x3E,// Not supporter (agctest)
#define CC1101_CONFIG_TEST2	0x88,// Various test settings
#define CC1101_CONFIG_TEST1	0x31,// Various test settings
#define CC1101_CONFIG_TEST0	0x09 // Various test settings
