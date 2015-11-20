/*
 * EEPROM. Non-volatile memory that houses the firmware. Describes parameters, including PID.
 *
 */

#ifndef EEPROM_H_                    //Checks to see if EEPROM_H_ has been #defined earlier in this file, or in an included file. If not, it includes the code below.
#define EEPROM_H_

void readGlobalSet();
bool readEEPROM();
void update_constants();
void writeGlobalSet(uint8_t b);
void writeParams(uint8_t b);
void LoadDefaults();
void readPLog(void);
void writePLog(void);

#endif /* EEPROM_H_ */
