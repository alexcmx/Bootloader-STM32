/*
 * FirmwareStruct.h
 *
 *  Created on: May 24, 2023
 *      Author: user
 */

#ifndef INC_FIRMWARESTRUCT_H_
#define INC_FIRMWARESTRUCT_H_
#include <stdint.h>

struct FirmwareHeader {
	uint16_t version;
	uint16_t num_blocks;
	uint8_t blocksize;
	uint8_t crc;
};


uint8_t gencrc(void *data, size_t len)
{
    uint8_t crc = 0xff;
    size_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= ((uint8_t *)data)[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}

uint8_t calc_crc_header(FirmwareHeader * header)
{
	return gencrc(header, sizeof(*header) - 1);
}

bool check_crc_header(FirmwareHeader * header)
{
	if ( 0x00 != gencrc(header, sizeof(*header)))
	{
		printf("CRC %02X %02X", gencrc(header, sizeof(*header)-1),header->crc);
	}
	return 0x00 == gencrc(header, sizeof(*header));
}

struct FirmwareBlock {
	uint8_t *data;
	uint8_t crc;
};



#endif /* INC_FIRMWARESTRUCT_H_ */
