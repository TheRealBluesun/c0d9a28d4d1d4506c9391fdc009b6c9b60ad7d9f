/*
 * ethutil.h
 *
 *  Created on: Nov 25, 2018
 *      Author: novi
 */

#ifndef SRC_ETHUTIL_H_
#define SRC_ETHUTIL_H_
#include "stdint.h"

int ethAddrFromPubkey(const uint8_t const* pub_bytes, uint8_t addr_str[43]);


#endif /* SRC_ETHUTIL_H_ */
