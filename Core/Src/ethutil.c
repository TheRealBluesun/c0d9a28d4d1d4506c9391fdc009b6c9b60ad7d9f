#include "ethutil.h"
#include "stdint.h"
#include "sha3.h"
#include "hexutil.h"

static sha3_context g_sha3c = {0};

// Address length = 40, plus '0x' plus NULL termination
int ethAddrFromPubkey(const uint8_t pub_bytes[64], uint8_t addr_str[43])
{
	addr_str[0] = '0';
	addr_str[1] = 'x';
	addr_str[42] = '\0';

	// Create ETH address from key
	sha3_Init256(&g_sha3c);
	sha3_Update(&g_sha3c, pub_bytes, 64);
	uint8_t* pub_hash = (uint8_t*)sha3_Finalize(&g_sha3c);
	bin2hex(pub_hash+12, addr_str+2, 20);
	return 1;
}
