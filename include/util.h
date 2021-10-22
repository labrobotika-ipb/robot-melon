/*
 * util.h
 *
 *  Created on: Oct 7, 2021
 *      Author: sujiwo
 */

#ifndef INCLUDE_UTIL_H_
#define INCLUDE_UTIL_H_

#include <cstdint>
#include <arpa/inet.h>


inline void pack_int16(void* pB, int16_t v, bool isLittleEndian = true)
{
	if(isLittleEndian)
		v = (int16_t)htons((uint16_t)v);

	memcpy(pB, &v, sizeof(int16_t));
}

inline int16_t unpack_int16(const void* pB, bool isLittleEndian = true)
{
	int16_t v = 0;
	memcpy(&v, pB, sizeof(int16_t));

	if(isLittleEndian)
		v = (int16_t)ntohs((uint16_t)v);

	return v;
}

inline void pack_uint16(void* pB, uint16_t v, bool isLittleEndian = true)
{
	if(isLittleEndian)
		v = htons(v);

	memcpy(pB, &v, sizeof(uint16_t));
}

inline uint16_t unpack_uint16(const void* pB, bool isLittleEndian = true)
{
	uint16_t v = 0;
	memcpy(&v, pB, sizeof(uint16_t));

	if(isLittleEndian)
		v = ntohs(v);

	return v;
}

inline void pack_int32(void* pB, int32_t v, bool isLittleEndian = true)
{
	if(isLittleEndian)
		v = (int32_t)htonl((uint32_t)v);

	memcpy(pB, &v, sizeof(int32_t));
}

inline int32_t unpack_int32(const void* pB, bool isLittleEndian = true)
{
	int32_t v = 0;
	memcpy(&v, pB, sizeof(int32_t));

	if(isLittleEndian)
		v = (int32_t)ntohl((uint32_t)v);

	return v;
}

inline void pack_uint32(void* pB, uint32_t v, bool isLittleEndian = true)
{
	if(isLittleEndian)
		v = htonl(v);

	memcpy(pB, &v, sizeof(uint32_t));
}

inline uint32_t unpack_uint32(const void* pB, bool isLittleEndian = true)
{
	uint32_t v = 0;
	memcpy(&v, pB, sizeof(uint32_t));

	if(isLittleEndian)
		v = ntohl(v);

	return v;
}

inline void f2b(uint8_t *pB, float f)
{
	uint8_t* pF = (uint8_t*) &f;

	for (uint8_t i = 0; i < sizeof(float); i++)
	{
		*pB = *pF;
		pF++;
		pB++;
	}
}



#endif /* INCLUDE_UTIL_H_ */
