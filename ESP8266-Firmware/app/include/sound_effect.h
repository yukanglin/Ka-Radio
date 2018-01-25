#ifndef __SOUND_EFFECT_H__
#define __SOUND_EFFECT_H__
#pragma once
#include "c_types.h"
//ICACHE_STORE_ATTR ICACHE_RODATA_ATTR
#define ICACHE_STORE_TYPEDEF_ATTR __attribute__((aligned(4),packed))
#define ICACHE_STORE_ATTR __attribute__((aligned(4)))
#define ICACHE_RAM_ATTR __attribute__((section(".iram0.text")))
extern const  char sound_effect1[] STORE_ATTR ICACHE_RODATA_ATTR;
extern const long sound_effect1_size;
#endif