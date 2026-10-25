// gw_reg_runtime.h -- gateway registration state, runtime part
#ifndef GW_REG_RUNTIME_H
#define GW_REG_RUNTIME_H

#include <stddef.h>
#include <stdint.h>

void gwRegNoteKeep(uint32_t destIp, uint32_t nowMs);
bool gwRegOnDatagram(uint32_t srcIp, const uint8_t *payload, size_t len, uint32_t nowMs);
bool gwRegBlocked(void);
void gwRegOnServerRename(const char *newCall);
void gwRegTick(void);
void gwRegBoot(void);

#endif
