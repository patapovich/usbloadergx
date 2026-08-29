/****************************************************************************
 * USB GameCube Controller Adapter (WUP-028 protocol) menu input
 *
 * Supports the official Nintendo adapter and clones in native mode
 * (Mayflash "Wii U" switch position), VID 057E PID 0337.
 ***************************************************************************/
#ifndef GC_ADAPTER_H_
#define GC_ADAPTER_H_

#include <gctypes.h>

void GCAdapter_Init(void);
void GCAdapter_Shutdown(void);
bool GCAdapter_Connected(void);
//! Merge the adapter state into userInput[0].pad, call from UpdatePads()
void GCAdapter_ScanPads(void);

#endif
