/****************************************************************************
 * USB GameCube Controller Adapter (WUP-028 protocol) menu input
 *
 * Reads the official Nintendo GameCube controller adapter (and clones in
 * native mode, e.g. Mayflash set to "Wii U"), VID 057E PID 0337, through
 * the IOS /dev/usb/hid interface via libogc, and merges the first
 * connected port into userInput[0].pad the same way the WiiDRC code does.
 *
 * Protocol: one interrupt OUT byte 0x13 starts polling, then 37-byte
 * interrupt IN reports: 0x21 followed by 9 bytes per port:
 * [status, buttons1, buttons2, stickX, stickY, cstickX, cstickY, L, R]
 ***************************************************************************/
#include <gccore.h>
#include <ogc/lwp.h>
#include <ogc/usb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

#include "Controls/GCAdapter.h"
#include "GUI/gui.h"
#include "gecko.h"

#define GCA_VID 0x057E
#define GCA_PID 0x0337
#define GCA_EP_IN 0x81
#define GCA_EP_OUT 0x02
#define GCA_REPORT_SIZE 37
#define GCA_STICK_DEADZONE 15

static lwp_t AdapterThread = LWP_THREAD_NULL;
static volatile bool Running = false;
static volatile s32 AdapterFd = -1;
static volatile bool PortConnected = false;
static volatile u16 HeldButtons = 0;
static volatile s8 StickX = 0, StickY = 0, CStickX = 0, CStickY = 0;
static volatile u8 TriggerL = 0, TriggerR = 0;
static u16 PrevButtons = 0;

static u8 Report[GCA_REPORT_SIZE] ATTRIBUTE_ALIGN(32);
static u8 PollCmd[1] ATTRIBUTE_ALIGN(32) = {0x13};
static int DebugLogsLeft = 6;

static void DebugLog(const char *fmt, ...)
{
	if (DebugLogsLeft <= 0)
		return;
	DebugLogsLeft--;
	FILE *f = fopen("sd:/gcadapter.log", "a");
	if (!f)
		return;
	va_list args;
	va_start(args, fmt);
	vfprintf(f, fmt, args);
	va_end(args);
	fclose(f);
}

static u16 MapButtons(u8 b1, u8 b2)
{
	u16 buttons = 0;
	if (b1 & 0x01) buttons |= PAD_BUTTON_A;
	if (b1 & 0x02) buttons |= PAD_BUTTON_B;
	if (b1 & 0x04) buttons |= PAD_BUTTON_X;
	if (b1 & 0x08) buttons |= PAD_BUTTON_Y;
	if (b1 & 0x10) buttons |= PAD_BUTTON_LEFT;
	if (b1 & 0x20) buttons |= PAD_BUTTON_RIGHT;
	if (b1 & 0x40) buttons |= PAD_BUTTON_DOWN;
	if (b1 & 0x80) buttons |= PAD_BUTTON_UP;
	if (b2 & 0x01) buttons |= PAD_BUTTON_START;
	if (b2 & 0x02) buttons |= PAD_TRIGGER_Z;
	if (b2 & 0x04) buttons |= PAD_TRIGGER_R;
	if (b2 & 0x08) buttons |= PAD_TRIGGER_L;
	return buttons;
}

static s8 ApplyDeadzone(s16 value)
{
	if (value > GCA_STICK_DEADZONE || value < -GCA_STICK_DEADZONE)
		return value > 127 ? 127 : (value < -128 ? -128 : (s8) value);
	return 0;
}

static bool TryOpenAdapter(void)
{
	static usb_device_entry devices[8] ATTRIBUTE_ALIGN(32);
	u8 count = 0;
	s32 listRes = USB_GetDeviceList(devices, 8, USB_CLASS_HID, &count);
	if (listRes < 0 || count == 0)
	{
		// some interface versions don't want a class filter
		s32 listRes0 = USB_GetDeviceList(devices, 8, 0, &count);
		DebugLog("list(HID)=%d list(0)=%d count=%u ios=%d\n",
				 (int) listRes, (int) listRes0, count, IOS_GetVersion());
		if (listRes0 < 0)
			return false;
	}

	for (u8 i = 0; i < count; i++)
	{
		DebugLog("dev %u: id=%08x vid=%04x pid=%04x\n",
				 i, (unsigned int) devices[i].device_id, devices[i].vid, devices[i].pid);
		if (devices[i].vid != GCA_VID || devices[i].pid != GCA_PID)
			continue;

		s32 fd;
		s32 openRes = USB_OpenDevice(devices[i].device_id, GCA_VID, GCA_PID, &fd);
		DebugLog("open=%d fd=%d\n", (int) openRes, (int) fd);
		if (openRes < 0)
			return false;

		// start polling
		s32 wr = USB_WriteIntrMsg(fd, GCA_EP_OUT, sizeof(PollCmd), PollCmd);
		DebugLog("pollcmd write=%d\n", (int) wr);
		AdapterFd = fd;
		gprintf("GCAdapter: opened, fd %d\n", fd);
		return true;
	}
	return false;
}

static void *AdapterLoop(void *arg)
{
	while (Running)
	{
		if (AdapterFd < 0)
		{
			if (!TryOpenAdapter())
			{
				usleep(1000 * 1000);
				continue;
			}
		}

		s32 res = USB_ReadIntrMsg(AdapterFd, GCA_EP_IN, GCA_REPORT_SIZE, Report);
		if (res < 0)
		{
			gprintf("GCAdapter: read error %d\n", res);
			s32 fd = AdapterFd;
			AdapterFd = -1;
			PortConnected = false;
			HeldButtons = 0;
			USB_CloseDevice(&fd);
			usleep(1000 * 1000);
			continue;
		}

		if (Report[0] != 0x21)
			continue;

		// use the first port with a controller plugged in
		bool found = false;
		for (int port = 0; port < 4; port++)
		{
			const u8 *data = &Report[1 + 9 * port];
			if (!(data[0] & 0x30)) // 0x10 wired, 0x20 wireless
				continue;

			HeldButtons = MapButtons(data[1], data[2]);
			StickX = ApplyDeadzone((s16) data[3] - 128);
			StickY = ApplyDeadzone((s16) data[4] - 128);
			CStickX = ApplyDeadzone((s16) data[5] - 128);
			CStickY = ApplyDeadzone((s16) data[6] - 128);
			TriggerL = data[7];
			TriggerR = data[8];
			found = true;
			break;
		}
		PortConnected = found;
		if (!found)
			HeldButtons = 0;
	}

	if (AdapterFd >= 0)
	{
		s32 fd = AdapterFd;
		AdapterFd = -1;
		USB_CloseDevice(&fd);
	}
	return NULL;
}

void GCAdapter_Init(void)
{
	if (Running)
		return;
	s32 initRes = USB_Initialize();
	DebugLog("USB_Initialize=%d ios=%d\n", (int) initRes, IOS_GetVersion());
	if (initRes < 0)
	{
		gprintf("GCAdapter: USB_Initialize failed\n");
		return;
	}
	Running = true;
	if (LWP_CreateThread(&AdapterThread, AdapterLoop, NULL, NULL, 16 * 1024, 70) < 0)
	{
		gprintf("GCAdapter: thread creation failed\n");
		Running = false;
		AdapterThread = LWP_THREAD_NULL;
	}
}

void GCAdapter_Shutdown(void)
{
	if (!Running)
		return;
	Running = false;
	if (AdapterThread != LWP_THREAD_NULL)
	{
		LWP_JoinThread(AdapterThread, NULL);
		AdapterThread = LWP_THREAD_NULL;
	}
	PortConnected = false;
	HeldButtons = 0;
	PrevButtons = 0;
}

bool GCAdapter_Connected(void)
{
	return PortConnected;
}

void GCAdapter_ScanPads(void)
{
	u16 held = PortConnected ? HeldButtons : 0;
	u16 down = held & ~PrevButtons;
	u16 up = PrevButtons & ~held;
	PrevButtons = held;

	if (!PortConnected)
		return;

	userInput[0].pad.btns_h |= held;
	userInput[0].pad.btns_d |= down;
	userInput[0].pad.btns_u |= up;
	userInput[0].pad.stickX = StickX;
	userInput[0].pad.stickY = StickY;
	userInput[0].pad.substickX = CStickX;
	userInput[0].pad.substickY = CStickY;
	userInput[0].pad.triggerL = TriggerL;
	userInput[0].pad.triggerR = TriggerR;
}
