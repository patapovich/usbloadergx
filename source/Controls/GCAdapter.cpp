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
#include <ogc/ipc.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

#include "Controls/GCAdapter.h"
#include "GUI/gui.h"
#include "gecko.h"

#define GCA_VID 0x057E
#define GCA_PID 0x0337
#define GCA_REPORT_SIZE 37
#define GCA_STICK_DEADZONE 15

// raw /dev/usb/hid v5 ioctls, see wiibrew /dev/usb/hid_(v5)
#define HIDV5_IOCTL_GETVERSION 0
#define HIDV5_IOCTL_GETDEVICECHANGE 1
#define HIDV5_IOCTL_GETDEVICEINFO 3
#define HIDV5_IOCTL_ATTACH 4
#define HIDV5_IOCTL_RELEASE 5
#define HIDV5_IOCTL_ATTACHFINISH 6
#define HIDV5_IOCTL_SUSPENDRESUME 0x10
#define HIDV5_IOCTL_INTRMSG 0x13

static lwp_t AdapterThread = LWP_THREAD_NULL;
static volatile bool Running = false;
static volatile s32 HidFd = -1;
static u32 DeviceId = 0;
static u8 EpIn = 0x81, EpOut = 0x02;
static volatile bool PortConnected = false;
static volatile u16 HeldButtons = 0;
static volatile s8 StickX = 0, StickY = 0, CStickX = 0, CStickY = 0;
static volatile u8 TriggerL = 0, TriggerR = 0;
static u16 PrevButtons = 0;

static u8 Report[GCA_REPORT_SIZE] ATTRIBUTE_ALIGN(32);
static u8 PollCmd[32] ATTRIBUTE_ALIGN(32);
static u8 IoBuf[0x180] ATTRIBUTE_ALIGN(32);
static u8 InfoBuf[0x60] ATTRIBUTE_ALIGN(32);
static u8 MsgBuf[32] ATTRIBUTE_ALIGN(32);
static ioctlv IntrVec[2] ATTRIBUTE_ALIGN(32);
static int DebugLogsLeft = 10;

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

static s32 DeviceIoctl(u8 ioctl, const void *extra, u8 extraLen, void *out, u32 outLen)
{
	memset(MsgBuf, 0, sizeof(MsgBuf));
	*(u32 *) MsgBuf = DeviceId;
	if (extra && extraLen)
		memcpy(MsgBuf + 8, extra, extraLen);
	return IOS_Ioctl(HidFd, ioctl, MsgBuf, 0x20, out, outLen);
}

static s32 IntrTransfer(u32 out_dir, void *data, u16 len)
{
	static u8 msg[32] ATTRIBUTE_ALIGN(32);
	memset(msg, 0, sizeof(msg));
	*(u32 *) (msg + 0) = DeviceId;
	*(u32 *) (msg + 8) = out_dir; // non-zero: interrupt OUT, zero: IN

	IntrVec[0].data = msg;
	IntrVec[0].len = 32;
	IntrVec[1].data = data;
	IntrVec[1].len = len;

	if (out_dir)
		return IOS_Ioctlv(HidFd, HIDV5_IOCTL_INTRMSG, 2, 0, IntrVec);
	return IOS_Ioctlv(HidFd, HIDV5_IOCTL_INTRMSG, 1, 1, IntrVec);
}

static bool TryOpenAdapter(void)
{
	s32 fd = IOS_Open("/dev/usb/hid", 0);
	if (fd < 0)
	{
		DebugLog("hid open=%d\n", (int) fd);
		return false;
	}

	u32 *ver = (u32 *) IoBuf;
	memset(IoBuf, 0, 0x20);
	s32 res = IOS_Ioctl(fd, HIDV5_IOCTL_GETVERSION, NULL, 0, IoBuf, 0x20);
	if (res < 0 || ver[0] != 0x50001)
	{
		DebugLog("getversion=%d ver=%08x\n", (int) res, (unsigned int) ver[0]);
		IOS_Close(fd);
		return false;
	}

	memset(IoBuf, 0, sizeof(IoBuf));
	res = IOS_Ioctl(fd, HIDV5_IOCTL_GETDEVICECHANGE, NULL, 0, IoBuf, 0x180);
	// unlock the manager for other handles no matter what
	IOS_Ioctl(fd, HIDV5_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0);
	if (res < 0)
	{
		DebugLog("devicechange=%d\n", (int) res);
		IOS_Close(fd);
		return false;
	}

	bool found = false;
	for (s32 i = 0; i < res && i < 32; i++)
	{
		const u8 *e = &IoBuf[i * 12];
		u16 vid = (e[4] << 8) | e[5];
		u16 pid = (e[6] << 8) | e[7];
		if (vid == GCA_VID && pid == GCA_PID)
		{
			DeviceId = *(u32 *) e;
			found = true;
			break;
		}
	}
	if (!found)
	{
		IOS_Close(fd);
		return false;
	}

	HidFd = fd;

	s32 attach = DeviceIoctl(HIDV5_IOCTL_ATTACH, NULL, 0, NULL, 0);
	u32 resumed = 1;
	s32 resume = DeviceIoctl(HIDV5_IOCTL_SUSPENDRESUME, &resumed, sizeof(resumed), NULL, 0);
	memset(InfoBuf, 0, sizeof(InfoBuf));
	s32 info = DeviceIoctl(HIDV5_IOCTL_GETDEVICEINFO, NULL, 0, InfoBuf, 0x60);
	if (info >= 0)
	{
		// interrupt endpoint descriptors at fixed offsets, address at +2
		if (InfoBuf[80 + 2])
			EpIn = InfoBuf[80 + 2];
		if (InfoBuf[88 + 2])
			EpOut = InfoBuf[88 + 2];
	}

	memset(PollCmd, 0, sizeof(PollCmd));
	PollCmd[0] = 0x13;
	s32 wr = IntrTransfer(1, PollCmd, 1);
	DebugLog("id=%08x attach=%d resume=%d info=%d ep=%02x/%02x write=%d\n",
			 (unsigned int) DeviceId, (int) attach, (int) resume, (int) info, EpIn, EpOut, (int) wr);
	if (wr < 0)
	{
		DeviceIoctl(HIDV5_IOCTL_RELEASE, NULL, 0, NULL, 0);
		IOS_Close(fd);
		HidFd = -1;
		return false;
	}
	gprintf("GCAdapter: attached, device %08x\n", (unsigned int) DeviceId);
	return true;
}

static void *AdapterLoop(void *arg)
{
	while (Running)
	{
		if (HidFd < 0)
		{
			if (!TryOpenAdapter())
			{
				usleep(1000 * 1000);
				continue;
			}
		}

		s32 res = IntrTransfer(0, Report, GCA_REPORT_SIZE);
		if (res < 0)
		{
			gprintf("GCAdapter: read error %d\n", res);
			DebugLog("read error=%d\n", (int) res);
			s32 fd = HidFd;
			HidFd = -1;
			PortConnected = false;
			HeldButtons = 0;
			if (fd >= 0)
				IOS_Close(fd);
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

	if (HidFd >= 0)
	{
		DeviceIoctl(HIDV5_IOCTL_RELEASE, NULL, 0, NULL, 0);
		IOS_Close(HidFd);
		HidFd = -1;
	}
	return NULL;
}

void GCAdapter_Init(void)
{
	if (Running)
		return;
	DebugLog("GCAdapter init, ios=%d\n", IOS_GetVersion());
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
