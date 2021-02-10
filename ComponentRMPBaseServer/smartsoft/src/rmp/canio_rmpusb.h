#ifndef _RMPUSB_CANLIB_
#define _RMPUSB_CANLIB_

#include <ftd2xx/ftd2xx.h>
#include "canio.h"

class CANIOrmpusb: public CANIO {
private:
	FT_STATUS ftStatus;
	FT_HANDLE ftHandle;
	static const unsigned int RX_BUFFER_SIZE = 65536;
	static const unsigned int RX_WARNING_SIZE = 500;
	static const unsigned int RX_KEEP_SIZE = 200;
	unsigned char rxbuf[RX_BUFFER_SIZE];

	bool ready;
	const char* serial;
	int rcount;
	int timeouts;
	int writecount, readcount;
	int usbreadcount, nomsgreadcount;
	unsigned int rxbufcount;
	class rmpUsbPacket {
	public:
		rmpUsbPacket();
		rmpUsbPacket(CanPacket &cpkt);
		~rmpUsbPacket();
		void InitPacket();
		unsigned char computeChecksum();
		void addChecksum();
		char *toString();

		unsigned char bytes[18];
		unsigned char *pID;
		unsigned char *pDATA;
	};

public:
	CANIOrmpusb(const char* serial = NULL);
	virtual ~CANIOrmpusb();
	virtual int Init();
	virtual int ReadPacket(CanPacket *pkt);
	virtual int WritePacket(CanPacket &pkt);
	virtual int Shutdown();
};

#endif // _RMPUSB_CANLIB_
