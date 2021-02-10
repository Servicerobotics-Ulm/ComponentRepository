#include "canio_rmpusb.h"
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <cstdlib>

CANIOrmpusb::CANIOrmpusb(const char* serial) :
	ready(false) {
	this->serial = serial;
	printf("serial is set in CANIO : %s\n", this->serial);
}

CANIOrmpusb::~CANIOrmpusb() {
}

int CANIOrmpusb::Init() {
	DWORD iVID = 0x0403; // Vendor ID for Future Technology Devices Inc
	DWORD iPID = 0xE729; // Product ID for FTD245BM chip in the Segway RMP
	FT_DEVICE ftDevice;
	DWORD deviceID;
	char SerialNumber[16];
	char Description[64];

	ftStatus = FT_SetVIDPID(iVID, iPID); // use our VID and PID
	if (ftStatus != FT_OK) {
		printf("Unable to set appropriate VID/PID for USB device\n");
		return ftStatus;
	}

	if (this->serial != NULL) {
		strncpy(SerialNumber, this->serial, 16);
		printf("Looking to connect to Segway #:%s\n", SerialNumber);
		ftStatus = FT_OpenEx(SerialNumber, FT_OPEN_BY_SERIAL_NUMBER, &ftHandle);
	} else {
		char desc[] = "Robotic Mobile Platform";
		ftStatus = FT_OpenEx(desc, FT_OPEN_BY_DESCRIPTION, &ftHandle);
	}
	if (ftStatus != FT_OK) {
		printf("FT_Open(0) failed\n");
		return ftStatus;
	}

	// Get the info
	ftStatus = FT_GetDeviceInfo(ftHandle, &ftDevice, &deviceID, SerialNumber, Description, NULL);
	if (ftStatus == FT_OK) {
		printf("  SerialNumber=%s\n", SerialNumber);
		printf("  Description=[%s]\n", Description);
		printf("  ftHandle=0x%x\n", ftHandle);
	}

	// Boost the baud rate
	ftStatus = FT_SetBaudRate(ftHandle, FT_BAUD_460800);
	if (ftStatus != FT_OK) {
		printf("Unable to increase USB baud rate\n");
		FT_Close(ftHandle);
		return ftStatus;
	}

	// Decrease the internal latency timer
	ftStatus = FT_SetLatencyTimer(ftHandle, 2);
	if (ftStatus != FT_OK) {
		printf("Unable to decrease latency timer...continuing anyway\n");
	}

	// Set a timeout value of 10ms for reads and writes
	ftStatus = FT_SetTimeouts(ftHandle, 10, 10);
	if (ftStatus != FT_OK) {
		printf("FT_SetTimeouts failed\n");
		FT_Close(ftHandle);
		return ftStatus;
	}

	DWORD rsize;
	ftStatus = FT_GetQueueStatus(ftHandle, &rsize);
	if (ftStatus != FT_OK) {
		printf("Unable to get Queue status\n");
	} else {
		printf("At Init there are %d characters in read queue\n", rsize);
	}

	ftStatus = FT_ResetDevice(ftHandle);
	if (ftStatus != FT_OK) {
		printf("Unable to reset USB FIFO\n");
		FT_Close(ftHandle);
		return ftStatus;
	}

	ftStatus = FT_Purge(ftHandle, FT_PURGE_RX | FT_PURGE_TX);
	if (ftStatus != FT_OK) {
		printf("Unable to clear USB read/write buffers\n");
		FT_Close(ftHandle);
		return ftStatus;
	}

	ftStatus = FT_ResetDevice(ftHandle);
	if (ftStatus != FT_OK) {
		printf("Unable to reset USB FIFO\n");
		FT_Close(ftHandle);
		return ftStatus;
	}

	ftStatus = FT_GetQueueStatus(ftHandle, &rsize);
	if (ftStatus != FT_OK) {
		printf("Unable to get Queue status\n");
	} else {
		printf("After purge/reset there are %d characters in read queue\n", rsize);
	}
	ready = true;
	rcount = timeouts = 0;
	rxbufcount = 0;
	writecount = readcount = 0;
	usbreadcount = nomsgreadcount = 0;
	return 0;
}

/* Closes the CAN channel
 *
 * returns: 0 on success, nonzero error code otherwise
 */
int CANIOrmpusb::Shutdown() {
	ftStatus = FT_Close(ftHandle);
	if (ftStatus != FT_OK) {
		printf("FT_Close failed\n");
		return ftStatus;
	}
	ready = false;
	return 0;
}

/* Writes the given packet
 *
 * returns: 0 on success, nonzero error code otherwise
 */
int CANIOrmpusb::WritePacket(CanPacket &cpkt) {
	DWORD bytes_written;
	DWORD bytes_to_write;

	writecount++;
#ifdef PRINTSTATS
	if ((writecount % 100) == 0) {
		printf("writes %d  read calls %d  device reads %d  aborted reads %d\n",writecount,readcount,usbreadcount,nomsgreadcount);
	}
#endif

	if (!ready) {
		return -1;
	}

	static struct timeval *last = NULL;
	struct timeval curr;

	// initialize the first timeval if necessary
	if (!last) {
		last = new struct timeval;
		gettimeofday(last, NULL);
	}

	// get the current time
	gettimeofday(&curr, NULL);

	// calculate how long since the last write
	double msecs = (curr.tv_sec - last->tv_sec) * 1000 + (curr.tv_usec - last->tv_usec) / 1000;

	// if it's been less than 30 milliseconds, sleep so that we don't
	// overload the CAN bus
	if (msecs < 30) {
		usleep((30 - msecs) * 1000);
	}

	// create a USB packet from our CAN packet
	CANIOrmpusb::rmpUsbPacket packet(cpkt);

	//    printf("USB packet created: %s\n",packet.toString());

	bytes_to_write = 18;
	unsigned char *pData = packet.bytes;
	while (bytes_to_write > 0) {
		//printf("Debug towrite%u \n",bytes_to_write);
		ftStatus = FT_Write(ftHandle, pData, bytes_to_write, &bytes_written);
		if (ftStatus != FT_OK) {
			printf("Error while trying to write packet to USB port\n");
			return ftStatus;
		}
		bytes_to_write -= bytes_written;
		pData += bytes_written;
		//printf("Debug towrite%u written%u\n",bytes_to_write,bytes_written);	
	}
	//    printf("USB packet sent: %s\n",packet.toString());

	// record the time
	gettimeofday(last, NULL);

	return 0;
}

/* Reads a packet from the USB bus, and extracts the CAN data.
 */
int CANIOrmpusb::ReadPacket(CanPacket *pkt) {
	// We'll read as much as we possibly can, since the read call is slow.
	// The extra bytes will be kept around until next time, and just processed
	// from memory.

	readcount++;
	DWORD bytes_read;

	if (!ready) {
		return -1;
	}

	CANIOrmpusb::rmpUsbPacket packet;

	if (rxbufcount > RX_WARNING_SIZE) {
		// Player is falling behind in processing the messages, so throw out
		// the oldest bytes
		memcpy(rxbuf, rxbuf + rxbufcount - RX_KEEP_SIZE, RX_KEEP_SIZE);
		//        printf("Warning: purged %d old bytes from USB interface\n", rxbufcount-RX_KEEP_SIZE);
		rxbufcount = RX_KEEP_SIZE;
	}

	if (rxbufcount < 180) {
		// only read more from the device if we're running low
		DWORD rsize;
		ftStatus = FT_GetQueueStatus(ftHandle, &rsize);
		if (ftStatus != FT_OK) {
			printf("Unable to get Queue status\n");
			return -1;
		}

		// Limit what we will read to the space left in the buffer
		if (rsize > (RX_BUFFER_SIZE - rxbufcount)) {
			rsize = RX_BUFFER_SIZE - rxbufcount;
		}

		if (rsize < 18) {
			// not a full packet available
			// it seems that if we try to read something now, it
			// provokes the FTDI chip into producing garbage, so instead
			// we'll wait until later to get at least 1 full packet
			return 0;
		}
		ftStatus = FT_Read(ftHandle, rxbuf + rxbufcount, rsize, &bytes_read);
		usbreadcount++;
		if (ftStatus != FT_OK) {
			printf("Error reading from USB device\n");
			return -1;
		}

		rxbufcount += bytes_read;

	}

	int i;
	// Now, search for the next valid packet
	for (i = 0; i < ((int) rxbufcount - 17); i++) {
		if ((rxbuf[i] != 0xF0) || (rxbuf[i + 1] != 0x55) || (rxbuf[i + 2] != 0xAA)) {
			continue;
		}

		// move bytes into 18-byte USB packet and check
		memcpy(packet.bytes, rxbuf + i, 18);
		if (packet.computeChecksum() != packet.bytes[17]) {
			continue;
		}

		// extract CAN packet
		// this is what the v2.0 preliminary documentation says: ID is located in
		// bytes 6 and 7, but it doesn't actually say which 11 bits to use, so I was
		// using the low bits.  This is how packets are created when sending, but
		// I've learned it's totally wrong for received packets.
		// pkt->id = ((packet.bytes[6] << 8) + packet.bytes[7]) & 0x7FF;

		// this is what the Segway RMI_DEMO does for received packets.  It uses all 8
		// bits from byte 4 and the first 3 bits from byte 5.
		pkt->id = ((packet.bytes[4] << 3) | ((packet.bytes[5] >> 5) & 7)) & 0xfff;
		memcpy(pkt->msg, packet.pDATA, 8);
		//  printf("CAN packet extracted: %s\n", pkt->toString());

		// shift the receive buffer
		memcpy(rxbuf, rxbuf + i + 18, rxbufcount - i - 18);
		rxbufcount -= (i + 18);

		// return the CAN bytes received
		return 10;
	}

	// we didn't find any valid messages
	// shift the receive buffer
	memcpy(rxbuf, rxbuf + i, rxbufcount - i);
	rxbufcount -= i;

	nomsgreadcount++;
	return 0;

}

CANIOrmpusb::rmpUsbPacket::rmpUsbPacket() {
	InitPacket();
}

CANIOrmpusb::rmpUsbPacket::rmpUsbPacket(CanPacket &cpkt) {
	InitPacket();
	bytes[6] = (cpkt.id >> 8) & 0xff;
	bytes[7] = cpkt.id & 0xff;
	memcpy(pDATA, cpkt.msg, 8);
	addChecksum();
}

void CANIOrmpusb::rmpUsbPacket::InitPacket() {
	bytes[0] = 0xF0;
	bytes[1] = 0x55;
	bytes[2] = 0;
	bytes[3] = 0;
	bytes[4] = 0;
	bytes[5] = 0;
	bytes[6] = 0;
	bytes[7] = 0;
	bytes[8] = 0;
	bytes[9] = 0;
	bytes[10] = 0;
	bytes[11] = 0;
	bytes[12] = 0;
	bytes[13] = 0;
	bytes[14] = 0;
	bytes[15] = 0;
	bytes[16] = 0;
	bytes[17] = 0;
	pID = bytes + 6;
	pDATA = bytes + 9;
}

CANIOrmpusb::rmpUsbPacket::~rmpUsbPacket() {
}

unsigned char CANIOrmpusb::rmpUsbPacket::computeChecksum() {
	// code copied from Segway RMP Interface Guide
	unsigned short checksum;
	unsigned short checksum_high;
	checksum = 0;
	for (int i = 0; i < 17; i++) {
		checksum += (short) bytes[i];
	}
	checksum_high = checksum >> 8;
	checksum &= 0xff;
	checksum += checksum_high;
	checksum_high = checksum >> 8;
	checksum &= 0xff;
	checksum += checksum_high;
	checksum = (~checksum + 1) & 0xff;
	return (unsigned char) checksum;
}

void CANIOrmpusb::rmpUsbPacket::addChecksum() {
	bytes[17] = computeChecksum();
}

char * CANIOrmpusb::rmpUsbPacket::toString() {
	static char buf[256];
	buf[0] = 0;
	for (int i = 0; i < 18; i++) {
		sprintf(buf + strlen(buf), "%d:%02X ", i, bytes[i]);
	}
	return buf;
}

