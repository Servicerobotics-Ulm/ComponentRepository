//--------------------------------------------------------------------------
//  Copyright (C) 2010 Jonas Brich
//
//        brich@mail.hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "Unicap Video Server component".
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//--------------------------------------------------------------------------

#ifndef UNICAP_HH_
#define UNICAP_HH_

#include "aceSmartSoft.hh"
#include <DomainVision/CommVideoImage.hh>
#include <unicap.h>

#define TEXT_COLOR_RESET   "\033[0m"
#define TEXT_COLOR_GREEN   "\033[32m"       /* Green */
#define TEXT_COLOR_RED     "\033[31m"       /* Red   */
#include "ParameterStateStruct.hh"
/**
 * Define for better usage.
 */
#define UNICAPINZ Unicap::instance()

/**
 * This class holds all the library specific code.
 *
 * - The init()-method should be called on startup. It init the Camera with all specified properties in
 * the init-File and opens the capture mode.
 * - The getImage()-method gets the image from the camera and copies it into the given CommMutableVideoImage.
 * It is a real copy, the image is a duplicate.
 */
class Unicap: public ACE_Event_Handler {
public:

	class UnicapException: public std::exception {
	private:
		std::string message;

	public:
		UnicapException(std::string m) :
			message(m) {
		}

		virtual ~UnicapException() throw() {
		}

		const char* what() const throw() {
			return message.c_str();
		}
	};

	static Unicap* instance();

	void init();

	void getImage(DomainVision::CommVideoImage& image, ParameterStateStruct& localState);

	bool startCaptureMode();
	bool stopCaptureMode();

	virtual ~Unicap();
private:
	Unicap();

	static Unicap* _instance;
	static SmartACE::SmartSemaphore _instance_sem;

	const int MAX_DEVICES;
	const int MAX_FORMATS;
	const int MAX_PROPERTIES;

	unicap_handle_t _handle;
	unicap_data_buffer_t _buffer;

	unsigned long int _seq_counter;

	void init_getCameraAndOpenCamera();
	void init_getAndSetFormatOfCamera();
	void init_getAndSetPropertiesOfCamera();
	void init_setImageCapturing();

	void _handleProperty(double value, unicap_property_t& property);
	void _handleMenuProperty(int value, unicap_property_t& property);

	// This method handles the SIGINT-signal and closes the handle to the camera
	virtual int handle_signal(int signum, siginfo_t * = 0, ucontext_t * = 0);

	// Debug methods
	void showProperty(unicap_property_t& property) const;
	void showAllProperties() const;
};

#endif /* UNICAP_HH_ */
