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

#include <sstream>

#include "Unicap.hh"
#include "ComponentUnicapImageServer.hh"

#include "ParameterStateStruct.hh"

Unicap* Unicap::_instance = NULL;
SmartACE::SmartSemaphore Unicap::_instance_sem;

Unicap* Unicap::instance() {
	if (_instance == NULL) {
		_instance_sem.acquire();
		if (_instance == NULL) {
			_instance = new Unicap();
			_instance_sem.release();
		}
	}
	return _instance;
}

Unicap::Unicap() :
	MAX_DEVICES(10), MAX_FORMATS(10), MAX_PROPERTIES(32) {
	_handle = NULL;
	_buffer.data = NULL;
	_seq_counter = 0;
	ACE_Reactor::instance()->register_handler(SIGINT, this);
}

void Unicap::init() {

	this->init_getCameraAndOpenCamera();
	this->init_getAndSetFormatOfCamera();
	this->init_getAndSetPropertiesOfCamera();
	this->init_setImageCapturing();
}

/**
 * Get the devices.
 *
 * Gets all devices which are attached at the computer and then searches for
 * the specified string in the ini-File.
 * If the specified device is attached it will be opened, otherwise the init will fail.
 */
void Unicap::init_getCameraAndOpenCamera() {

	ParameterStateStruct localState = COMP->getGlobalState();

	int dev_count = 0;
	int dev = -1;
	unicap_device_t devices[MAX_DEVICES];
	unicap_status_t status = STATUS_SUCCESS;

	// Iterate over all cameras which are attached at the computer
	std::cout << "Available Cameras:\n";
	for (; SUCCESS(status) && (dev_count < MAX_DEVICES); dev_count++) {
		status = unicap_enumerate_devices(NULL, &devices[dev_count], dev_count);
		if (SUCCESS(status)) {
			std::cout << "Device: " << devices[dev_count].device << " | Identifier: " << devices[dev_count].identifier
					<< std::endl;
			// Compare the current device of the camera with the device specified in the ini-File
			if (strcmp(devices[dev_count].device, localState.getHardware().getDevice().c_str()) == 0) {
				// Get first camera on the device
				if (dev == -1) {
					dev = dev_count;
				}
				// Get the camera corresponding to the identifier
				if (strcmp(devices[dev_count].identifier, localState.getHardware().getIdentifier().c_str()) == 0) {
					dev = dev_count;
				}
			}
		} else {
			break;
		}
	}

	// Specified device not found;
	if (dev == -1) {
		throw UnicapException("Specified device not found. Cannot find Camera.");
	}

	// Open the camera
	if (!SUCCESS(unicap_open(&_handle, &devices[dev]))) {
		throw UnicapException("Cannot open Camera.");
	}
}

/**
 * Get the formats
 *
 * Gets all formats which are available from the camera and then searches for
 * the specified string in the ini-File.
 * If the specified format is available it will be set, otherwise the init will fail.
 */
void Unicap::init_getAndSetFormatOfCamera() {

	ParameterStateStruct localState = COMP->getGlobalState();

	int format_count = 0;
	int format = -1;
	unicap_format_t formats[MAX_FORMATS];
	unicap_status_t status = STATUS_SUCCESS;

	std::stringstream formatIni;
	if (strcmp(localState.getHardware().getCamera_type().c_str(), "FireWire") == 0) {
		// Build the format out of the ini-File, e.g. Y(Mono) 1024x786
		formatIni << localState.getHardware_properties().getFormat() << " " << localState.getHardware_properties().getWidth() << "x"
				<< localState.getHardware_properties().getHeight();

		// Iterate over all formats given by the camera
		for (; SUCCESS(status) && (format_count < MAX_FORMATS); format_count++) {
			status = unicap_enumerate_formats(_handle, NULL, &formats[format_count], format_count);
			if (SUCCESS(status)) {
				// Compare the current format of the camera with the format specified in the ini-File
				if (strcmp(formats[format_count].identifier, formatIni.str().c_str()) == 0) {
					format = format_count;
					break;
				}
			} else {
				break;
			}
		}
	} else if (strcmp(localState.getHardware().getCamera_type().c_str(), "USB") == 0) {
		formatIni << localState.getHardware_properties().getFormat();

		// Iterate over all formats given by the camera
		for (; SUCCESS(status) && (format_count < MAX_FORMATS); format_count++) {
			status = unicap_enumerate_formats(_handle, NULL, &formats[format_count], format_count);
			if (SUCCESS(status)) {
				// Compare the current format of the camera with the format specified in the ini-File
					std::cout << "Format :" << formats[format_count].identifier <<"h:"<<localState.getHardware_properties().getHeight() <<"w:"<<localState.getHardware_properties().getWidth() <<"\n";
				if (strcmp(formats[format_count].identifier, formatIni.str().c_str()) == 0) {
					formats[format_count].size.height = localState.getHardware_properties().getHeight();
					formats[format_count].size.width = localState.getHardware_properties().getWidth();
					format = format_count;
					std::cout << "Format :" << formats[format_count].identifier << "h:"<<formats[format_count].size.height<< "w:"<< formats[format_count].size.width<<"\n";
					break;
				}
			} else {
				break;
			}
		}
	}

	// If the specified format is not available, all possible formats will be printed.
	if (format == -1) {
		status = STATUS_SUCCESS;
		format_count = 0;
		std::cout << "Specified Format: " << formatIni.str().c_str() << std::endl;
		for (; SUCCESS(status) && (format_count < MAX_FORMATS); format_count++) {
			status = unicap_enumerate_formats(_handle, NULL, &formats[format_count], format_count);
			if (SUCCESS(status)) {
				if (format_count == 0) {
					std::cout << "Available Range:\n";
					for (int i = 0; i < formats[format_count].size_count; ++i) {
						std::cout << formats[format_count].sizes[i].width << "x" << formats[format_count].sizes[i].height
								<< std::endl;
					}
					std::cout << "Available Formats:\n";
				}
				std::cout << formats[format_count].identifier << std::endl;
			} else {
				break;
			}
		}
		throw UnicapException("Specified format not found.");
	}
	std::cout<<"format.buffer_size"<<formats[format].buffer_size<<"w: "<<formats[format].size.width<<"h: "<<formats[format].size.height<<std::endl;

	// Set the format
	if (!SUCCESS(unicap_set_format(_handle, &formats[format]))) {
		throw UnicapException("Format cannot be set.");
	}
}

/**
 * Get the properties
 *
 * Gets all properties which are available from the camera. Then the properties which are
 * specified in the ini-File will be set.
 * If the property is available in the ini-File but not in the camera nothing will happen.
 * If the property is not specified in the ini-File but available in the camera,
 * the default value will be assumed.
 *
 * Possible property types:
 * 	UNICAP_PROPERTY_TYPE_DATA
 * 		GPIO, Register
 * 	UNICAP_PROPERTY_TYPE_MENU
 * 		TriggerMode, TriggerPolarity, StrobeMode, etc.
 * 	UNICAP_PROPERTY_TYPE_VALUE_LIST
 * 		Framerate
 * 	UNICAP_PROPERTY_TYPE_FLAGS
 * 		White_Balance_Mode
 *  UNICAP_PROPERTY_TYPE_RANGE
 *  	brightness, auto_exposure, hue, gain, gamma, etc.
 */

// SONY DFW-X710
// UNICAP_PROPERTY_TYPE_RANGE & UNICAP_PROPERTY_TYPE_FLAGS & UNICAP_PROPERTY_TYPE_VALUE_LIST
//	ID: brightness | Value: 16 Range: 0 - 127 Stepping: 1
//	ID: abs focus | Value: 300 Range: 70 - 620 Stepping: 1
//	ID: abs zoom | Value: 300 Range: 10 - 350 Stepping: 1
//	ID: init lens | Value: 0 Range: 0 - 1 Stepping: 1
//	ID: strobe_duration | Value: 10 Range: 10 - 40000 Stepping: 10
//	ID: strobe_delay | Value: 0 Range: -20000 - 20000 Stepping: 10
//	ID: auto_exposure | Value: 100 Range: 90 - 115 Stepping: 1
//	ID: sharpness | Value: 3 Range: 0 - 7 Stepping: 1
//	ID: white_balance_mode | Value: 3 Range: 0 - 7 Stepping: 0
//	ID: white_balance_u | Value: 2048 Range: 1792 - 2304 Stepping: 1
//	ID: white_balance_v | Value: 2048 Range: 1792 - 2304 Stepping: 1
//	ID: hue | Value: 128 Range: 83 - 173 Stepping: 1
//	ID: saturation | Value: 128 Range: 0 - 511 Stepping: 1
//	ID: gamma | Value: 129 Range: 128 - 130 Stepping: 1
//	ID: shutter | Value: 0.0143 Range: 1e-05 - 17.5 Stepping: 0.005
//	ID: gain | Value: 70 Range: 70 - 551 Stepping: 1
//	ID: pan | Value: 0 Range: 0 - 0 Stepping: 0
//	ID: tilt | Value: 0 Range: 0 - 0 Stepping: 0
//	ID: optical_filter | Value: 0 Range: 0 - 1 Stepping: 1
//	ID: timeout | Value: 1 Range: 0 - 600 Stepping: 1
//	ID: gpio | Value: 1 Range: 0 - 600 Stepping: 1
//  ID: frame rate | Value: 0 Range: 0 - 1 Stepping: ?
void Unicap::init_getAndSetPropertiesOfCamera() {

	ParameterStateStruct localState = COMP->getGlobalState();

	int property_count = 0;
	int range_ppty_count = 0;
	unicap_property_t properties[MAX_PROPERTIES];
	unicap_status_t status = STATUS_SUCCESS;

	// Iterate over all properties and set the specified ini-Value
	for (; SUCCESS(status) && (property_count < MAX_PROPERTIES); property_count++) {
		status = unicap_enumerate_properties(_handle, NULL, &properties[range_ppty_count], property_count);
		if (SUCCESS(status)) {
			if (properties[range_ppty_count].type == UNICAP_PROPERTY_TYPE_RANGE || properties[range_ppty_count].type
					== UNICAP_PROPERTY_TYPE_FLAGS || properties[range_ppty_count].type == UNICAP_PROPERTY_TYPE_MENU
					|| properties[range_ppty_count].type == UNICAP_PROPERTY_TYPE_VALUE_LIST || properties[range_ppty_count].type
					== UNICAP_PROPERTY_TYPE_DATA || properties[range_ppty_count].type == UNICAP_PROPERTY_TYPE_UNKNOWN) {
				if (strcmp(properties[range_ppty_count].identifier, "auto_exposure") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: auto_exposure\n";
					_handleProperty(localState.getHardware_properties().getAuto_exposure(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "white_balance_mode") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: white_balance_mode\n";
					_handleProperty(localState.getHardware_properties().getWhite_balance_mode(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "brightness") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: brightness\n";
					_handleProperty(localState.getHardware_properties().getBrightness(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "gain") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: gain\n";
					_handleProperty(localState.getHardware_properties().getGain(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "gamma") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: gamma\n";
					_handleProperty(localState.getHardware_properties().getGamma(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "hue") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: hue\n";
					_handleProperty(localState.getHardware_properties().getHue(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "saturation") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: saturation\n";
					_handleProperty(localState.getHardware_properties().getSaturation(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "sharpness") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: sharpness\n";
					_handleProperty(localState.getHardware_properties().getSharpness(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "shutter") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: shutter\n";
					_handleProperty(localState.getHardware_properties().getShutter(), properties[range_ppty_count]);
				}
				// Set only if Auto_white_balance is false
				else if ((strcmp(properties[range_ppty_count].identifier, "white_balance_u") == 0)
						&& localState.getHardware_properties().getAutoflag_white_balance_mode() == false) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: white_balance_u\n";
					_handleProperty(localState.getHardware_properties().getWhite_balance_u(), properties[range_ppty_count]);
				}
				// Set only if Auto_white_balance is false
				else if ((strcmp(properties[range_ppty_count].identifier, "white_balance_v") == 0)
						&& localState.getHardware_properties().getAutoflag_white_balance_mode() == false) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: white_balance_v\n";
					_handleProperty(localState.getHardware_properties().getWhite_balance_v(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "trigger_mode") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: trigger_mode\n";
					_handleMenuProperty(localState.getHardware_properties().getTrigger_mode(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "trigger_polarity") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: trigger_polarity\n";
					_handleMenuProperty(localState.getHardware_properties().getTrigger_polarity(), properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "frame rate") == 0) {
					if (localState.getHardware().getDebug_info())
						std::cout << "Prop: frame rate\n";
					_handleProperty(localState.getHardware_properties().getFramerate(), properties[range_ppty_count]);
				} else {
					range_ppty_count++;
					continue;
				}
				if (localState.getHardware().getDebug_info())
					showProperty(properties[range_ppty_count]);
				range_ppty_count++;
			}
		} else {
			break;
		}
	}

	// Set auto mode
	if (localState.getHardware_properties().getAutoflag_white_balance_mode()) {
		unicap_set_property_auto(_handle, "white_balance_mode");
	}
	if (localState.getHardware_properties().getAutoflag_shutter()) {
		unicap_set_property_auto(_handle, "shutter");
	}
}

/**
 * Init Video capturing
 *
 * Init the video capturing. First the capturing format will be set to
 * UNICAP_BUFFER_TYPE_USER. The other variant would be UNICAP_BUFFER_TYPE_SYSTEM.
 *
 * UNICAP_BUFFER_TYPE_USER:
 * 	The image will be put in an returnedBuffer over a wait-method.
 * UNICAP_BUFFER_TYPE_SYSTEM:
 * 	The image will can be get over a callback method.
 */
void Unicap::init_setImageCapturing() {

	unicap_format_t uniFormat;

	if (!SUCCESS(unicap_get_format(_handle, &uniFormat))) {
		throw UnicapException("Format for video capturing cannot be get.");
	}

	uniFormat.buffer_type = UNICAP_BUFFER_TYPE_USER;

	std::cout<<"format.buffer_size"<<uniFormat.buffer_size<<"w: "<<uniFormat.size.width<<"h: "<<uniFormat.size.height<<std::endl;


	if (!SUCCESS(unicap_set_format(_handle, &uniFormat))) {
		throw UnicapException("Format for video capturing cannot be set.");
	}

	// Allocate the memory for the image buffer
	_buffer.data = (unsigned char*) malloc(uniFormat.buffer_size);
	_buffer.buffer_size = uniFormat.buffer_size;
}

bool Unicap::startCaptureMode() {
	// Start capture mode
	return SUCCESS(unicap_start_capture(_handle));
}
bool Unicap::stopCaptureMode() {
	// Stop capture if _handle is set
	if (_handle != NULL) {
		return SUCCESS(unicap_stop_capture(_handle));
	}
	return true;
}

void Unicap::getImage(DomainVision::CommVideoImage& image, ParameterStateStruct& localState) {
	unicap_data_buffer_t *returned_buffer = &_buffer;

	if (!SUCCESS(unicap_queue_buffer(_handle, returned_buffer))) {
		throw UnicapException("Failed to queue image buffer.");
	}
	// Wait for the buffer to be filled with the image
	if (!SUCCESS(unicap_wait_buffer(_handle, &returned_buffer))) {
		throw UnicapException("Failed to wait for queued buffer.");
	}
	_seq_counter++;

	if (image.get_width() == localState.getHardware_properties().getWidth() && image.get_height() == localState.getHardware_properties().getHeight()
			&& image.get_size() == returned_buffer->buffer_size) {
		image.set_data(_buffer.data);
		image.set_sequence_counter(_seq_counter);
		if (localState.getImage().getDebug_info())
			std::cout << "Image: DATA VALID" << std::endl;
	} else {
		image.set_data_invalid();
		if (localState.getImage().getDebug_info())
			std::cout << "Image: DATA INVALID" << std::endl;
	}

	if (localState.getHardware().getDebug_info()) {
		showAllProperties();
	}
}

void Unicap::_handleProperty(double value, unicap_property_t& property) {
	if (SUCCESS(unicap_get_property(_handle, &property))) {
		property.value = value;
		if (SUCCESS(unicap_set_property(_handle, &property))) {
			return;
		}
	}
	throw UnicapException("Property cannot be set.");
}

void Unicap::_handleMenuProperty(int value, unicap_property_t& property) {
	if (SUCCESS(unicap_get_property(_handle, &property))) {
		if ((value >= 0) && (value < property.menu.menu_item_count)) {
			strcpy(property.menu_item, property.menu.menu_items[value]);
			if (SUCCESS(unicap_set_property(_handle, &property))) {
				return;
			}
		}
	}
	throw UnicapException("Property cannot be set.");
}

void Unicap::showProperty(unicap_property_t& property) const {
	double value = -1;
	char* menu;
	int enable = -1;
	if (SUCCESS(unicap_get_property_menu(_handle, property.identifier, &menu)))
		std::cout << "Value: " << menu << std::endl;
	if (SUCCESS(unicap_get_property_value(_handle, property.identifier, &value)))
		std::cout << "Value: " << value << std::endl;
	if (SUCCESS(unicap_get_property_auto(_handle, property.identifier, &enable)))
		std::cout << "Enable: " << enable << std::endl;
}

void Unicap::showAllProperties() const {
	int property_count = 0;
	int range_ppty_count = 0;
	unicap_property_t properties[MAX_PROPERTIES];
	unicap_status_t status = STATUS_SUCCESS;

	std::cout << "Properties:\n" << "---------------------------------------------------\n";

	// Iterate over all properties and set the specified ini-Value
	for (; SUCCESS(status) && (property_count < MAX_PROPERTIES); property_count++) {
		status = unicap_enumerate_properties(_handle, NULL, &properties[range_ppty_count], property_count);
		if (SUCCESS(status)) {
			if (properties[range_ppty_count].type == UNICAP_PROPERTY_TYPE_RANGE || properties[range_ppty_count].type
					== UNICAP_PROPERTY_TYPE_FLAGS || properties[range_ppty_count].type == UNICAP_PROPERTY_TYPE_MENU
					|| properties[range_ppty_count].type == UNICAP_PROPERTY_TYPE_VALUE_LIST || properties[range_ppty_count].type
					== UNICAP_PROPERTY_TYPE_DATA || properties[range_ppty_count].type == UNICAP_PROPERTY_TYPE_UNKNOWN) {
				if (strcmp(properties[range_ppty_count].identifier, "auto_exposure") == 0) {
					std::cout << "Prop: auto_exposure\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "white_balance_mode") == 0) {
					std::cout << "Prop: white_balance_mode\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "brightness") == 0) {
					std::cout << "Prop: brightness\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "gain") == 0) {
					std::cout << "Prop: gain\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "gamma") == 0) {
					std::cout << "Prop: gamma\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "hue") == 0) {
					std::cout << "Prop: hue\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "saturation") == 0) {
					std::cout << "Prop: saturation\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "sharpness") == 0) {
					std::cout << "Prop: sharpness\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "shutter") == 0) {
					std::cout << "Prop: shutter\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "white_balance_u") == 0) {
					std::cout << "Prop: white_balance_u\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "white_balance_v") == 0) {
					std::cout << "Prop: white_balance_v\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "trigger_mode") == 0) {
					std::cout << "Prop: trigger_mode\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "trigger_polarity") == 0) {
					std::cout << "Prop: trigger_polarity\n";
					showProperty(properties[range_ppty_count]);
				} else if (strcmp(properties[range_ppty_count].identifier, "frame rate") == 0) {
					std::cout << "Prop: frame rate\n";
					showProperty(properties[range_ppty_count]);
				}
				range_ppty_count++;
			} else {
				break;
			}
		}
	}
}

int Unicap::handle_signal(int signum, siginfo_t *, ucontext_t *) {
	if (signum == SIGINT) {

		if (_handle != NULL) {
			unicap_stop_capture(_handle);
		}
		if (_buffer.data != NULL) {
			free(_buffer.data);
		}
		if (_handle != NULL) {
			unicap_close(_handle);
		}
		exit(0);
	}
	return 0;
}

Unicap::~Unicap() {

	if (_handle != NULL) {
		unicap_stop_capture(_handle);
	}
	if (_buffer.data != NULL) {
		free(_buffer.data);
	}
	if (_handle != NULL) {
		unicap_close(_handle);
	}
}
