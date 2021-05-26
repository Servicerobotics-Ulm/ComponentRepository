/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2013-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/

/** @file PersonTrackingModule.h
    @brief
	Describes the \c PersonTrackingModule interface, 
	which gives access to the Person module's configuration and output data.
 */
#pragma once
#include "RealSense/VideoModule.h"
#include "RealSense/PersonTracking/PersonTrackingData.h"
#include "RealSense/PersonTracking/PersonTrackingConfiguration.h"

#ifndef _WIN32
extern "C" {
    void* RS_PersonTrackingModule_CreateInstance();
}
#endif

namespace Intel {
    namespace RealSense {
        namespace PersonTracking {
            class PersonTrackingConfiguration;
            class PersonTrackingData;
			struct CalibrationData;

            /**
            @class PersonTrackingModule
			@brief The entry point to the middleware module. From this class, you can query the configuration and the processing result.
			
            */
            class PersonTrackingModule : public Base
            {
            public:
                PXC_CUID_OVERWRITE(PXC_UID('P', 'O', 'T', 'M'));

                /**
                @brief Returns current configuration.
                */
                virtual PersonTrackingConfiguration* PXCAPI QueryConfiguration() = 0;

                /**
                @brief Returns latest available output data.
                */
                virtual PersonTrackingData* PXCAPI QueryOutput() = 0;

				/**
				@brief Sets platform camera intrinsics

				Intended to be used only by clients working with both cameras.
				*/
				virtual void PXCAPI SetPlatformCameraIntrinsics(float rotationMatrix[3][3], float translationMatrix[3],
					float focalLengthX, float focalLengthY, float cX, float cY) = 0;

				/**
				@brief Sets platform as the main color camera for person tracking module

				Intended to be used only by clients working with both cameras.
				*/
				virtual void PXCAPI SetPlatformCameraAsMainColorPlane() = 0;

				/**
				@brief Forces middleware to only process frames that contain color images from both integrated and platform cameras.
				*/
				virtual void PXCAPI SyncronizeCamerasImages() = 0;

				/**
				@brief Compatibility mode for resolving specific issue with old core. Should generally not be used.
				*/
				virtual void PXCAPI UseRGBA2BGRAHack() = 0;

				/**
				@brief Notifies the middleware that it should use intrinsics projection between integrated and platform camera.
				*/
				virtual void PXCAPI SetIntrinsicsProjection() = 0;

				/**
				@brief Sets calibration data for the intrinsics projection.
				*/
				virtual void PXCAPI SetCalibrationData(const CalibrationData& cd) = 0;

				/**
				@brief Used to overcome a bug which caused the platform camera images to be mirrored to the integrated camera images.
				*/
				virtual void PXCAPI FlipPcamImageHack() = 0;

#ifndef _WIN32
                /**
                @brief Creates a standalone module.
                */
                static __inline PersonTrackingModule* CreateInstance() {
                    return (PersonTrackingModule*)RS_PersonTrackingModule_CreateInstance();
                }
#endif
            };
        }
    }
}
