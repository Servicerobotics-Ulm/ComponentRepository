/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2016 Intel Corporation. All Rights Reserved.

*******************************************************************************/

/** @file person_tracking_video_module_interface.h
    @brief
    Describes the \c person_tracking_video_module_interface interface,
    which gives access to the Person module's configuration and output data.
 */

#pragma  once
#include "3rdParty/include/rs/core/video_module_interface.h"
#include "RealSense/PersonTracking/PersonTrackingConfiguration.h"
#include "RealSense/PersonTracking/PersonTrackingData.h"


namespace rs
{
    namespace person_tracking
    {
        /**
        @class person_tracking_video_module_interface
        @brief The entry point to person tracking middleware module. From this class, you can query the configuration and the processing result.
        */
        class person_tracking_video_module_interface : public rs::core::video_module_interface
        {
        public:
            /**
            @brief Returns current configuration.
            */
            virtual Intel::RealSense::PersonTracking::PersonTrackingConfiguration* QueryConfiguration() = 0;

            /**
            @brief Returns latest available output data.
            */
            virtual Intel::RealSense::PersonTracking::PersonTrackingData* QueryOutput() = 0;

            person_tracking_video_module_interface(const person_tracking_video_module_interface&) = delete;
            person_tracking_video_module_interface& operator=(const person_tracking_video_module_interface&) = delete;
            virtual ~person_tracking_video_module_interface()= default;
            person_tracking_video_module_interface()= default;
        };
    }
}
