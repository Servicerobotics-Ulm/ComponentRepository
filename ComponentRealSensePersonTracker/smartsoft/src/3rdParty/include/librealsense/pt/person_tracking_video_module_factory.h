/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2016 Intel Corporation. All Rights Reserved.

*******************************************************************************/
/** @file person_tracking_video_module_factory.h
    @brief
    Describes the \c person_tracking_video_module_factory class,
    which possibility to create person_tracking_video_module instance.
 */
#pragma once

#include "person_tracking_video_module_interface.h"

namespace rs
{
    namespace person_tracking
    {
        class person_tracking_video_module_factory
        {
        public:
            /**
            @brief Creates person tracking video module object.
            @param[in] dataDir      Path to person tracking data files
            @return person tracking video module object
            */
            static person_tracking_video_module_interface* create_person_tracking_video_module(const wchar_t* dataDir);

            /**
            @brief Creates person tracking video module object.
            @return person tracking video module object
            */
            static person_tracking_video_module_interface* create_person_tracking_video_module();
        };
    }
}

