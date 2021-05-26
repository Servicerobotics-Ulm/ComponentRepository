/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not  be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2013-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/

/** @file PersonTrackingConfiguration.h
    @brief
	Describes the \c PersonTrackingConfiguration class.
 */
#pragma once
#include "RealSense/PersonTracking/PersonTrackingData.h"
#include "RealSense/PersonTracking/PersonTrackingModule.h"

namespace Intel {
	namespace RealSense {

		namespace PersonTracking {
			/**
			@brief Interface for configuration of the Person module features.
			*/
			class PersonTrackingConfiguration : public Base
			{
			public:
				/* Constants */
				PXC_CUID_OVERWRITE(PXC_UID('P', 'O', 'T', 'C'));

				/**
				@enum TrackingStrategyType
				@brief not in use
				tracking strategy type
				**/
				enum TrackingStrategyType
				{
					STRATEGY_APPEARANCE_TIME,
					STRATEGY_CLOSEST_TO_FARTHEST,
					STRATEGY_FARTHEST_TO_CLOSEST,
					STRATEGY_LEFT_TO_RIGHT,
					STRATEGY_RIGHT_TO_LEFT
				};

				/** @brief person tracking configuration **/
				class TrackingConfiguration
				{
				public:
					/** @brief tracking mode **/
					enum TrackingMode
					{
						TRACKING_MODE_FOLLOWING, //don't start tracking automaticaly
						TRACKING_MODE_INTERACTIVE, // not supported
						TRACKING_MODE_SINGLE_PERSON  // not supported
					};
					/** @brief detection mode **/
					enum DetectionMode
					{
						AUTO,               // auto-detection using load-balancing, recommended to use when activated every frame, default mode
						MANUAL_CLOSE_RANGE, // detect all person in close range currently using face detection in single frame
						MANUAL_MID_RANGE,   // regular detection withing 0.7-3 meters based on depth and rgb in single frame
						MANUAL_FAR_RANGE,   // far range detection using rgb only in single frame
						MANUAL_ALL          // activate all detectors together in single frame
					};

					virtual ~TrackingConfiguration() {}

					/**
					@brief Enables person tracking feature and subfeatures(blob, head bounding box...)
					*/
					virtual void PXCAPI Enable() = 0;

					/**
					@brief Disables person tracking feature and subfeatures(blob, head bounding box...)
					*/
					virtual void PXCAPI Disable() = 0;

					/**
					@brief Get state person tracking feature(enabled/disabled)
					@return bool true if feature is enabled, else false
					*/
					virtual bool32_t PXCAPI IsEnabled() = 0;

					/**
					@brief Enables person mask on color image
					*/
					virtual void PXCAPI EnableSegmentation() = 0;

					/**
					@brief Disables person mask on color image
					*/
					virtual void PXCAPI DisableSegmentation() = 0;

					/**
					@brief Get state segmenation feature(enabled/disabled)
					@return bool true if feature is enabled, else false
					*/
					virtual bool32_t PXCAPI IsSegmentationEnabled() = 0;

					/**
					@brief Enables person mask on depth image
					*/
					virtual void PXCAPI EnableBlob() = 0;

					/**
					@brief Disables person mask on depth image
					*/
					virtual void PXCAPI DisableBlob() = 0;

					/**
					@brief Get current state of person mask on depth image feature(enabled/disabled)
					 @return bool true if feature is enabled, else false
					*/
					virtual bool32_t PXCAPI IsBlobEnabled() = 0;
#ifdef PT_MW_DEV
					virtual void PXCAPI EnablePersonOrientation() = 0;

					virtual void PXCAPI DisablePersonOrientation() = 0;

					virtual bool32_t PXCAPI IsPersonOrientationEnabled() = 0;
#endif
					/**
					@brief Enables head bounding box feature
					**/
					virtual void PXCAPI EnableHeadBoundingBox() = 0;

					/**
					@brief Disables head bounding box feature
					**/
					virtual void PXCAPI DisableHeadBoundingBox() = 0;

					/**
					@brief Get current state of head bounding feature(enabled/disabled)
					@return bool true if feature is enabled, else false
					**/
					virtual bool32_t PXCAPI IsHeadBoundingBoxEnabled() = 0;

					/** @brief Not in use**/
					virtual void PXCAPI SetMaxTrackedPersons(int32_t maxTrackedPersons) = 0;

					/** @brief Not in use**/
					virtual int32_t PXCAPI GetMaxTrackedPersons() const = 0;

					/**
					@brief Set tracking mode
					@param[in] mode Tracking mode
					**/
					virtual void PXCAPI SetTrackingMode(TrackingMode mode) = 0;

					/**
					@brief Get current tracking mode
					@return current tracking mode
					**/
					virtual TrackingMode PXCAPI GetTrackingMode() = 0;

					/**
					@brief Set detection mode mode
					@param[in] mode detection mode
					**/
					virtual void PXCAPI SetDetectionMode(DetectionMode mode) = 0;

					/**
					@brief Get current detection mode
					 @return DetectionMode current detection mode
					**/
					virtual DetectionMode PXCAPI GetDetectionMode() = 0;
					/**
					@brief Enable detection from far  > 3m(don't make downscale of color image for detection)
					 **/
					virtual void PXCAPI EnableDetectionFromFar() = 0;

					/**
					@brief Disable detection from far > 3m
					 **/
					virtual void PXCAPI DisableDetectionFromFar() = 0;
				};

				/**
				@class SkeletonJointsConfiguration
				@brief skeleton joints configuration object. Enable/Disable/Configure skeleton joints feature
				**/
				class SkeletonJointsConfiguration
				{
				public:
					/** @brief Not in use **/
					enum SkeletonMode
					{
						AREA_UPPER_BODY, //This includes all joints of the upper body.
						AREA_UPPER_BODY_ROUGH, //This includes only 4 points: head, chest, and hands.
						AREA_FULL_BODY_ROUGH, // not use
						AREA_FULL_BODY
					};

					virtual ~SkeletonJointsConfiguration() {}

					/**
					@brief Enables skeleton feature
					**/
					virtual void PXCAPI Enable() = 0;

					/**
					@brief Disables skeleton feature
					**/
					virtual void PXCAPI Disable() = 0;

					/**
					@brief  Get current state of skeleton feature(enabled/disabled)
					 @return bool true if feature is enabled, else false
					**/
					virtual bool32_t PXCAPI IsEnabled() = 0;

					/** Not in use **/
					virtual void PXCAPI SetMaxTrackedPersons(int32_t maxTrackedPersons) = 0;

					/** Not in use **/
					virtual void PXCAPI  SetTrackingArea(SkeletonMode area) = 0;
				};

				class PersonLyingConfiguration
				{
				public:
					enum DetectionState
					{
						SEARCH_CANDIDATES_STATE,
						CLASSIFICATION_STATE
					};

					virtual void PXCAPI Enable() = 0;

					virtual void PXCAPI Disable() = 0;

					virtual bool32_t PXCAPI IsEnabled() = 0;

					virtual void PXCAPI SetMaxTrackedPersons(int32_t maxTrackedPersons) = 0;

					virtual void PXCAPI SetDetectionState(DetectionState state) = 0;

					virtual DetectionState PXCAPI GetDetectionState() = 0;
				};

				class PoseConfiguration
				{
				public:
					virtual ~PoseConfiguration() {}

					virtual void PXCAPI Enable() = 0;

					virtual void PXCAPI Disable() = 0;

					virtual bool32_t PXCAPI IsEnabled() = 0;

					virtual void PXCAPI SetMaxTrackedPersons(int32_t maxTrackedPersons) = 0;
				};

				/**
				 @class RecognitionConfiguration
				 @brief Recogntion configuration
				 **/
				class RecognitionConfiguration
				{
				public:
					/** @enum RecognitionAsyncPolicy
					*   @brief Async recognition policies
					*/
					enum RecognitionAsyncPolicy
					{
						/*
						* Recognizing a person based on multiple frames.
						* Improves accuracy but takes longer time to return.
						* Recommended in cases of head pose and tough light environments
						*/
						//MultipleFrames,

						SingleFrame /**< Recognizing a person based on a single frame.
								     Lighter process, suitable for reasonable lighting and frontal face. */
					};

					/**
					@enum RecognitionPolicy
					@brief Recognition policies
					*/
					enum RecognitionPolicy
					{
						Standard, /**< Standard */

						/*
						* Little to no false positives.
						* Suitable for login use-case. Might take longer to recognize a person.
						*/
						//Strict,
					};

					/**
					 @class RecognitionDatabase
					 @brief recognition database
					 **/
					class RecognitionDatabase
					{
					public:
						virtual ~RecognitionDatabase() {}
						/**
						@brief Get registered users
						@return int number of registered users
						**/
						virtual int32_t PXCAPI GetNumberOfRegisteredUsers() = 0; //

						/**
						@brief Check if user exists at recognition database
						@param[in] recognitionId user recognition id
						@return bool true if user with specified recognition id exists at database
						**/
						virtual bool32_t PXCAPI Exists(int32_t recognitionId) = 0;

						/**
						@brief Remove descriptor from recognition database
						@param[in] recognitionId recognition id of user which have specified descriptorId
						@param[in] descriptorId descriptor id for removing
						 */
						virtual void PXCAPI RemoveDescriptor(int32_t recognitionId, int32_t descriptorId) = 0;

						/**
						@brief Remove user from recognition database
						@param[in] recognitionId user recognition id
						 */
						virtual void PXCAPI RemoveEntry(int32_t recognitionId) = 0; //unregister

						/**
						@brief Clear recognition database
						 */
						virtual void PXCAPI Clear() = 0;

						/**
						@brief Gets all registered ids
						@param[out] outIds User-allocated array to store output
						@param[in] userAllocatedOutIdsSize Allocated array size
						@param[out] numIdsWritten Number of registered ids written to output
						*/
						virtual void PXCAPI GetRegisteredUsersIds(int32_t* outIds, int32_t userAllocatedOutIdsSize, int32_t* numIdsWritten) = 0;

						/**
						@brief Get number of user descriptors for specific user
						@param[in] recognitionId recognition id for required user
						@return int number of descriptors for specified user
						**/
						virtual int PXCAPI GetNumberOfUserDescriptors(int recognitionId) = 0;

						/**
						@brief Get user descriptors for specific user
						@param[in] recognitionId recognition id for required user
						@param[out] outDescriptorIds array of descriptors id's for specific user
						@return bool true - success, else fail
						**/
						virtual bool PXCAPI GetUserDescriptorIds(int recognitionId, int* outDescriptorIds) = 0;
					};

					/**
					 @class RecognitionDatabaseUtilities
					 @brief recognition database utilities (serialization/deserialization)
					 **/
					class RecognitionDatabaseUtilities
					{
					public:
						virtual ~RecognitionDatabaseUtilities() {}

						/* Creation / Destruction */
						//virtual RecognitionDatabase* CreateDatabase() = 0;
						//virtual DestoryDatabase(RecognitionDatabase* database) = 0;

						/* Merge */
						//virtual bool MergeDatabases(RecognitionDatabase* firstDatabase, RecognitionDatabase* secondDatabase, RecognitionDatabase* MergedDatabase);

						/* Memory */

						/**
						 @brief Get memory size for recogntion database
						 @param[in] database recognition database
						 @return int database size in bytes
						 */
						virtual int32_t PXCAPI GetDatabaseMemorySize(RecognitionDatabase* database) = 0;

						/**
						 @brief Deserialize recognition database from buffer
						 @param[out] database recognition database
						 @param[in] buffer buffer with serialized recognition database
						 @param[in] bufferSize size of recogntion database
						 @return bool true - success, else fail
						 */
						virtual bool32_t PXCAPI DeserializeDatabase(RecognitionDatabase* database, unsigned char* buffer, int32_t bufferSize) = 0;

						/**
						 @brief Serialize recognition database to buffer
						 @param[in] database recognition database
						 @param[out] outBuffer buffer with serialized recognition database
						 @param[in] inAllocatedBufferSize size of buffer
						 @param[out] outWrittenBufferSize bytes written to buffer(recogntion database size)
						 @return bool true - success, else fail
						 */
						virtual bool32_t PXCAPI SerializeDatabase(RecognitionDatabase* database, unsigned char* outBuffer, int32_t inAllocatedBufferSize, int32_t* outWrittenBufferSize) = 0;

						/* FileSystem */
						//virtual bool32_t PXCAPI LoadDatabaseFromFile(RecognitionDatabase* database, const char* dbFilePath) = 0;
						//virtual bool32_t PXCAPI SaveDatabaseToFile(RecognitionDatabase* database, const char* dbFilePath) = 0;
					};

					virtual ~RecognitionConfiguration() {}

					/**
					@brief Enables recognition feature
					**/
					virtual void PXCAPI Enable() = 0;

					/**
					@brief Disables recognition feature
					**/
					virtual void PXCAPI Disable() = 0;

					/**
					@brief Get current state of recogntion feature(enabled/disabled)
					@return bool true if feature is enabled, else false
					**/
					virtual bool32_t PXCAPI IsEnabled() = 0;

					/**
					@brief Get current recognition policy
					@return RecognitionPolicy current recognition policy
					 */
					virtual RecognitionPolicy PXCAPI QueryPolicy() = 0;

					/**
					@brief Set recognition policy
					@param[in] recognitionPolicy recogntion policy
					@return bool true - success, else fail
					*/
					virtual bool32_t PXCAPI SetPolicy(RecognitionPolicy recognitionPolicy) = 0;

					/**
					@brief Get asynchronous recognition policy
					@return RecognitionAsyncPolicy current asynchronous recogntion policy
					 */
					virtual RecognitionAsyncPolicy PXCAPI QueryAsyncPolicy() = 0;

					/**
					@brief Set asynchronous recognition policy
					@param[in] asyncRecognitionPolicy recogntion policy
					@return bool true - success, else fail
					*/
					virtual bool32_t PXCAPI SetAsyncPolicy(RecognitionAsyncPolicy asyncRecognitionPolicy) = 0;

					/**
					@brief Get recognition database
					@return RecognitionDatabase recognition database instance
					 */
					virtual RecognitionDatabase* PXCAPI QueryDatabase() = 0;
					//virtual void SetDatabase(IDatabase* database) = 0; // Goes along with create/destroy

					/**
					@brief Get recogntion database utilitied
					@return RecognitionDatabaseUtilities recognition database utilities instance
					 */
					virtual RecognitionDatabaseUtilities* PXCAPI QueryDatabaseUtilities() = 0;
				};

				/** @brief Gestures configuration **/
				class GesturesConfiguration
				{
				public:
					virtual ~GesturesConfiguration() {}

					/**
					@brief Enables gesture feature
					 only enable feature, nor specific gestures
					**/
					virtual void PXCAPI Enable() = 0;

					/**
					@brief Disable gesture feature
					**/
					virtual void PXCAPI Disable() = 0;

					/**
					@brief  Get current state of gesture feature(enabled/disabled)
					@return bool true if feature is enabled, else false
					**/
					virtual bool32_t PXCAPI IsEnabled() = 0;

					/**
					@brief not in use
					**/
					virtual void PXCAPI SetMaxTrackedPersons(int32_t maxTrackedPersons) = 0;

					/**
					 @brief Enable detection for specific gesture type
					 @param[in] gestureType gesture type
					 */
					virtual void PXCAPI EnableGesture(GestureType gestureType) = 0;

					/**
					 @brief Enable detection of all supported gestures
					 */
					virtual void PXCAPI EnableAllGestures() = 0;

					/**
					 @brief Disable detection for specific gesture type
					 @param[in] gestureType gesture type
					 */
					virtual void PXCAPI DisableGesture(GestureType gestureType) = 0;

					/**
					 @brief Disable detection of all supported gestures
					 */
					virtual void PXCAPI DisableAllGestures() = 0;
				};

				class ExpressionsConfiguration
				{
				public:
					virtual ~ExpressionsConfiguration()
					{}

					virtual void PXCAPI Enable() = 0;

					virtual void PXCAPI Disable() = 0;

					virtual bool32_t PXCAPI IsEnabled() = 0;

					virtual void PXCAPI SetMaxTrackedPeople(int32_t maxTrackedPeople) = 0;

					virtual void PXCAPI EnableAllExpressions() = 0;

					virtual void PXCAPI DisableAllExpressions() = 0;

					virtual Status PXCAPI EnableExpression(PersonExpressionsEnum expression) = 0;

					virtual void PXCAPI DisableExpression(PersonExpressionsEnum expression) = 0;

					virtual bool32_t PXCAPI IsExpressionEnabled(PersonExpressionsEnum expression) = 0;
				};

				/**
				@class PersonFaceConfiguration
				@brief person face configuration object
				**/
				class PersonFaceConfiguration
				{
				public:
					virtual ~PersonFaceConfiguration() {}

					/**
					@brief Enable head pose(head angular position) feature
					**/
					virtual void PXCAPI EnableHeadPose() = 0;

					/**
					@brief Disable head pose feature
					**/
					virtual void PXCAPI DisableHeadPose() = 0;

					/**
					@brief Get current state of head pose feature(enabled/disabled)
					@return bool true if feature is enabled, else false
					**/
					virtual bool32_t PXCAPI IsHeadPoseEnabled() const = 0;

					/**
					@brief Enable face landmarks feature
					**/
					virtual void PXCAPI EnableFaceLandmarks() = 0;

					/**
					@brief Disable face landmarks feature
					**/
					virtual void PXCAPI DisableFaceLandmarks() = 0;

					/**
					@brief  Get current state of face landmarks feature(enabled/disabled)
					@return bool true if feature is enabled, else false
					**/
					virtual bool32_t PXCAPI IsFaceLandmarksEnabled() const = 0;

					/**
					@brief Enable face gaze feature
					**/
					virtual void PXCAPI EnableGaze() = 0;

					/**
					@brief Disable face gaze feature
					**/
					virtual void PXCAPI DisableGaze() = 0;

					/**
					@brief  Get current state of face gaze feature(enabled/disabled)
					@return bool true if feature is enabled, else false
					**/
					virtual bool32_t PXCAPI IsGazeEnabled() const = 0;
#ifdef PT_MW_DEV
					/**
						@brief Returns the Person Tracking Expressions Configuration interface.
						*/
						virtual ExpressionsConfiguration* PXCAPI QueryExpressions() = 0;
#endif
				};

				/**
				@brief Returns the Person Tracking Detection Configuration interface.
				@return TrackingConfiguration object for tracking feature enable/disable/configure
				*/
				virtual TrackingConfiguration* PXCAPI QueryTracking() = 0;
				/**
				@brief Returns the Person Tracking Skeleton Joints Configuration interface.
				@return SkeletonJointsConfiguration object for skeleton feature enable/disable/configure
				*/
				virtual SkeletonJointsConfiguration* PXCAPI QuerySkeletonJoints() = 0;
#ifdef PT_MW_DEV
				/**
			@brief Returns the Person Tracking Pose Configuration interface.
			*/
			virtual PoseConfiguration* PXCAPI QueryPose() = 0;
#endif
				/**
				@brief Returns the Person Tracking Recognition Configuration interface.
				@return RecognitionConfiguration object for recognition feature enable/disable/configure
				*/
				virtual RecognitionConfiguration* PXCAPI QueryRecognition() = 0;

				/**
				@brief Returns the Person Tracking Gestures Configuration interface.
				@return GesturesConfiguration object for gesture feature enable/disable/configure
				*/
				virtual GesturesConfiguration* PXCAPI QueryGestures() = 0;
#ifdef PT_MW_DEV
				/**
			@brief Returns the Person Lying Configuration interface.
			*/
			virtual PersonLyingConfiguration* PXCAPI QueryPersonLying() = 0;
#endif
				/**
				@brief Returns the person face configuration interface.
				@return PersonFaceConfiguration object for face feature enable/disable/configure
				*/
				virtual PersonFaceConfiguration* QueryFace() = 0;

				/**
				@enum TrackingAngles
				@brief Profile includes frontal.
				*/
				enum TrackingAngles
				{
					TRACKING_ANGLES_FRONTAL = 0,
					TRACKING_ANGLES_PROFILE,
					TRACKING_ANGLES_ALL
				};

				/**
				@brief Sets the range of user angles to be tracked.
				*/
				virtual void PXCAPI SetTrackedAngles(TrackingAngles angles) = 0;


				/* Event Handlers */

				/**
				@class AlertHandler
				@brief  alert handler
				*/
				class AlertHandler {
				public:
					/**
					@brief Called when a registered alert event is fired.
					*/
					virtual void PXCAPI OnFiredAlert(const AlertData & alertData) = 0;
				};
#ifdef PT_MW_DEV
				/* Tracking Configuration */

			/**
			@brief Restarts the tracking process and resets all of the output data.
			*/
			virtual Status PXCAPI ResetTracking() = 0;

			/* Alerts Configuration */

			/**
			@brief Enables alert messaging for a specific event.
			*/
			virtual Status PXCAPI EnableAlert(AlertType alertEvent) = 0;

			/**
			@brief Enables all alert messaging events.
			*/
			virtual Status PXCAPI EnableAllAlerts(void) = 0;

			/**
			@brief Tests the activation status of the given alert.
			*/
			virtual bool32_t PXCAPI IsAlertEnabled(AlertType alertEvent) const = 0;

			/**
			@brief Disables alert messaging for a specific event.
			*/
			virtual Status PXCAPI DisableAlert(AlertType alertEvent) = 0;

			/**
			@brief Disables messaging for all alerts.
			*/
			virtual Status PXCAPI DisableAllAlerts(void) = 0;

			/**
			@brief Registers an event handler object for the alerts.
			*/
			virtual Status PXCAPI SubscribeAlert(AlertHandler *alertHandler) = 0;

			/**
			@brief Unsubscribes an alert handler object.
			*/
			virtual Status PXCAPI UnsubscribeAlert(AlertHandler *alertHandler) = 0;
#endif
			};

			typedef PersonTrackingConfiguration::TrackingStrategyType TrackingStrategyType;
			typedef PersonTrackingConfiguration::TrackingConfiguration TrackingConfiguration;
			typedef PersonTrackingConfiguration::TrackingConfiguration::TrackingMode TrackingMode;
			typedef PersonTrackingConfiguration::SkeletonJointsConfiguration SkeletonJointsConfiguration;
			typedef PersonTrackingConfiguration::SkeletonJointsConfiguration::SkeletonMode SkeletonMode;
			typedef PersonTrackingConfiguration::PoseConfiguration PoseConfiguration;
			typedef PersonTrackingConfiguration::RecognitionConfiguration RecognitionConfiguration;
			typedef PersonTrackingConfiguration::GesturesConfiguration GesturesConfiguration;
			typedef PersonTrackingConfiguration::ExpressionsConfiguration ExpressionsConfiguration;
			typedef PersonTrackingConfiguration::TrackingAngles TrackingAngles;
			typedef PersonTrackingConfiguration::PersonFaceConfiguration PersonFaceConfiguration;
		}
	}
}
