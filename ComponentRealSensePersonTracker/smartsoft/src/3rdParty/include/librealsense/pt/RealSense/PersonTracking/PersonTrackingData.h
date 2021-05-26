/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2013-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/
/** @file PersonTrackingData.h
    @brief
    Describes classes: \c PersonTrackingData, \c PersonLying, \c ObjectProposal, 
	   \c PersonPose, \c PersonTracking,  
	   \c PersonFace and various structs and enums.
*/
#pragma once
#include "RealSense/Image.h"
#include "RealSense/PersonTracking/PersonTrackingModule.h"
#include <cstdint>

namespace Intel {
    namespace RealSense {
        namespace PersonTracking {

            /**
            @brief
			Describes person tracking data
            */
            class PersonTrackingData : public Base
            {
            public:
                /* Constants */
                PXC_CUID_OVERWRITE(PXC_UID('P', 'O', 'T', 'D'));

                /** @enum AlertType
				@brief 
                Identifiers for the events that can be detected and fired by the person module.
                */
                enum AlertType {
                    ALERT_PERSON_DETECTED = 0x0001,         /**< The tracked person was lost before and now they can be tracked again. */
                    ALERT_PERSON_NOT_DETECTED = 0x0002,     /**< The middleware cannot find the tracked person (possibly because the person left the frame). */
                    ALERT_PERSON_OCCLUDED = 0X0004,			/**< The tracked person is occluded by another person or object. */
                    ALERT_PERSON_TOO_CLOSE = 0X0008,		/**< The person is too close. Means that the middleware might lose the person and stop tracking. */
                    ALERT_PERSON_TOO_FAR = 0X0010			/**< The person is too far. Means that the middleware might lose the person and stop tracking. */
                };

                /**
                @enum AccessOrderType
                @brief The order in which the person data can be accessed.
                */
                enum AccessOrderType {
                    ACCESS_ORDER_BY_INDEX = 0            /**<  Ordered by index of the person */
#ifdef PT_MW_DEV
                    , ACCESS_ORDER_BY_TIME               /**<   Ordered from oldest to newest person in the scene */          
                    , ACCESS_ORDER_NEAR_TO_FAR           /**<   Ordered from nearest to farthest person in scene */
                    , ACCESS_ORDER_LEFT_TO_RIGHT         /**<   Ordered from left to right in scene */
#endif
                };

                /**
                @enum TrackingState
                @brief The current state of the module: either tracking specific people or performing full detection.
                */
                enum TrackingState
                {
                    TRACKING_STATE_TRACKING = 0,        /**<  Tracking specific people */
                    TRACKING_STATE_DETECTING            /**<  Performing full detection */
                };

                /* Data Structures */

                /**
                @struct AlertData
                */
                struct AlertData
                {
                    AlertType    label;                  /**<  The type of alert. */
                    int32_t      personId;               /**<  The ID of the person that triggered the alert, if relevant and known. */
                    int64_t      timeStamp;              /**<  The time stamp in which the event occurred. */
                    int32_t      frameNumber;            /**<  The frame number in which the event occurred (relevant for recorded sequences).*/
                };

                struct PoseEulerAngles
                {
                    float yaw;
                    float pitch;
                    float roll;
                    int32_t reserved[10];
                };

                struct BoundingBox2D
                {
                    RectI32 rect;
                    int32_t confidence;
                };

				struct PointInfo
				{
					Point3DF32 point;
					int32_t confidence;
				};

				struct PointCombined
				{
					PointInfo image;
					PointInfo world;
					int32_t reserved[10];
				};

                /* Interfaces */

				/**
				@brief
				Describes objects that were detected in the frame, 
				     that were not classified as persons, but are potential candidates.
				*/
				class ObjectProposal
				{
				public:
					struct ProposedObject
					{
						BoundingBox2D boundingBox;
						Point3DF32 centerMass;
					};

					virtual int32_t PXCAPI QueryNumberOfObjects(void) const = 0;

					virtual ProposedObject PXCAPI QueryObject(int32_t index) const = 0;
				};

				/**
				@brief
				     Describes a lying candidate.
				
				     Lying candidates are objects in the frame that are suspected of being a person who is lying down. 
				     On each of these candidates you should also run a classification to check if the object is indeed a person.
				*/
				class PersonLying
				{
				public:
					enum ClassificationResult
					{
						CLASSIFIED = 0,
						NEED_DIFFERENT_VIEW_POINT,
						NOT_CLASSIFIED
					};

					/**
					@brief Queries number of lying candidates in current frame.
					*/
					virtual int32_t PXCAPI QueryNumberOfCandidates(void) = 0;

					/**
					@brief Queries candidate's position.
					@param[in] index Candidate index.
					@return Candidate position in image and world coordinates.
					*/
					virtual PointCombined PXCAPI QueryCandidatePosition(int32_t index) = 0;

					/**
					@brief Queries candidates 2D bounding box.
					@param[in] index Candidate index
					@return Candidate 2D bounding box.
					*/
					virtual BoundingBox2D PXCAPI QueryCandidate2DBoundingBox(int32_t index) = 0;

					/**
					@brief Queries candidate classification result.
					@return Candidate classification result.
					*/
					virtual ClassificationResult PXCAPI QueryCandidateClassificationResult(int32_t index) = 0;

					/**
					@brief Queries candidate classification confidence: whether it's a person or not.
					@return Candidate classification confidence.
					*/
					virtual int32_t PXCAPI QueryCandidateClassificationConfidence(int32_t index) = 0;

					/**
					@brief Queries candidate eigen value.
					@return Candidate eigen value.
					*/
					virtual float PXCAPI QueryCandidateEigenValue(int32_t index, int32_t eigIndex) = 0;

					/**
					@brief Queries candidate eigen vector.
					@return Candidate eigen vector.
					*/
					virtual Point3DF32 PXCAPI QueryCandidateEigenVector(int32_t index, int32_t eigIndex) = 0;
				};

                /**
                    @brief
                    Describes a person's pose
		         */
				class PersonPose
                {
                public:
                    /**
                    @enum PositionState
                    @brief Options for the position of the person.
                    */
                    enum PositionState
                    {
                        POSITION_LYING_DOWN = 0,         /**<  Lying down */
                        POSITION_SITTING,                /**<  Sitting */
                        POSITION_STANDING                /**<  Standing */
						//POSITION_CROUCHING,
                        //POSITION_JUMPING
                    };

                    struct PositionInfo
                    {
                        PositionState position;
                        int32_t confidence;
                    };

                    struct LeanInfo
                    {
                        PoseEulerAngles leanData;
                        int32_t confidence;
                    };

                    /**
                    @brief Gets the person's position data.
                    */
                    virtual PositionInfo PXCAPI QueryPositionState() = 0;
#ifdef PT_MW_DEV
                    /**
                    @brief Gets the direction a person is leaning towards.
                    @param[out] leanData Where the person is leaning in terms of yaw, pitch, and roll.
                    @return False, if lean data does not exist; true, otherwise.
                    */
                    virtual bool32_t PXCAPI QueryLeanData(LeanInfo& leanInfo) = 0;
#endif
                };
				
                class PersonRecognition
                {
                public:

					enum RecognitionStatus
					{
						RecognitionPassedPersonRecognized = 0,
						RecognitionPassedPersonNotRecognized,
						RecognitionFailed,
						RecognitionFailedFaceNotDetected,
						RecognitionFailedFaceNotClear,
						RecognitionFailedPersonTooFar,
						RecognitionFailedPersonTooClose,
						RecognitionFailedFaceAmbiguity
					};

                    enum RegistrationStatus
                    {
						RegistrationSuccessful = 0,
						RegistrationFailed = 1,
						RegistrationFailedAlreadyRegistered,
						RegistrationFailedFaceNotDetected,
						RegistrationFailedFaceNotClear,
						RegistrationFailedPersonTooFar,
						RegistrationFailedPersonTooClose						
                    };

					class RecognizerScoreData
					{
					public:
						virtual float PXCAPI QuerySimilarityScoreById(int userId) = 0; // Similarity score (0-100).
					};

					struct RecognizerData
					{
						int trackingId; // Person tracker id
						int recognitionId; // Person recognition unique identifier
						float similarityScore; // Similarity score (0-100)
						RecognizerScoreData* scoreData; // Recognition score data
					};

					class RegistrationHandler
					{
					public:
						virtual ~RegistrationHandler() {}
						virtual void OnAsyncRegistrationEnded(int32_t recognitionId, int32_t trackingId, int32_t descriptorId, RegistrationStatus status) = 0;
					};

					class ReinforceRegistrationHandler
					{
					public:
						virtual ~ReinforceRegistrationHandler() {}
						virtual void OnAsyncReinforceRegisterationEnded(int32_t trackingId, int32_t descriptor_Id, RegistrationStatus status) = 0;
					};

					class RecognitionHandler
					{
					public:
						virtual ~RecognitionHandler() {}
						virtual void OnAsyncRecognitionEnded(RecognizerData* recognizerData, RecognitionStatus status) = 0;
					};
                    
					/**
                    @brief Registers a user in the Recognition database.
					@param[out] outputRecognitionId Unique user ID assigned to the registered person by the Recognition module.                    
					@return Operation status: success or failure
                    */
					virtual RegistrationStatus PXCAPI RegisterUser(int32_t* outputRecognitionId, int32_t* outTrackingId, int32_t* outDescriptorId) = 0;
					virtual bool32_t PXCAPI RegisterUserAsync(RegistrationHandler* callback) = 0;

                    /**
                    @brief Uses current frame to reinforce an existing registration.
                    @param[in] recognitionId Existing user ID to reinforce.
                    @param[in] policy Registration policy used for reinforcement.
                    @return Operation status: success or failure
                    */
                    virtual RegistrationStatus PXCAPI ReinforceUserRegistration(int32_t recognitionId, int32_t* outTrackingId, int32_t* outDescriptorId) = 0;
					virtual bool32_t PXCAPI ReinforceUserRegistartionAsync(int32_t recognitionId, ReinforceRegistrationHandler* callback) = 0;

					/**
					@brief Recognizes user					
					@param[out] result - see RecognizerData
					@return Operation status: success or failure
					*/
					virtual RecognitionStatus PXCAPI RecognizeUser(RecognizerData* result) = 0;
					virtual bool32_t PXCAPI RecognizeUserAsync(RecognitionHandler* callback) = 0;
                };

                class RecognitionModuleData
                {
                public:
					enum RecognizeAllStatus
					{
						RecognizeAllPassed = 0,
						RecognizeAllUserAllocatedArrayIsSmallerThanExpected,
						RecognizeAllInternalError,
					};

					struct RecognizerDataWithStatus
					{
						int	trackingId;	 // Person tracker id
						int	recognitionId; // Person recognition unique identifier
						float similarityScore;	 // Recognition confidence, 0-100
						PersonRecognition::RecognitionStatus status;	// Status of the recognition
					};

					// @brief Handler for async recognition of all the detected persons in a frame
					class RecognizeAllHandler
					{
					public:
						virtual ~RecognizeAllHandler() {}
						/**
						@brief Callback to be used when calling async recognition of all the detected persons in a frame ended
						@param resultsArray Array with a recognition information for each person detected in the frame. 
						The array is handled by the middleware. YOu do not need to handle any memory allocations/deletions.
						@param numberOfResultsWritten Number of elements in \c resultsArray
						*/
						virtual void OnAsyncRecognitionEnded(RecognizerDataWithStatus* resultsArray, int32_t numberOfResultsWritten) = 0;
					};
					/**
					@brief Recognizes all the detected persons in the frame
					@param resultsArray User owned and allocated array of size greater or equal to the number of detected persons in the frame
					@param resultsArraySize Length of \c resultsArray
					@param[out] numberOfResultsWritten Number of elements actually written to \c resultsArray
					@return Operation status: failure is expected in case the size of \c resultsArray is not greater than or equal to the number of detected persons in the frame
					*/
					virtual RecognizeAllStatus PXCAPI RecognizeAll(RecognizerDataWithStatus* resultsArray, int32_t resultsArraySize, int32_t* numberOfResultsWritten) = 0;
					/**
					@brief Recognizes all the detected persons in the frame.
					
					The method runs in the background and is invoked on a specific frame.

					@param callback Invoked by the middleware upon completion of the background operation. May be invoked from a different thread. The callback is guaranteed 
					       to be invoked eventually, assuming that the return value of this operation is true.
					@return true if the async operation successfully submitted to background processing
					*/
					virtual bool32_t PXCAPI RecognizeAllAsync(RecognizeAllHandler* callback) = 0;

					virtual int PXCAPI QueryNumberOfOngoingAsyncRequests() = 0;
                };

                class PersonGestures
                {
                public:
                    enum GestureType
                    {
                        Pointing,
						Wave
                    };
                    /**
                    @enum WaveType
                    Wave types
                    */
					enum WaveType
					{
						WAVE_NOT_DETECTED = -1,   /**<  Wave not detected */
						WAVE_LEFT_LA = 1,		  /**<  Wave ended with hand motion to the left, in the left area */
						WAVE_RIGHT_LA = 2,		  /**<  Wave ended with hand motion to the right, in the left area */
						WAVE_LEFT_RA = 3,		  /**<  Wave ended with hand motion to the left, in the right area */
						WAVE_RIGHT_RA = 4		  /**<  Wave ended with hand motion to the right, in the right area */
					};

                    struct PointingData2D
                    {
                        PointF32 origin;
                        PointF32 direction;
                    };
                    struct PointingData3D
                    {
                        Point3DF32 origin;
                        Point3DF32 direction;
                    };
                    struct PointingInfo
                    {
                        PointingData3D worldPointingData;
                        PointingData2D colorPointingData;
                        int64_t gestureStartTimeStamp;    //The time stamp for when the gesture started.
                        int confidence;
                    };

                    /**
                    @brief Retrieves information about a person pointing their hand, in the form of a point of origin and a vector for the direction.
                    @return The structure that describes the pointing gesture.
                    */
                    virtual PointingInfo QueryPointingInfo() = 0;

                    /**
                    @brief Indicates whether a pointing gesture is occurring.
                    @return true, if the person is pointing; false, otherwise.
                    */
                    virtual bool IsPointing() = 0;

					/**
					@brief Retrieves wave gesture type.
					@return \c WaveType enum.
					*/
					virtual WaveType QueryWaveGesture() = 0;
                };

                class PersonExpressions
                {
                public:
                    enum PersonExpressionsEnum
                    {
                        NEUTRAL = 0,
                        HAPPINESS = 1,
                        SADNESS = 2,
                        SURPRISE = 3,
                        FEAR = 4,
                        ANGER = 5,
                        DISGUST = 6,
                        CONTEMPT = 7,
						MOUTH_OPEN = 8
                    };

                    struct PersonExpressionsResult
                    {
                        int32_t confidence;
                        int32_t reserved[10];
                    };

                    /**
                    @brief Queries a single expression result.
                    @param[in] expression Requested expression.
                    @param[out] expressionIntensityResult Expression intensity.
                    @return true, if expression was calculated successfully; false, if expression calculation failed.
                    */
                    virtual bool32_t PXCAPI QueryExpression(PersonExpressionsEnum expression, PersonExpressionsResult* expressionResult) const = 0;
                };

                class PersonJoints
                {
                public:
                    enum JointType
                    {
                        JOINT_ANKLE_LEFT,
                        JOINT_ANKLE_RIGHT,
                        JOINT_ELBOW_LEFT,
                        JOINT_ELBOW_RIGHT,
                        JOINT_FOOT_LEFT,
                        JOINT_FOOT_RIGHT,
                        JOINT_HAND_LEFT,
                        JOINT_HAND_RIGHT,
                        JOINT_HAND_TIP_LEFT,
                        JOINT_HAND_TIP_RIGHT,
                        JOINT_HEAD,
                        JOINT_HIP_LEFT,
                        JOINT_HIP_RIGHT,
                        JOINT_KNEE_LEFT,
                        JOINT_KNEE_RIGHT,
                        JOINT_NECK,
                        JOINT_SHOULDER_LEFT,
                        JOINT_SHOULDER_RIGHT,
                        JOINT_SPINE_BASE,
                        JOINT_SPINE_MID,
                        JOINT_SPINE_SHOULDER,
                        JOINT_THUMB_LEFT,
                        JOINT_THUMB_RIGHT,
                        JOINT_WRIST_LEFT,
                        JOINT_WRIST_RIGHT
                    };

                    struct SkeletonPoint
                    {
                        JointType jointType;
                        int32_t confidenceImage;
                        int32_t confidenceWorld;
                        Point3DF32 world;
                        PointF32   image;
                        int32_t reserved[10];
                    };

                    struct Bone
                    {
                        JointType startJoint;
                        JointType endJoint;
                        PoseEulerAngles orientation;
                        int32_t orientationConfidence;
                        int32_t reserved[10];
                    };

                    /**
                    @brief Returns the number of tracked joints.
                    */
                    virtual int32_t PXCAPI QueryNumJoints() = 0;

                    /**
                    @brief Retrieves all joints.
                    @param[out] joints Joint locations are copied into this array. 
					     The application is expected to allocate this array from the size retrieved using \c QueryNumJoints().
                    @return true, if data and parameters exist; false, otherwise.
                    */
                    virtual bool32_t PXCAPI QueryJoints(SkeletonPoint* joints) = 0;

#ifdef PT_MW_DEV
                    /**
                    @brief Returns the number of tracked bones.
                    */
                    virtual int32_t PXCAPI QueryNumBones() = 0;

                    /**
                    @brief Retrieves all bones.
                    @param[out] bones Bone locations are copied into this array. 
					     The application is expected to allocate this array from the size retrieved using \c QueryNumBones().
                    @return true, if data and parameters exist; false, otherwise.
                    */
                    virtual bool32_t PXCAPI QueryBones(Bone* bones) = 0;
#endif
                };

				/**
                @brief Person Tracking
                */
                class PersonTracking
                {
                public:
                    struct BoundingBox3D
                    {
                        Box3DF32 box;
                        int32_t confidence;
                    };

                    struct SpeedInfo
                    {
                        Point3DF32 direction;
                        float magnitude;
                    };

                    /**
                        @enum PersonOrientation
                        @brief The current orientation of the person, relative to the camera.
                    */
                    enum PersonOrientation
                    {
                        ORIENTATION_FRONTAL = 0,        /**<  Frontal   */
                        ORIENTATION_45_DEGREE_RIGHT,    /**<  45 degrees right   */
                        ORIENTATION_45_DEGREE_LEFT,     /**<  45 degrees left   */
                        ORIENTATION_PROFILE_RIGHT,      /**<  Profile right  */
                        ORIENTATION_PROFILE_LEFT,       /**<  Profile left   */
                        ORIENTATION_REAR                /**<  Rear  */
                    };

                    struct OrientationInfo
                    {
                        PersonOrientation orientation;
                        int32_t confidence;
                    };

					/**
					@enum DetectionSource 
					@brief The type of classifier that marked a detected object in the frame as a person.
					*/
					enum class DetectionSource : std::int8_t
					{
						NOT_SPECIFIED,		/**<  Unspecified  */
						FACE,				/**<  Face  */
						HEAD_SHOULDERS,		/**<  Head and shoulders  */
						UPPER_BODY,		    /**<  Upper body  */
						FULL_BODY,		    /**<  Full body  */
						LOWER_BODY,		    /**<  Lower body  */
						HEAD_LESS,			/**<  Body, without consideration of the head. Usually, in a case where the person is very close to the camera.   */
						TORSO,				/**<  Torso   */
					};

					/**
					@struct RelativeDistance
					Options for relative distance.
					*/
					enum class RelativeDistance : std::int8_t
					{
						NOT_SPECIFIED,
						IN_RANGE,
						RELATIVE_NEAR,
						RELATIVE_FAR
					};

					/**
					@struct ApproximateDistance
					@brief Can be used when returned center of mass of a person is not available or has low confidence.
					*/
					struct ApproximateDistance
					{
						float distance;
						RelativeDistance relative;
					};

                    /**
                    @brief Returns the person's unique identifier.
                    */
                    virtual int32_t PXCAPI QueryId() const = 0;

                    /**
                    @brief Returns the location and dimensions of the tracked person, represented by a 2D bounding box (defined in pixels).
                    */
                    virtual BoundingBox2D PXCAPI Query2DBoundingBox() const = 0;

                    /**
                    @brief Retrieves the 2D image mask of the tracked person.
                    */
                    virtual Image* PXCAPI QuerySegmentationImage() = 0;

                    /**
                    @brief Retrieves the center of mass of the tracked person.
                    @return Center of mass in image and world coordinates.
                    */
                    virtual PointCombined PXCAPI QueryCenterMass() = 0;

					/**
					@brief Retrieves center of mass relative to latest known distance of person.
					*/
					virtual ApproximateDistance PXCAPI QueryRelativeLocation() const = 0;

                    /**
                    @brief Returns the location and dimensions of the tracked person's head, represented by a 2D bounding box (defined in pixels).
                    */
                    virtual BoundingBox2D PXCAPI QueryHeadBoundingBox() const = 0;

                    /**
                    @brief Retrieves the 3D image mask of the tracked person.
                    */
                    virtual Image* PXCAPI QueryBlobMask() = 0;
#ifdef PT_MW_DEV
                    /**
                        @brief Calculates the orientation of the person relative to the camera. 
						
						This function does the calculations synchronously, on demand, and should only be called when needed.
                        @return The orientation of the person and the confidence of the result.
                    */
                    virtual OrientationInfo PXCAPI QueryPersonOrientation() = 0;
#endif
					/**
					@brief Returns the person detection source.
					*/
					virtual DetectionSource PXCAPI QueryDetectionSource() const = 0;

#ifdef PT_MW_DEV
                    /**
                    @brief Returns the location and dimensions of the tracked person, represented by a 3D bounding box.
                    */
                    virtual BoundingBox3D PXCAPI Query3DBoundingBox() const = 0;

                    /**
                    @brief Returns the speed of the person in 3D world coordinates.
                    @param[out] direction Direction of the movement.
                    @param[out] magnitude Magnitude of the movement in meters/second.
                    @return true if data exists; false, otherwise.
                    */
                    virtual bool32_t PXCAPI QuerySpeed(SpeedInfo& speed) const = 0;

                    /**
                    @brief Gets the number of pixels in the blob.
                    */
                    virtual int32_t PXCAPI Query3DBlobPixelCount() const = 0;

                    /**
                    @brief Retrieves the 3D blob of the tracked person.
                    @param[out] blob Array of 3D points to which the blob will be copied. The array must be allocated by the application.
                    */
                    virtual Status PXCAPI Query3DBlob(Point3DF32* blob) const = 0;

                    /**
                    @brief Retrieves the tracked person's height in millimeters.
                    @return Person height.
                    */
                    virtual float PXCAPI QueryHeight() const = 0;

                    /**
                    @brief Gets the contour size (number of points in the contour).
                    */
                    virtual int32_t PXCAPI QueryContourSize() const = 0;

                    /**
                    @brief Gets the data of the contour line.
                    */
                    virtual Status PXCAPI QueryContourPoints(PointI32* contour) = 0;
#endif
                };

				/**
				@brief Describes a person's face
				*/
				class PersonFace
				{
				public:
					struct LandmarksLocation
					{
						Point3DF32 image;
						Point3DF32 world;
					};
					struct LandmarksInfo
					{
						LandmarksLocation* landmarks;
						int32_t confidence;
						int numLandmarks;
					};

					enum class FaceGaze
					{
						NOT_SPECIFIED,
						FRONTAL,
						LEFT,
						LEFT_45,
						RIGHT,
						RIGHT_45
					};
					struct FaceGazeInfo
					{
						FaceGaze gaze;
						int32_t confidence;
					};

					/**
					@brief Returns the number of tracked landmarks
					*/
					virtual int32_t PXCAPI QueryNumLandmarks() = 0;

					/**
					@brief Gets the data of the face landmarks
					*/
					virtual LandmarksInfo* PXCAPI QueryLandmarks() = 0;

					/**
					@brief Retrieves the orientation of the tracked person's head, represented by Euler angles.
					@param[out] pose This is the orientation of the tracked person's head.
					@return true, if the output is valid; false in the case of failure.
					*/
					virtual bool32_t PXCAPI QueryHeadPose(PoseEulerAngles& pose) const = 0;

					/**
					@brief Gets the head orientation
					*/
					virtual FaceGazeInfo PXCAPI QueryGaze() const = 0;
#ifdef PT_MW_DEV
					/**
					@brief Returns the \c PersonExpressions interface.
					*/
					virtual PersonExpressions* PXCAPI QueryExpressions() = 0;
#endif
				};

                class Person
                {
                public:
                    /**
                    @brief Returns the \c PersonTracking interface.
                    */
                    virtual PersonTracking* PXCAPI QueryTracking() = 0;

                    /**
                    @brief Returns the \c PersonRecognition interface.
                    */
                    virtual PersonRecognition* PXCAPI QueryRecognition() = 0;

                    /**
                    @brief Returns the \c PersonJoints interface.
                    */
                    virtual PersonJoints* PXCAPI QuerySkeletonJoints() = 0;

                    /**
                    @brief Returns the \c PersonGestures interface.
                    */
                    virtual PersonGestures* PXCAPI QueryGestures() = 0;

                    /**
					@brief Returns the \c PersonFace interface.
					*/
					virtual PersonFace* PXCAPI QueryFace() = 0;

#ifdef PT_MW_DEV
                    /**
                    @brief Returns the \c PersonPose interface.
                    */
                    virtual PersonPose* PXCAPI QueryPose() = 0;
#endif
                };

            public:
                /* General */

                /* Person Outputs */

                /**
                @brief Returns the number of persons detected in the current frame.
                */
                virtual int32_t PXCAPI QueryNumberOfPeople(void) = 0;

                /**
                @brief Retrieves the person object data using a specific \c accessOrder and related index.
                */
                virtual Person* PXCAPI QueryPersonData(AccessOrderType accessOrder, int32_t index) = 0;

                /**
                @brief Retrieves the person object data by its unique ID.
                */
                virtual Person* PXCAPI QueryPersonDataById(int32_t personID) = 0;

                /**
                @brief Adds the person to the list of tracked people starting from the next frame.
                */
                virtual void PXCAPI StartTracking(int32_t personID) = 0;

                /**
                @brief Removes the person from tracking.
                */
                virtual void PXCAPI StopTracking(int32_t personID) = 0;

                /**
                @brief Retrieves current tracking state of the Person Tracking module.
                */
                virtual TrackingState PXCAPI GetTrackingState() = 0;

                /**
                @brief Resets the Person Tracking module.
                */
                virtual void PXCAPI ResetTracking(void) = 0;

                /**
                @brief Returns the \c PersonRecognition module interface.
                */
                virtual RecognitionModuleData* PXCAPI QueryRecognitionModuleData() = 0;
#ifdef PT_MW_DEV
				/**
				@brief Retrieves the \c PersonLying interface.
				*/
				virtual PersonLying* PXCAPI QueryPersonLying(void) = 0;

				/**
				@brief Retrieves the \c ObjectProposal interface.
				*/
				virtual ObjectProposal* PXCAPI QueryObjectProposal(void) = 0;

				/**
				@brief Sets the detection classifier that will be used in the next frame.
				*/
				virtual void PXCAPI SetDetectionClassifierForNextFrame(PersonTracking::DetectionSource classifier) = 0;


                /* Alerts Outputs */

                /**
                @brief Returns the number of fired alerts in the current frame.
                */
                virtual int32_t PXCAPI QueryFiredAlertsNumber(void) const = 0;

                /**
                @brief Gets the details of the fired alert with the given index.
                */
                virtual Status PXCAPI QueryFiredAlertData(int32_t index, AlertData & alertData) const = 0;

                /**
                @brief Returns whether the specified alert is fired in the current frame and, if so, retrieve its data.
                */
                virtual bool32_t PXCAPI IsAlertFired(AlertType alertEvent, AlertData & alertData) const = 0;
#endif
            };

            typedef PersonTrackingData::AlertType AlertType;
            typedef PersonTrackingData::AccessOrderType AccessOrderType;
            typedef PersonTrackingData::TrackingState TrackingState;
            typedef PersonTrackingData::AlertData AlertData;
            typedef PersonTrackingData::PoseEulerAngles PoseEulerAngles;
            typedef PersonTrackingData::BoundingBox2D BoundingBox2D;
            typedef PersonTrackingData::PersonPose PersonPose;
            typedef PersonTrackingData::PersonPose::PositionState PositionState;
            typedef PersonTrackingData::PersonPose::PositionInfo PositionInfo;
            typedef PersonTrackingData::PersonPose::LeanInfo LeanInfo;
            typedef PersonTrackingData::PersonRecognition PersonRecognition;
            typedef PersonTrackingData::RecognitionModuleData RecognitionModuleData;
            typedef PersonTrackingData::PersonGestures PersonGestures;
            typedef PersonTrackingData::PersonGestures::GestureType GestureType;
            typedef PersonTrackingData::PersonGestures::PointingData2D PointingData2D;
            typedef PersonTrackingData::PersonGestures::PointingData3D PointingData3D;
            typedef PersonTrackingData::PersonGestures::PointingInfo PointingInfo;
            typedef PersonTrackingData::PersonExpressions PersonExpressions;
            typedef PersonTrackingData::PersonExpressions::PersonExpressionsEnum PersonExpressionsEnum;
            typedef PersonTrackingData::PersonExpressions::PersonExpressionsResult PersonExpressionsResult;
            typedef PersonTrackingData::PersonJoints PersonJoints;
            typedef PersonTrackingData::PersonJoints::JointType JointType;
            typedef PersonTrackingData::PersonJoints::SkeletonPoint SkeletonPoint;
            typedef PersonTrackingData::PersonJoints::Bone Bone;
            typedef PersonTrackingData::PersonTracking PersonTracking;
            typedef PersonTrackingData::PersonTracking::BoundingBox3D BoundingBox3D;
            typedef PersonTrackingData::PersonTracking::SpeedInfo SpeedInfo;
            typedef PersonTrackingData::PointInfo PointInfo;
            typedef PersonTrackingData::PointCombined PointCombined;
            typedef PersonTrackingData::PersonTracking::OrientationInfo OrientationInfo;
            typedef PersonTrackingData::Person Person;
            typedef PersonTrackingData::PersonRecognition::RecognitionStatus RecognitionStatus;
			typedef PersonTrackingData::PersonRecognition::RecognizerData RecognizerData;
			typedef PersonTrackingData::PersonRecognition::RecognizerScoreData RecognizerScoreData;
            typedef PersonTrackingData::PersonRecognition::RegistrationStatus RegistrationStatus;
			typedef PersonTrackingData::RecognitionModuleData::RecognizerDataWithStatus RecognizerDataWithStatus;
			typedef PersonTrackingData::RecognitionModuleData::RecognizeAllStatus RecognizeAllStatus;
        }
    }
}
