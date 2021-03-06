//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// Please do not modify this file. It will be re-generated
// running the code generator.
//--------------------------------------------------------------------------
#ifndef _PARAMETERSTATESTRUCTCORE_HH
#define _PARAMETERSTATESTRUCTCORE_HH

#include "aceSmartSoft.hh"

#include "nlohmann/json.hpp"

#include <iostream>

// forward declaration (in order to define validateCOMMIT(ParameterStateStruct) which is implemented in derived class)
class ParameterStateStruct;

class ParameterStateStructCore
{
	friend class ParamUpdateHandler;
public:
	
		///////////////////////////////////////////
		// Internal params
		///////////////////////////////////////////
		
		/**
		 * Definition of Parameter General
		 */
		class GeneralType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			float init_relative_obj_pose_x;
			float init_relative_obj_pose_y;
			float init_relative_obj_pose_z;
			float max_detection_score;
			std::string model_path;
			float obj_depth;
			float obj_height;
			std::string obj_types;
			float obj_width;
			float rack_dimension_max_x;
			float rack_dimension_max_y;
			float rack_dimension_max_z;
			float rack_dimension_min_x;
			float rack_dimension_min_y;
			float rack_dimension_min_z;
			float relative_obj_pose_pitch;
			float relative_obj_pose_x;
			float relative_obj_pose_y;
			float relative_obj_pose_z;
			bool send_obstacle_mesh;
		
		public:
			// default constructor
			GeneralType() {
				init_relative_obj_pose_x = 0.027;
				init_relative_obj_pose_y = 0.115;
				init_relative_obj_pose_z = 0.055;
				max_detection_score = 0.016;
				model_path = "/mnt/ssh/zafh/tausch/matthias_r/rack_model_cloud.pcd";
				obj_depth = 0.04;
				obj_height = 0.105;
				obj_types = "CHOCO,SPECIAL,FROSTIES,CORN-FLAKES,SMACKS";
				obj_width = 0.07;
				rack_dimension_max_x = 2.0;
				rack_dimension_max_y = 0.8;
				rack_dimension_max_z = 2.0;
				rack_dimension_min_x = 0.0;
				rack_dimension_min_y = -0.8;
				rack_dimension_min_z = 0.1;
				relative_obj_pose_pitch = -0.2;
				relative_obj_pose_x = 0.0;
				relative_obj_pose_y = 0.115;
				relative_obj_pose_z = 0.0;
				send_obstacle_mesh = true;
			}
		
			/**
			 * here are the public getters
			 */
			inline float getInit_relative_obj_pose_x() const { return init_relative_obj_pose_x; }
			inline float getInit_relative_obj_pose_y() const { return init_relative_obj_pose_y; }
			inline float getInit_relative_obj_pose_z() const { return init_relative_obj_pose_z; }
			inline float getMax_detection_score() const { return max_detection_score; }
			inline std::string getModel_path() const { return model_path; }
			inline float getObj_depth() const { return obj_depth; }
			inline float getObj_height() const { return obj_height; }
			inline std::string getObj_types() const { return obj_types; }
			inline float getObj_width() const { return obj_width; }
			inline float getRack_dimension_max_x() const { return rack_dimension_max_x; }
			inline float getRack_dimension_max_y() const { return rack_dimension_max_y; }
			inline float getRack_dimension_max_z() const { return rack_dimension_max_z; }
			inline float getRack_dimension_min_x() const { return rack_dimension_min_x; }
			inline float getRack_dimension_min_y() const { return rack_dimension_min_y; }
			inline float getRack_dimension_min_z() const { return rack_dimension_min_z; }
			inline float getRelative_obj_pose_pitch() const { return relative_obj_pose_pitch; }
			inline float getRelative_obj_pose_x() const { return relative_obj_pose_x; }
			inline float getRelative_obj_pose_y() const { return relative_obj_pose_y; }
			inline float getRelative_obj_pose_z() const { return relative_obj_pose_z; }
			inline bool getSend_obstacle_mesh() const { return send_obstacle_mesh; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "General(";
				os << "init_relative_obj_pose_x = " << init_relative_obj_pose_x << ", ";
				os << "init_relative_obj_pose_y = " << init_relative_obj_pose_y << ", ";
				os << "init_relative_obj_pose_z = " << init_relative_obj_pose_z << ", ";
				os << "max_detection_score = " << max_detection_score << ", ";
				os << "model_path = " << model_path << ", ";
				os << "obj_depth = " << obj_depth << ", ";
				os << "obj_height = " << obj_height << ", ";
				os << "obj_types = " << obj_types << ", ";
				os << "obj_width = " << obj_width << ", ";
				os << "rack_dimension_max_x = " << rack_dimension_max_x << ", ";
				os << "rack_dimension_max_y = " << rack_dimension_max_y << ", ";
				os << "rack_dimension_max_z = " << rack_dimension_max_z << ", ";
				os << "rack_dimension_min_x = " << rack_dimension_min_x << ", ";
				os << "rack_dimension_min_y = " << rack_dimension_min_y << ", ";
				os << "rack_dimension_min_z = " << rack_dimension_min_z << ", ";
				os << "relative_obj_pose_pitch = " << relative_obj_pose_pitch << ", ";
				os << "relative_obj_pose_x = " << relative_obj_pose_x << ", ";
				os << "relative_obj_pose_y = " << relative_obj_pose_y << ", ";
				os << "relative_obj_pose_z = " << relative_obj_pose_z << ", ";
				os << "send_obstacle_mesh = " << send_obstacle_mesh << ", ";
				os << ")\n";
			}
			
		}; // end class GeneralType
		
		/**
		 * Definition of Parameter ModelCreation
		 */
		class ModelCreationType 
		{
			friend class ParamUpdateHandler;
		protected:
			/**
			 * here are the member definitions
			 */
			bool capture_model;
			bool create_model;
			std::string load_model_path;
			std::string save_model_path;
			float theta;
			float transformation_x;
			float transformation_y;
			float transformation_z;
		
		public:
			// default constructor
			ModelCreationType() {
				capture_model = false;
				create_model = false;
				load_model_path = "/home/rollenhagen/rack_models/a-frame_model_clouds/model_cloud.pcd";
				save_model_path = "/tmp/rack_model_cloud.pcd";
				theta = 0.0;
				transformation_x = -0.65;
				transformation_y = 0.35;
				transformation_z = -0.75;
			}
		
			/**
			 * here are the public getters
			 */
			inline bool getCapture_model() const { return capture_model; }
			inline bool getCreate_model() const { return create_model; }
			inline std::string getLoad_model_path() const { return load_model_path; }
			inline std::string getSave_model_path() const { return save_model_path; }
			inline float getTheta() const { return theta; }
			inline float getTransformation_x() const { return transformation_x; }
			inline float getTransformation_y() const { return transformation_y; }
			inline float getTransformation_z() const { return transformation_z; }
			
			void to_ostream(std::ostream &os = std::cout) const
			{
				os << "ModelCreation(";
				os << "capture_model = " << capture_model << ", ";
				os << "create_model = " << create_model << ", ";
				os << "load_model_path = " << load_model_path << ", ";
				os << "save_model_path = " << save_model_path << ", ";
				os << "theta = " << theta << ", ";
				os << "transformation_x = " << transformation_x << ", ";
				os << "transformation_y = " << transformation_y << ", ";
				os << "transformation_z = " << transformation_z << ", ";
				os << ")\n";
			}
			
		}; // end class ModelCreationType
		
	
		///////////////////////////////////////////
		// External params
		///////////////////////////////////////////
		
	
		///////////////////////////////////////////
		// Instance params
		///////////////////////////////////////////
		
		/**
		 * Definition of instantiated ParameterRepository CommObjectRecognitionObjects
		 */
		class CommObjectRecognitionObjectsType {
			friend class ParamUpdateHandler;
			public:
			/**
			 * Definition of instantiated ParameterSet ObjectRecognitionParameter
			 */
			class ObjectRecognitionParameterType {
				friend class ParamUpdateHandler;
				public:
				/**
				 * Definition of Parameter BELIEF_THRESHOLD
				 */
				class BELIEF_THRESHOLDType {
					friend class ParamUpdateHandler;
				
				protected:
					/**
					 * here are the member definitions
					 */
					double threshold;
					
				public:
					// default constructor
					BELIEF_THRESHOLDType() {
						threshold = 0.0;
					}
					
					/**
					 * here are the getter methods
					 */
					inline double getThreshold() const { return threshold; }
					
					void to_ostream(std::ostream &os = std::cout) const
					{
						os << "\tBELIEF_THRESHOLD(";
						os << "threshold = " << threshold << ", ";
						os << ")\n";
					}
					
				}; // end of parameter class BELIEF_THRESHOLDType
				
				/**
				 * Definition of Parameter CLOUD
				 */
				class CLOUDType {
					friend class ParamUpdateHandler;
				
				protected:
					/**
					 * here are the member definitions
					 */
					unsigned int id;
					
				public:
					// default constructor
					CLOUDType() {
						id = 0;
					}
					
					/**
					 * here are the getter methods
					 */
					inline unsigned int getId() const { return id; }
					
					void to_ostream(std::ostream &os = std::cout) const
					{
						os << "\tCLOUD(";
						os << "id = " << id << ", ";
						os << ")\n";
					}
					
				}; // end of parameter class CLOUDType
				
				/**
				 * Definition of Parameter CLUSTERING
				 */
				class CLUSTERINGType {
					friend class ParamUpdateHandler;
				
				protected:
					/**
					 * here are the member definitions
					 */
					bool doClustering;
					
				public:
					// default constructor
					CLUSTERINGType() {
						doClustering = true;
					}
					
					/**
					 * here are the getter methods
					 */
					inline bool getDoClustering() const { return doClustering; }
					
					void to_ostream(std::ostream &os = std::cout) const
					{
						os << "\tCLUSTERING(";
						os << "doClustering = " << doClustering << ", ";
						os << ")\n";
					}
					
				}; // end of parameter class CLUSTERINGType
				
				/**
				 * Definition of Parameter ESTIMATEFILLING
				 */
				class ESTIMATEFILLINGType {
					friend class ParamUpdateHandler;
				
				protected:
					/**
					 * here are the member definitions
					 */
					bool estimateObjectProperties;
					
				public:
					// default constructor
					ESTIMATEFILLINGType() {
						estimateObjectProperties = false;
					}
					
					/**
					 * here are the getter methods
					 */
					inline bool getEstimateObjectProperties() const { return estimateObjectProperties; }
					
					void to_ostream(std::ostream &os = std::cout) const
					{
						os << "\tESTIMATEFILLING(";
						os << "estimateObjectProperties = " << estimateObjectProperties << ", ";
						os << ")\n";
					}
					
				}; // end of parameter class ESTIMATEFILLINGType
				
				/**
				 * Definition of Parameter SETOBJECTID
				 */
				class SETOBJECTIDType {
					friend class ParamUpdateHandler;
				
				protected:
					/**
					 * here are the member definitions
					 */
					unsigned int id;
					
				public:
					// default constructor
					SETOBJECTIDType() {
						id = 0;
					}
					
					/**
					 * here are the getter methods
					 */
					inline unsigned int getId() const { return id; }
					
					void to_ostream(std::ostream &os = std::cout) const
					{
						os << "\tSETOBJECTID(";
						os << "id = " << id << ", ";
						os << ")\n";
					}
					
				}; // end of parameter class SETOBJECTIDType
				
				protected:
					/**
					 * internal members
					 */
					BELIEF_THRESHOLDType BELIEF_THRESHOLD;
					CLOUDType CLOUD;
					CLUSTERINGType CLUSTERING;
					ESTIMATEFILLINGType ESTIMATEFILLING;
					SETOBJECTIDType SETOBJECTID;
					
				public:
					/**
					 * public getter methods
					 */
					inline BELIEF_THRESHOLDType getBELIEF_THRESHOLD() const { return BELIEF_THRESHOLD; }
					inline CLOUDType getCLOUD() const { return CLOUD; }
					inline CLUSTERINGType getCLUSTERING() const { return CLUSTERING; }
					inline ESTIMATEFILLINGType getESTIMATEFILLING() const { return ESTIMATEFILLING; }
					inline SETOBJECTIDType getSETOBJECTID() const { return SETOBJECTID; }
					
					void to_ostream(std::ostream &os = std::cout) const
					{
						os << "ObjectRecognitionParameter(\n";
						BELIEF_THRESHOLD.to_ostream(os);
						CLOUD.to_ostream(os);
						CLUSTERING.to_ostream(os);
						ESTIMATEFILLING.to_ostream(os);
						SETOBJECTID.to_ostream(os);
						os << ")";
					}
			}; // end of parameter-set class ObjectRecognitionParameterType
			
			protected:
				/**
				 * internal members
				 */
				ObjectRecognitionParameterType ObjectRecognitionParameter;
			
			public:
				/**
				 * public getter methods
				 */
				inline ObjectRecognitionParameterType getObjectRecognitionParameter() const { return ObjectRecognitionParameter; }
				
				void to_ostream(std::ostream &os = std::cout) const
				{
					os << "CommObjectRecognitionObjects(\n";
					ObjectRecognitionParameter.to_ostream(os);
					os << ")";
				}
		}; // end of parameter-repository wrapper class CommObjectRecognitionObjectsType
	
protected:

	// Internal params
	GeneralType General;
	ModelCreationType ModelCreation;
	
	// External params
	
	// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
	CommObjectRecognitionObjectsType CommObjectRecognitionObjects;
	

	void setContent(const ParameterStateStructCore &commit) {
		// External params
	
		this->CommObjectRecognitionObjects = commit.getCommObjectRecognitionObjects();
	}

	// special trigger method (user upcall) called before updating parameter global state
	virtual SmartACE::ParamResponseType handleCOMMIT(const ParameterStateStruct &commitState) = 0;
public:
	ParameterStateStructCore() {  }
	virtual ~ParameterStateStructCore() {  }
	
	// internal param getters
	GeneralType getGeneral() const {
		return General;
	}
	ModelCreationType getModelCreation() const {
		return ModelCreation;
	}
	
	// external param getters
	
	// repo wrapper class getter(s)
	CommObjectRecognitionObjectsType getCommObjectRecognitionObjects() const {
		return CommObjectRecognitionObjects;
	}
	
	// helper method to easily implement output stream in derived classes
	void to_ostream(std::ostream &os = std::cout) const
	{
		// Internal params
		General.to_ostream(os);
		ModelCreation.to_ostream(os);
		
		// External params
		
		// Instance params (encapsulated in a wrapper class for each instantiated parameter repository)
		CommObjectRecognitionObjects.to_ostream(os);
	}
	
	std::string getAsJSONString() {
		nlohmann::json param;
	
		param["General"] = nlohmann::json {
			{"init_relative_obj_pose_x" , getGeneral().getInit_relative_obj_pose_x()},
			{"init_relative_obj_pose_y" , getGeneral().getInit_relative_obj_pose_y()},
			{"init_relative_obj_pose_z" , getGeneral().getInit_relative_obj_pose_z()},
			{"max_detection_score" , getGeneral().getMax_detection_score()},
			{"model_path" , getGeneral().getModel_path()},
			{"obj_depth" , getGeneral().getObj_depth()},
			{"obj_height" , getGeneral().getObj_height()},
			{"obj_types" , getGeneral().getObj_types()},
			{"obj_width" , getGeneral().getObj_width()},
			{"rack_dimension_max_x" , getGeneral().getRack_dimension_max_x()},
			{"rack_dimension_max_y" , getGeneral().getRack_dimension_max_y()},
			{"rack_dimension_max_z" , getGeneral().getRack_dimension_max_z()},
			{"rack_dimension_min_x" , getGeneral().getRack_dimension_min_x()},
			{"rack_dimension_min_y" , getGeneral().getRack_dimension_min_y()},
			{"rack_dimension_min_z" , getGeneral().getRack_dimension_min_z()},
			{"relative_obj_pose_pitch" , getGeneral().getRelative_obj_pose_pitch()},
			{"relative_obj_pose_x" , getGeneral().getRelative_obj_pose_x()},
			{"relative_obj_pose_y" , getGeneral().getRelative_obj_pose_y()},
			{"relative_obj_pose_z" , getGeneral().getRelative_obj_pose_z()},
			{"send_obstacle_mesh" , getGeneral().getSend_obstacle_mesh()}
		};
		param["ModelCreation"] = nlohmann::json {
			{"capture_model" , getModelCreation().getCapture_model()},
			{"create_model" , getModelCreation().getCreate_model()},
			{"load_model_path" , getModelCreation().getLoad_model_path()},
			{"save_model_path" , getModelCreation().getSave_model_path()},
			{"theta" , getModelCreation().getTheta()},
			{"transformation_x" , getModelCreation().getTransformation_x()},
			{"transformation_y" , getModelCreation().getTransformation_y()},
			{"transformation_z" , getModelCreation().getTransformation_z()}
		};
	
		param["ObjectRecognitionParameter"] = nlohmann::json {
			{ "BELIEF_THRESHOLD", {
				{"threshold" , getCommObjectRecognitionObjects().getObjectRecognitionParameter().getBELIEF_THRESHOLD().getThreshold()}
			}},
			{ "CLOUD", {
				{"id" , getCommObjectRecognitionObjects().getObjectRecognitionParameter().getCLOUD().getId()}
			}},
			{ "CLUSTERING", {
				{"doClustering" , getCommObjectRecognitionObjects().getObjectRecognitionParameter().getCLUSTERING().getDoClustering()}
			}},
			{ "ESTIMATEFILLING", {
				{"estimateObjectProperties" , getCommObjectRecognitionObjects().getObjectRecognitionParameter().getESTIMATEFILLING().getEstimateObjectProperties()}
			}},
			{ "SETOBJECTID", {
				{"id" , getCommObjectRecognitionObjects().getObjectRecognitionParameter().getSETOBJECTID().getId()}
			}}
		};
		
		return param.dump();
	}
};

#endif
