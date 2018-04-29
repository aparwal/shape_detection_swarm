/* Include the controller definition */
#include "footbot_mapping.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
#include <vector>
#include <math.h>
#include <algorithm>
/****************************************/
/****************************************/

FootBotMapping::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void FootBotMapping::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/

void FootBotMapping::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void FootBotMapping::SStateData::Reset() {
   State = APPROACH_OBJECT;
   ObjectVisibility = false;
   ObjectReached = false;
   ObjectCaged = false;
   RobotAhead = false;
   IncomingRobotSeen = false;
   facing_object = false;
   vertex_bot = false;
   concave_vertex = false;
   time_in_map = 0;
   time_in_concave = 0;
   time_in_concave_region = 0;
   concave_region = false;
   aligned_object = false;
   step_counter = 0;
}

void FootBotMapping::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "reach_distance", ReachDistance);
      // GetNodeAttribute(t_node, "min_time", MinimumMoveAroundTime);
      GetNodeAttribute(t_node, "average_alpha", AvgAlpha);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}


FootBotMapping::SStateData::SStateData(){
   Reset();
}

/****************************************/
/****************************************/

FootBotMapping::FootBotMapping() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcLEDs(NULL),
   // m_pcLight(NULL),
   // m_pcRNG(NULL),
   m_pcCamera(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL)
   {}

/****************************************/
/****************************************/
void FootBotMapping::Init(TConfigurationNode& t_node) {
  try{
    /*
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><bootbot_shape_detection_controller><actuators> and
    * <controllers><bootbot_shape_detection_controller><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_FootBotProximitySensor    >("footbot_proximity"  );

    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
    //  m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
    m_pcCamera    = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    // m_pcLEDs->SetAllColors(CColor::BLACK);


      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Caging-related */
      // m_sCagingParams.Init(GetNode(t_node, "caging"));
      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the footbot_shape_detection controller for robot \"" << GetId() << "\"", ex);
   }
   /* Enable camera filtering */
   m_pcCamera->Enable();
}

/****************************************/
/****************************************/

void FootBotMapping::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();

}

/****************************************/
/****************************************/

void FootBotMapping::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

// CVector2 FootBotMapping::DiffusionVector(bool& b_collision) {
//    /* Get readings from proximity sensor */
//    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
//    /* Sum them together */
//    CVector2 cDiffusionVector;
//    for(size_t i = 0; i < tProxReads.size(); ++i) {
//       cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
//    }
//     /*If the angle of the vector is small enough and the closest obstacle
//       is far enough, ignore the vector and go straight, otherwise return
//       it*/
//    if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
//       cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
//       b_collision = false;
//       return CVector2::X;
//    }
//    else {
//       b_collision = true;
//       cDiffusionVector.Normalize();
//       return -cDiffusionVector;
//    }
// }

/****************************************/
/****************************************/

CVector2 FootBotMapping::VectorToObject() {

	m_sStateData.ObjectVisibility = false;
    m_sStateData.ObjectReached = false;
    m_sStateData.ObjectCaged = false;
    m_sStateData.RobotAhead = false;
    m_sStateData.IncomingRobotSeen = false;
    m_sStateData.concave_region = false;
    CRange<CRadians> FrontAngleRange(-1*CRadians::PI_OVER_FOUR,CRadians::PI_OVER_FOUR);

    CVector2 cAccum,current,nearest= CVector2(1000,0);
    size_t unBlobsSeen = 0;

    /* Get the camera readings */
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();

    /* Go through the camera readings to locate oject */
    if(! sReadings.BlobList.empty()) {

        for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
         	/*Look for green*/
            if(sReadings.BlobList[i]->Color == CColor::GREEN) {
                current = CVector2(sReadings.BlobList[i]->Distance,
                               sReadings.BlobList[i]->Angle);

                /* Find the closest reading */
                if (current.SquareLength() < nearest.SquareLength()){
                    nearest = current;
                }

                /*Change state variable*/
                m_sStateData.ObjectVisibility = true;
            }
            /*Look for red*/
            if(sReadings.BlobList[i]->Color == CColor::RED) {

                if (sReadings.BlobList[i]->Distance < m_sStateData.ReachDistance * 3.0f)

                	if (FrontAngleRange.WithinMinBoundIncludedMaxBoundIncluded(sReadings.BlobList[i]->Angle))

                		m_sStateData.RobotAhead = true;
            }
            /*Look for blue or white*/
            if ((sReadings.BlobList[i]->Color == CColor::WHITE)) {

            	// if (FrontAngleRange.WithinMinBoundIncludedMaxBoundIncluded(sReadings.BlobList[i]->Angle))

            	    m_sStateData.ObjectCaged = true;
            }
            if ((sReadings.BlobList[i]->Color == CColor::BLUE)) {

            	// if (m_sStateData.State == SStateData::CAGE_OBJECT)
            	// 	LOG << sReadings.BlobList[i]->Angle <<" id:"<< GetId().c_str()<<std::endl;

            	if (FrontAngleRange.WithinMinBoundIncludedMaxBoundIncluded(sReadings.BlobList[i]->Angle))
            		if (sReadings.BlobList[i]->Distance < m_sStateData.ReachDistance * 1.2f)
            			m_sStateData.IncomingRobotSeen = true;
            }
            /*Look for Convcave vertex or other concave region bots*/
            if (((sReadings.BlobList[i]->Color == CColor::MAGENTA)||(sReadings.BlobList[i]->Color == CColor::CYAN))&&
            	(sReadings.BlobList[i]->Distance < 1.8f * m_sStateData.ReachDistance)) {

            	// if(!m_sStateData.vertex_bot)

            	    m_sStateData.concave_region = true;
            }
        }
    }

    /* Set Reached flag and return*/
    if (m_sStateData.ObjectVisibility){

        if(nearest.Length() < m_sStateData.ReachDistance)

            m_sStateData.ObjectReached = true;

        m_sStateData.ObjVec = nearest;
        return nearest;
    }
    else
        return CVector2(0,0);
}

/****************************************/
/****************************************/
// CVector2 FootBotMapping::VectorOppositeObject(){
//     CVector2 cAccum;
//     size_t unBlobsSeen = 0;

//     /* Get the camera readings */
//     const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();

//     /* Go through the camera readings to locate oject */
//     if(! sReadings.BlobList.empty()) {
//       for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
//         /*Look for green*/
//         if(sReadings.BlobList[i]->Color == CColor::GREEN) {
//           cAccum += CVector2(1.0f,sReadings.BlobList[i]->Angle);
//           unBlobsSeen++;
//         }
//       }
//       if (unBlobsSeen > 0)
//       {
//         cAccum/=unBlobsSeen;
//         return -cAccum.Normalize();
//       }
//     }
// }



/**************************************/
/****************************************/

void FootBotMapping::ControlStep() {

    switch(m_sStateData.State){
        case SStateData::APPROACH_OBJECT:{
            // m_pcLEDs->SetSingleColor(12, CColor::BLUE);
            ApproachObject();
            break;
        }
        case SStateData::CAGE_OBJECT:{
            m_pcLEDs->SetSingleColor(12, CColor::RED);
            CageObject();
            break;
        }
        case SStateData::MAP_OBJECT:{
            
            MapObject();
            m_pcLEDs->SetSingleColor(12, CColor::WHITE);
            break;
        }
        case SStateData::AT_VERTEX:{
            
            MapObject();
            m_pcLEDs->SetSingleColor(12, CColor::YELLOW);
            break;
        }
        case SStateData::CONCAVE_VERTEX:{
        		
            MapObject();
            m_pcLEDs->SetSingleColor(12, CColor::MAGENTA);
            break;
        }
        case SStateData::CONCAVE_REGION :{
        		m_pcLEDs->SetSingleColor(12, CColor::WHITE);
        		MapObject();
        		
        		break;
        }
        default:
            LOGERR << "Unkown State: "<<m_sStateData.State <<" id:"<< GetId().c_str()<<std::endl;
    }
    UpdateState();
    // if (std::stoi(GetId().c_str()) == 12 )
    // 	LOG << m_sStateData.State << std :: endl;
}


/****************************************/
/****************************************/

void FootBotMapping::UpdateState() {
	VectorToObject();
	m_sStateData.step_counter++;
    switch(m_sStateData.State){
    	case SStateData::APPROACH_OBJECT:{
            if (m_sStateData.ObjectReached)
            	m_sStateData.State = SStateData::CAGE_OBJECT;
            // if (m_sStateData.ObjectCaged)
            // 	m_sStateData.State = SStateData::MAP_OBJECT;
            break;
        }
        case SStateData::CAGE_OBJECT:{
            // if (!m_sStateData.ObjectReached)
            // 	m_sStateData.State = SStateData::APPROACH_OBJECT;
            if (m_sStateData.ObjectCaged)
            	m_sStateData.State = SStateData::MAP_OBJECT;
            if (m_sStateData.IncomingRobotSeen){
            	m_sStateData.State = SStateData::MAP_OBJECT;
            	// LOG<<GetId()<<std::endl;
            }

            break;
        }
        case SStateData::MAP_OBJECT:{

        		if (m_sStateData.concave_region)
        			m_sStateData.State = SStateData::CONCAVE_REGION;
            if (m_sStateData.vertex_bot)
              m_sStateData.State = SStateData::AT_VERTEX;            
        		if (m_sStateData.concave_vertex)
              m_sStateData.State = SStateData::CONCAVE_VERTEX;
            
            break;
        }
        case SStateData::AT_VERTEX:{
        		if (m_sStateData.concave_vertex)
              m_sStateData.State = SStateData::CONCAVE_VERTEX;
            if (!m_sStateData.vertex_bot)
            	m_sStateData.State = SStateData::MAP_OBJECT;
            break;
        }
        case SStateData::CONCAVE_VERTEX:{
        		if (!m_sStateData.concave_vertex)
              m_sStateData.State = SStateData::AT_VERTEX;
            if (!m_sStateData.vertex_bot && !m_sStateData.concave_vertex)
            	if (!m_sStateData.concave_region)
            		m_sStateData.State = SStateData::MAP_OBJECT;
            	else
            		m_sStateData.State = SStateData::CONCAVE_REGION;

        	break;
        }
        case SStateData::CONCAVE_REGION:{
        		// m_sStateData.time_in_concave_region++;
        		// if(m_sStateData.time_in_concave_region > 100){
        		// 	if (m_sStateData.time_in_concave_region > 201)
        		// 		m_sStateData.time_in_concave_region = 0;
        		// 	m_sStateData.State = SStateData::MAP_OBJECT;
        		// 	m_pcLEDs->SetSingleColor(12, CColor::WHITE);
        		// }
        		if(m_sStateData.time_in_stable > 300){
        			m_pcLEDs->SetSingleColor(12, CColor::CYAN);
        			// LOG<<GetId()<<std::endl;
        		}

        		// if (!m_sStateData.concave_region)
          	// m_sStateData.State = SStateData::MAP_OBJECT;
            if (m_sStateData.vertex_bot)
            	m_sStateData.State = SStateData::AT_VERTEX;
        		if (m_sStateData.concave_vertex)
        			m_sStateData.State = SStateData::CONCAVE_VERTEX;

        	break;
        }

        default:
            LOGERR << "Unkown UpdateState: "<<m_sStateData.State <<" id:"<< GetId().c_str() << std::endl;
    }

}

/****************************************/
/****************************************/
void FootBotMapping::ApproachObject() {
	m_pcLEDs->SetSingleColor(12, CColor::BLUE);
	
	/*Move forward, towards the object*/
	m_pcWheels->SetLinearVelocity(10,10);
	
	/*Non caging bot move away from the object*/
	if (m_sStateData.ObjectCaged ){
		m_pcLEDs->SetSingleColor(12, CColor::WHITE);
		m_pcWheels->SetLinearVelocity(-10,-10);
	}
}
/****************************************/
/****************************************/
void FootBotMapping::CageObject() {

	Real fNormDistExp = ::pow(m_sStateData.ReachDistance / m_sStateData.ObjVec.Length(), 2);
  CVector2 Object_Distance_Correction = CVector2(-1000 / m_sStateData.ObjVec.Length() * (fNormDistExp * fNormDistExp - fNormDistExp),m_sStateData.ObjVec.Angle());


	//  if(!m_sStateData.ObjectReached){
	//  	SetWheelSpeedsFromVector(10*Object_Distance_Correction.Normalize());
	// }
	//  else
	SetWheelSpeedsFromVector(10*m_sStateData.ObjVec.Normalize().Rotate(CRadians(-1*CRadians::PI_OVER_TWO))+ 5*Object_Distance_Correction.Normalize());
}
/****************************************/
/****************************************/
void FootBotMapping::MapObject() {

	// Real fNormDistExp = ::pow(m_sStateData.ReachDistance / m_sStateData.ObjVec.Length(), 2);
 //   	CVector2 Object_Distance_Correction = CVector2(-500 / m_sStateData.ObjVec.Length() * (fNormDistExp * fNormDistExp - fNormDistExp),m_sStateData.ObjVec.Angle());
    if ((m_sStateData.ObjVec.Angle().GetAbsoluteValue() > 0.1) && (!m_sStateData.facing_object)){
        m_pcWheels->SetLinearVelocity(-3,3);

    }
    else{
        m_sStateData.facing_object = true;

        if((m_sStateData.ObjVec.Length() > (m_sStateData.ReachDistance)+0.3) && (!m_sStateData.aligned_object)){
            m_pcWheels->SetLinearVelocity(2,2);
        }
        else if((m_sStateData.ObjVec.Length() < (m_sStateData.ReachDistance)-0.3) && (!m_sStateData.aligned_object)){
            m_pcWheels->SetLinearVelocity(-2,-2);
        }
        else{

        	m_sStateData.aligned_object = true;
        	CheckForVertex();
        	// m_sStateData.vertex_bot = CheckNoDuplicateVertex();
        	m_pcWheels->SetLinearVelocity(0,0);
        	m_sStateData.time_in_map++;

        }
    }

		// CheckNoDuplicateVertex();
		if (m_sStateData.time_in_map > 0)
    	BroadcastIDs();
//m_pcWheels->SetLinearVelocity(0,0);
}

/****************************************/
/****************************************/
// void FootBotMapping::BroadcastIDs() {
// // TRANSMIT MESSAGE ---->

// 	const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
	
// 	for(size_t i = 1; i < tPackets.size(); ++i){
// 		for (size_t j = 1; j <= tPackets[i].Data[0]; ++j){		
// 			if(std::find(m_sStateData.vertex_list.begin(), m_sStateData.vertex_list.end(), tPackets[i].Data[j])!=m_sStateData.vertex_list.end()){
// 			}
// 			else{
// 				m_sStateData.vertex_list.push_back( tPackets[i].Data[j] );
// 			}
// 		}
// 	}
// 	if (m_sStateData.vertex_bot){
// 		if(std::find(m_sStateData.vertex_list.begin(), m_sStateData.vertex_list.end(), std::stoi(GetId().c_str()))!=m_sStateData.vertex_list.end()){
// 		}
// 		else{
// 			m_sStateData.vertex_list.push_back(std::stoi(GetId().c_str()));
// 		}
// 	}

// 	std::sort(m_sStateData.vertex_list.begin(), m_sStateData.vertex_list.end());

// 	LOG << GetId() << ": " <<m_sStateData.vertex_list.size() << std::endl;
	
// 	m_pcRABA->SetData(0,m_sStateData.vertex_list.size());

// 	for (int i = 1; i <= m_sStateData.vertex_list.size(); ++i)
// 	{
// 		m_pcRABA->SetData(i,m_sStateData.vertex_list[i-1]);
// 	}

// }
void FootBotMapping::BroadcastIDs() {

	const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
	int number_of_vertices=0;
	CByteArray data_stream;
	
	data_stream.Resize(0);
	UInt16 temp;

	
	for(size_t i = 0; i < tPackets.size(); ++i){
		CByteArray incoming_stream= CByteArray( tPackets[i].Data);
		for (size_t j = 0; j < 40; ++j){		
			// if ((std::stoi(GetId().c_str()) == 10) && (j<17))
				// LOG<< GetId()<<": "<<i<<": "<<j<<": "<<tPackets[i].Data[j]<<std::endl;
			
			incoming_stream>>temp;
			if (temp > m_sStateData.vertex_count[j]){
				 m_sStateData.vertex_count[j] = temp;
			}
		}
	}

	if ((m_sStateData.vertex_bot)||(m_sStateData.concave_vertex)){
		m_sStateData.vertex_count[std::stoi(GetId().c_str())] = m_sStateData.step_counter;
	}

	for (int i = 0; i < 40; ++i)
	{
		if (m_sStateData.vertex_count[i] > m_sStateData.step_counter - 30){
			number_of_vertices++;
		}
		data_stream << m_sStateData.vertex_count[i];
	}

	data_stream.Resize(500);

	// if ((std::stoi(GetId().c_str()) == 10))
	// 	LOG<<data_stream<<std::endl;
	m_pcRABA->SetData(data_stream);

	if(m_sStateData.old_number_of_vertex == number_of_vertices)
		m_sStateData.time_in_stable++;
	else
		m_sStateData.time_in_stable = 0;

	m_sStateData.old_number_of_vertex = number_of_vertices;

	LOG << GetId() << ": " <<number_of_vertices << std::endl;
}

/****************************************/
/****************************************/
bool FootBotMapping::CheckNoDuplicateVertex(){

	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& blobReadings = m_pcCamera->GetReadings();

	// if (!m_sStateData.vertex_bot)
	// 	return false;

  if(! blobReadings.BlobList.empty()) {

    for(size_t i = 0; i < blobReadings.BlobList.size(); ++i) {

      if ((blobReadings.BlobList[i]->Color == CColor::YELLOW) || (blobReadings.BlobList[i]->Color == CColor::MAGENTA)) {

        if ((blobReadings.BlobList[i]->Angle.GetValue() < 0.01) || (blobReadings.BlobList[i]->Angle.GetValue() > 3.00))  {

        		// LOG << GetId() << " " <<blobReadings.BlobList[i]->Angle.GetValue() <<std::endl;
            // m_pcLEDs->SetSingleColor(12, CColor::WHITE);
            // m_sStateData.vertex_bot = false;
            return false;
        }
      }
    }
  }
  return true;
}

/****************************************/
/****************************************/
void FootBotMapping::CheckForVertex() {
	const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    
    int first=0,second=1,left=0,right=0;
    if (tPackets.size() >= 2)
    {
    	/*Fing the two closest neighbours*/
      for(size_t i = 1; i < tPackets.size(); ++i) 
      {
        if ( tPackets[i].Range < tPackets[second].Range )
        {
          second = i;
        }
        if( tPackets[i].Range < tPackets[first].Range )
        {
          second = first;
          first = i;
        }
      }


    	/*Fing the  closest neighbours on both sides*/
      for(size_t i = 1; i < tPackets.size(); ++i) 
      {
      	if (tPackets[i].HorizontalBearing.GetValue() < 0)
	        if ( tPackets[i].Range < tPackets[left].Range )
	        {
	          left = i;
	        }

	      if (tPackets[i].HorizontalBearing.GetValue() > 0)
	        if( tPackets[i].Range < tPackets[right].Range )
	        {
	          right = i;
	        }
      }


      /*Angle between the two closest neighbours*/
      Real diff_angle = (tPackets[first].HorizontalBearing-tPackets[second].HorizontalBearing).GetAbsoluteValue();

      /*Add that to the eponential moving average*/
      m_sStateData.avg_diff_angle = (1 -m_sStateData.AvgAlpha)*m_sStateData.avg_diff_angle + m_sStateData.AvgAlpha*diff_angle;
      
       /*Add that to the eponential moving average*/
      m_sStateData.avg_left_angle = (1 -m_sStateData.AvgAlpha)*m_sStateData.avg_left_angle + m_sStateData.AvgAlpha*tPackets[left].HorizontalBearing.GetValue();
      
       /*Add that to the eponential moving average*/
      m_sStateData.avg_right_angle = (1 -m_sStateData.AvgAlpha)*m_sStateData.avg_right_angle + m_sStateData.AvgAlpha*tPackets[right].HorizontalBearing.GetValue();
      /*Set vertex flag is the angle between closest neighbours on wither side is less than 2.7 rads(should be pi, if not for multiple bots at vertex)*/
      if (std::stoi(GetId().c_str()) == 14  )
      	LOG << "vertex " << GetId()<<" "<< tPackets[right].HorizontalBearing.GetValue()<< " " << tPackets[left].HorizontalBearing.GetValue() ;
      if ((m_sStateData.avg_diff_angle < 2.5) &&
      	// (tPackets[first].HorizontalBearing.GetValue()*tPackets[second].HorizontalBearing.GetValue() < 0) && 
      	(CheckNoDuplicateVertex())){
        
        m_sStateData.vertex_bot = true;
    	}
    	else{
    		m_sStateData.vertex_bot = false;
    	}

    	/*Concave detection*/
    	/*Increase time_in_concave if both bots are behind you 
      or else decrease it*/
      if ((tPackets[left].HorizontalBearing.GetAbsoluteValue() > 1.8) ||
      	(tPackets[right].HorizontalBearing.GetAbsoluteValue() > 1.8)){

      	m_sStateData.time_in_concave++;
      }
      else{
      	m_sStateData.time_in_concave--;
      }
      if (std::stoi(GetId().c_str()) == 14  )
      	LOG<<" "<<m_sStateData.time_in_concave <<std::endl;

      /*If time in concave is above or below a certain limit, change flags*/
      if (m_sStateData.time_in_concave > 100){

      	m_sStateData.time_in_concave = 100;
      	m_sStateData.concave_vertex = true;
      	if (!CheckNoDuplicateVertex()){
	      	m_sStateData.concave_vertex = false;
	      	m_sStateData.time_in_concave = 0;
	      }

      	// if ((std::stoi(GetId().c_str()) == 13  ) ||(std::stoi(GetId().c_str()) == 12  )){
      	// 	LOG << "Cave " << GetId()<<" "<< m_sStateData.time_in_concave << " " << m_sStateData.avg_diff_angle
      	// 		<<  tPackets[first].HorizontalBearing.GetValue()<<tPackets[second].HorizontalBearing.GetValue() << std::endl;
      	// }

      }
      else if (m_sStateData.time_in_concave < -100){
      	m_sStateData.time_in_concave = -100;
      	m_sStateData.concave_vertex = false;
      	// m_sStateData.vertex_bot = false;
      }

    }
    // if (std::stoi(GetId().c_str()) == 13)
    // 	LOG << m_sStateData.State << m_sStateData.concave_vertex << m_sStateData.concave_region << std::endl;
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(FootBotMapping, "footbot_mapping_controller")
