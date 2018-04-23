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
}

void FootBotMapping::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "reach_distance", ReachDistance);
      // GetNodeAttribute(t_node, "min_time", MinimumMoveAroundTime);
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
            		if (sReadings.BlobList[i]->Distance < m_sStateData.ReachDistance * 1.1f)

            			m_sStateData.IncomingRobotSeen = true;



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
            m_pcLEDs->SetSingleColor(12, CColor::WHITE);
            MapObject();
            break;
        }
        case SStateData::AT_VERTEX:{
            m_pcLEDs->SetSingleColor(12, CColor::YELLOW);
            MapObject();
            break;
        }
        default:
            LOGERR << "Unkown State: "<<m_sStateData.State <<" id:"<< GetId().c_str()<<std::endl;
    }
    UpdateState();
}


/****************************************/
/****************************************/

void FootBotMapping::UpdateState() {
	VectorToObject();
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

            if (m_sStateData.vertex_bot)
              m_sStateData.State = SStateData::AT_VERTEX;
            break;
        }
        case SStateData::AT_VERTEX:{
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
	// if(m_sStateData.ObjectVisibility)
	// 	SetWheelSpeedsFromVector(m_sStateData.ObjVec);
	// else
	m_pcWheels->SetLinearVelocity(10,10);
	
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
        m_pcWheels->SetLinearVelocity(-2,2);
    }
    else{
        m_sStateData.facing_object = true;

        if(m_sStateData.ObjVec.Length() > (m_sStateData.ReachDistance)+0.5){
            m_pcWheels->SetLinearVelocity(2,2);
        }
        else if(m_sStateData.ObjVec.Length() < (m_sStateData.ReachDistance)-0.5){
            m_pcWheels->SetLinearVelocity(-2,-2);
        }
        else{
        	m_sStateData.vertex_bot = CheckForVertex();
        	CheckNoDuplicateVertex();
        	m_pcWheels->SetLinearVelocity(0,0);
        	m_sStateData.time_in_state++;
        }
    }

		// CheckNoDuplicateVertex();
		if (m_sStateData.time_in_state > 100)
    	BroadcastIDs();
//m_pcWheels->SetLinearVelocity(0,0);
}

/****************************************/
/****************************************/
void FootBotMapping::BroadcastIDs() {
// TRANSMIT MESSAGE ---->

	const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
	
	for(size_t i = 1; i < tPackets.size(); ++i){
		for (size_t j = 1; j <= tPackets[i].Data[0]; ++j){		
			if(std::find(m_sStateData.vertex_list.begin(), m_sStateData.vertex_list.end(), tPackets[i].Data[j])!=m_sStateData.vertex_list.end()){
			}
			else{
				m_sStateData.vertex_list.push_back( tPackets[i].Data[j] );
			}
		}
	}
	if (m_sStateData.vertex_bot){
		if(std::find(m_sStateData.vertex_list.begin(), m_sStateData.vertex_list.end(), std::stoi(GetId().c_str()))!=m_sStateData.vertex_list.end()){
		}
		else{
			m_sStateData.vertex_list.push_back(std::stoi(GetId().c_str()));
		}
	}

	std::sort(m_sStateData.vertex_list.begin(), m_sStateData.vertex_list.end());

	LOG << GetId() << ": " <<m_sStateData.vertex_list.size() << std::endl;
	
	m_pcRABA->SetData(0,m_sStateData.vertex_list.size());

	for (int i = 1; i <= m_sStateData.vertex_list.size(); ++i)
	{
		m_pcRABA->SetData(i,m_sStateData.vertex_list[i-1]);
	}

}
/****************************************/
/****************************************/
void FootBotMapping::CheckNoDuplicateVertex(){

	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& blobReadings = m_pcCamera->GetReadings();

  if(! blobReadings.BlobList.empty()) {

    for(size_t i = 0; i < blobReadings.BlobList.size(); ++i) {

      if ((blobReadings.BlobList[i]->Color == CColor::YELLOW)) {

        // LOG << blobReadings.BlobList[i]->Angle.GetValue() <<std::endl;

        if (blobReadings.BlobList[i]->Angle.GetValue() > 0) {

            m_pcLEDs->SetSingleColor(12, CColor::WHITE);
            m_sStateData.vertex_bot = false;
        }
      }
    }
  }
}

/****************************************/
/****************************************/
bool FootBotMapping::CheckForVertex() {
	const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    
    int first=0,second=1;
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
        // m_sStateData.b = tPackets[i].Range;
        // if((tPackets[i].Data[0] != 0) & (tPackets[i].Range < m_sCagingParams.TargetRobotDistance * 1.30f))
          // anglelist.push_back(tPackets[i].HorizontalBearing);
      }

      /*Angle between the two closest neighbours*/
      Real diff_angle = (tPackets[first].HorizontalBearing-tPackets[second].HorizontalBearing).GetAbsoluteValue();

      if ((tPackets[first].HorizontalBearing.GetValue() > 1.8) ||
      	(tPackets[second].HorizontalBearing.GetValue() < -1.8))
      	LOG << "Cave" << GetId()<<" "<< tPackets[first].HorizontalBearing.GetValue()
      	<<tPackets[second].HorizontalBearing.GetValue() << std::endl;
      
      if (diff_angle < 2.7){

      	// LOG << GetId() << std::endl;
        return true;

    	}
    }
    return false;
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
