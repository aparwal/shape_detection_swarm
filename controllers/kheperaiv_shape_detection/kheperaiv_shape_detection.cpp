/* Include the controller definition */
#include "kheperaiv_shape_detection.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CKheperaIVShapeDetection::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CKheperaIVShapeDetection::SDiffusionParams::Init(TConfigurationNode& t_node) {
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

void CKheperaIVShapeDetection::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

void CKheperaIVShapeDetection::SStateData::Reset() {
   State = STATE_SEARCH_OBJECT;
   ObjectVisibility = false;
   ObjectReached = false;
   TimeInMoveAround = 0;
}

void CKheperaIVShapeDetection::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "reach_distance", ReachDistance);
      GetNodeAttribute(t_node, "min_time", MinimumMoveAroundTime);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}  


CKheperaIVShapeDetection::SStateData::SStateData(){
   Reset();
}

/****************************************/
/****************************************/

CKheperaIVShapeDetection::CKheperaIVShapeDetection() :
   m_pcWheels(NULL),
   m_pcProximity(NULL)
   // m_pcLEDs(NULL),
   // m_pcLight(NULL),
   // m_pcRNG(NULL),
   // m_pcCamera(NULL) 
   {}

/****************************************/
/****************************************/
void CKheperaIVShapeDetection::Init(TConfigurationNode& t_node) {
  try{
    /*
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><kheperaiv_shape_detection_controller><actuators> and
    * <controllers><kheperaiv_shape_detection_controller><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_KheperaIVProximitySensor    >("kheperaiv_proximity"  );
   
    //  m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
    //  m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
    //  m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
    //  m_pcCamera    = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      // m_pcLEDs->SetAllColors(CColor::BLACK);
      

      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the kheperaiv_shape_detection controller for robot \"" << GetId() << "\"", ex);
   }
   /* Enable camera filtering */
   // m_pcCamera->Enable();
}

/****************************************/
/****************************************/

void CKheperaIVShapeDetection::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Set LED color */
   // m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

void CKheperaIVShapeDetection::SetWheelSpeedsFromVector(const CVector2& c_heading) {
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

CVector2 CKheperaIVShapeDetection::DiffusionVector(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}

/****************************************/
/****************************************/

//  CVector2 CKheperaIVShapeDetection::Vector2Object() {
//    m_sStateData.ObjectVisibility = false;
//    // m_sStateData.ObjectReached = false;
//    CVector2 cAccum;
//    size_t unBlobsSeen = 0;   

//    /* Get the camera readings */
//    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   
//    /* Go through the camera readings to locate oject */
//    if(! sReadings.BlobList.empty()) {

//       for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
//          /*Look for green*/
//          if(sReadings.BlobList[i]->Color == CColor::GREEN) {
         
//             cAccum += CVector2(sReadings.BlobList[i]->Distance,
//                                sReadings.BlobList[i]->Angle);

//             /*Change state variable*/            
//             m_sStateData.ObjectVisibility = true;
//             /* Increment the blobs seen counter */
//             ++unBlobsSeen;
//             /*Check if object is close*/
//          }
//       }

//    }

//    if (m_sStateData.ObjectVisibility){
//       cAccum /= unBlobsSeen;
//       if(cAccum.Length() < m_sStateData.ApproachDistance)
//          m_sStateData.ObjectReached = true;
//       else
//          m_sStateData.ObjectReached = false;
//    }

//    return cAccum;

// }

/****************************************/
/****************************************/

/****************************************/
/****************************************/


/****************************************/
/****************************************/

void CKheperaIVShapeDetection::ControlStep() {


  SearchObject();


}
   

/****************************************/
/****************************************/

void CKheperaIVShapeDetection::UpdateState() {

}

/****************************************/
/****************************************/

void CKheperaIVShapeDetection::SearchObject() {

   /* Get the diffusion vector to perform obstacle avoidance */
   bool bCollision;
   CVector2 cDiffusion = DiffusionVector(bCollision);

   // if (m_sStateData.ObjectVisibility)
   //    /*Move towards object while avoiding collisions*/
   //    SetWheelSpeedsFromVector(0.5*cVec2Obj+0.5*m_sWheelTurningParams.MaxSpeed * cDiffusion );
   // else
      /* Random walk*/
   SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion );
}

/****************************************/
/****************************************/

/****************************************/
/****************************************/

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
REGISTER_CONTROLLER(CKheperaIVShapeDetection, "kheperaiv_shape_detection_controller")
