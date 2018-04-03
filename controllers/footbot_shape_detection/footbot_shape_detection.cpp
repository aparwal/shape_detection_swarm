/* Include the controller definition */
#include "footbot_shape_detection.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
#include <vector>
#include <math.h>

/****************************************/
/****************************************/

FootBotShapeDetection::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void FootBotShapeDetection::SDiffusionParams::Init(TConfigurationNode& t_node) {
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

void FootBotShapeDetection::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

void FootBotShapeDetection::SCagingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_robot_distance", TargetRobotDistance);
      GetNodeAttribute(t_node, "target_object_distance", TargetObjectDistance);
      GetNodeAttribute(t_node, "robot_gain", RobotGain);
      GetNodeAttribute(t_node, "object_gain", ObjectGain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real FootBotShapeDetection::SCagingInteractionParams::GeneralizedLennardJones(Real f_distance,Real TargetDistance,Real Gain) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

void FootBotShapeDetection::SStateData::Reset() {
   State = SEARCH_OBJECT;
   ObjectVisibility = false;
   ObjectReached = false;
   TimeInMoveAround = 0;
   
}

void FootBotShapeDetection::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "reach_distance", ReachDistance);
      GetNodeAttribute(t_node, "min_time", MinimumMoveAroundTime);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}  


FootBotShapeDetection::SStateData::SStateData(){
   Reset();
}

/****************************************/
/****************************************/

FootBotShapeDetection::FootBotShapeDetection() :
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
void FootBotShapeDetection::Init(TConfigurationNode& t_node) {
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
      m_sCagingParams.Init(GetNode(t_node, "caging"));
      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the footbot_shape_detection controller for robot \"" << GetId() << "\"", ex);
   }
   /* Enable camera filtering */
   m_pcCamera->Enable();
    m_sStateData.b=99;
   if (GetId() == "fb0")
    m_sStateData.b=10;
  // m_sStateData.b = 33;
}

/****************************************/
/****************************************/

void FootBotShapeDetection::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Set LED color */
   // m_pcLEDs->SetAllColors(CColor::BLACK);
   m_pcRABA->ClearData();
   // m_pcRABA->SetData(0, m_sStateData.b);
}

/****************************************/
/****************************************/

void FootBotShapeDetection::SetWheelSpeedsFromVector(const CVector2& c_heading) {
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

CVector2 FootBotShapeDetection::DiffusionVector(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
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

CVector2 FootBotShapeDetection::VectorToObject() {
    m_sStateData.ObjectVisibility = false;
    m_sStateData.ObjectReached = false;
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
            
                // cAccum +=current;

                /* Find the closest reading */
                if (current.SquareLength() < nearest.SquareLength()){
                    nearest = current;
                }

                /*Change state variable*/            
                m_sStateData.ObjectVisibility = true;
            }
        }
    }

    /* Set Reached flag and return*/
    if (m_sStateData.ObjectVisibility){

        if(nearest.Length() < m_sStateData.ReachDistance)
            m_sStateData.ObjectReached = true;
        else
            m_sStateData.ObjectReached = false;
        return nearest;
    }
    else
        return CVector2(0,0);
}
/****************************************/
/****************************************/
CVector2 FootBotShapeDetection::CagingVector2() {
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    // std::vector<int> rangelist;
    int first=0,second=1;
    if (tPackets.size() >= 2)
    {
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
      m_sStateData.b = (tPackets[first].HorizontalBearing-tPackets[second].HorizontalBearing).GetValue();
      if (std::abs(std::cos(m_sStateData.b)) < 0.5)
        m_sStateData.b = 33;
    }
    else
      m_sStateData.b=999;
    // if (anglelist.size() == 2)
    // {
      // if(std::abs(std::cos((anglelist[0]-anglelist[1]).GetValue())) < 0.5)
        // m_sStateData.b = 00;
        // m_pcLEDs->SetSingleColor(12, CColor::RED);
    // }
          
}
/****************************************/
/****************************************/

CVector2 FootBotShapeDetection::CagingVector() {
    m_sStateData.ObjectVisibility = false;
    m_sStateData.ObjectReached = false;
    /* Get the camera readings */
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
    /* Go through the camera readings to calculate the caging interaction vector */
    if(! sReadings.BlobList.empty()) {
        CVector2 cAccum,current,nearest= CVector2(1000,0);
        Real fLJ;
        size_t unBlobsSeen = 0;

        for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {

            /*
            * The camera perceives the light as a yellow blob
            * The robots have their red beacon on
            * So, consider only red blobs
            * In addition: consider only the closest neighbors, to avoid
            * attraction to the farthest ones. Taking 180% of the target
            * distance is a good rule of thumb.
            */
            if(sReadings.BlobList[i]->Color == CColor::RED &&
                sReadings.BlobList[i]->Distance < m_sCagingParams.TargetRobotDistance * 1.80f) {
                /*
                 * Take the blob distance and angle
                 * With the distance, calculate the Lennard-Jones interaction force
                 * Form a 2D vector with the interaction force and the angle
                 * Sum such vector to the accumulator
                 */
                /* Calculate LJ */
                fLJ = m_sCagingParams.GeneralizedLennardJones(sReadings.BlobList[i]->Distance,
                    m_sCagingParams.TargetRobotDistance,m_sCagingParams.RobotGain);
                /* Sum to accumulator */
                cAccum += CVector2(fLJ, sReadings.BlobList[i]->Angle);
                /* Increment the blobs seen counter */
                ++unBlobsSeen;
            }

            if(sReadings.BlobList[i]->Color == CColor::GREEN &&
                sReadings.BlobList[i]->Distance < m_sCagingParams.TargetObjectDistance * 1.80f) {

                current = CVector2(sReadings.BlobList[i]->Distance,
                               sReadings.BlobList[i]->Angle);

                /* Find the closest reading */
                if (current.SquareLength() < nearest.SquareLength()){
                    nearest = current;
                }

                /*set visibility flag*/            
                m_sStateData.ObjectVisibility = true;
            }
        }
        /* final LJ for robot*/
        if (m_sStateData.ObjectVisibility){
            /* Calculate LJ */
            fLJ = m_sCagingParams.GeneralizedLennardJones(nearest.Length(),
                m_sCagingParams.TargetObjectDistance,m_sCagingParams.ObjectGain);
            /* Sum to accumulator */
            cAccum += CVector2(fLJ, nearest.Angle());
            /* Divide the accumulator by the number of blobs seen */
            cAccum /= (unBlobsSeen+1);
            // LOGERR<<unBlobsSeen<< std::endl;
            /* Clamp the length of the vector to the max speed */
            if(cAccum.Length() > m_sWheelTurningParams.MaxSpeed) {
                cAccum.Normalize();
                cAccum *= m_sWheelTurningParams.MaxSpeed;
            }

            /* Set reached flag*/
            if(nearest.Length() < m_sStateData.ReachDistance)
                m_sStateData.ObjectReached = true;
            else
                m_sStateData.ObjectReached = false;
        
            return cAccum;
        }
        else
            return CVector2();
    }
    else
         return CVector2();
}
/****************************************/
/****************************************/


/**************************************/
/****************************************/

void FootBotShapeDetection::ControlStep() {
  
    switch(m_sStateData.State){
        case SStateData::SEARCH_OBJECT:{
            // m_pcLEDs->SetSingleColor(12, CColor::WHITE);
            SearchObject();
            break;
        }
        case SStateData::APPROACH_OBJECT:{
            m_pcLEDs->SetSingleColor(12, CColor::BLUE);
            ApproachObject();
            break;
        }
        case SStateData::CAGE_OBJECT:{
            m_pcLEDs->SetSingleColor(12, CColor::RED);
            CageObject();
            m_pcRABA->SetData(0, (int)m_sStateData.b);
            CagingVector2();
            break;
        }
        default: 
            LOGERR << "Unkown State: "<<m_sStateData.State << std::endl;
    }
    UpdateState();
    
    

}
   

/****************************************/
/****************************************/

void FootBotShapeDetection::UpdateState() {
    switch(m_sStateData.State){
        case SStateData::SEARCH_OBJECT:{
            if (m_sStateData.ObjectVisibility)
            {
                m_sStateData.State = SStateData::APPROACH_OBJECT;
            }
            break;
        }
        case SStateData::APPROACH_OBJECT:{
            if (m_sStateData.ObjectReached)
                m_sStateData.State = SStateData::CAGE_OBJECT;

            else if(!m_sStateData.ObjectVisibility)
                m_sStateData.State = SStateData::SEARCH_OBJECT;

            break;
        }
        case SStateData::CAGE_OBJECT:{
            if(!m_sStateData.ObjectVisibility)
                m_sStateData.State = SStateData::SEARCH_OBJECT;
            else if(!m_sStateData.ObjectReached)
                m_sStateData.State = SStateData::APPROACH_OBJECT;
            break;
        }
        default: 
            LOGERR << "Unkown State: "<<m_sStateData.State << std::endl;
    }

}

/****************************************/
/****************************************/

void FootBotShapeDetection::SearchObject() {

   /* Get the diffusion vector to perform obstacle avoidance */
   bool bCollision;
   CVector2 cDiffusion = DiffusionVector(bCollision);

   /* Get vector to nearest object location*/
   CVector2 cObjVec = VectorToObject();

   // if (m_sStateData.ObjectVisibility)
   //    /*Move towards object while avoiding collisions*/
   //    SetWheelSpeedsFromVector(0.5*cVec2Obj+0.5*m_sWheelTurningParams.MaxSpeed * cDiffusion );
   // else
      /* Random walk*/
   SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion );
}

/****************************************/
/****************************************/

void FootBotShapeDetection::ApproachObject() {
    /* Get the diffusion vector to perform obstacle avoidance */
    bool bCollision;
    CVector2 cDiffusion = DiffusionVector(bCollision)*m_sWheelTurningParams.MaxSpeed;//.Scale(m_sWheelTurningParams.MaxSpeed,m_sWheelTurningParams.MaxSpeed);

    /* Get vector to nearest object location*/
    CVector2 cObjVec = VectorToObject().Normalize()*m_sWheelTurningParams.MaxSpeed;

    /* Get vector to nearest object location*/
    // CVector2 cVec2Obj = CVector2(m_sWheelTurningParams.MaxSpeed,cDiffusion.Angle()-CRadians(1.6));


    /*Move towards object while avoiding collisions*/
    if (bCollision){
        m_pcLEDs->SetSingleColor(12, CColor::WHITE);
       SetWheelSpeedsFromVector(0.5*cObjVec.Rotate(CRadians(2))+0.5*cDiffusion);
    }
    else{
        m_pcLEDs->SetSingleColor(12, CColor::BLUE);
        SetWheelSpeedsFromVector(cObjVec+0*cDiffusion );
    }
}

/****************************************/
/****************************************/

void FootBotShapeDetection::CageObject(){

    // m_pcWheels->SetLinearVelocity(0,0);
    // m_pcRABA->SetData(0, SStateData::CAGE_OBJECT);
    // const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    // if(!m_sStateData.ObjectVisibility)
    //     m_pcWheels->SetLinearVelocity(-1,-1);
    // else
        SetWheelSpeedsFromVector(CagingVector());
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
REGISTER_CONTROLLER(FootBotShapeDetection, "footbot_shape_detection_controller")
