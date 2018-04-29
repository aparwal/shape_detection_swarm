#ifndef FOOTBOT_MAPPING_H
#define FOOTBOT_MAPPING_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
#include <vector>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class FootBotMapping : public CCI_Controller {

public:

   /* Class constructor. */
   FootBotMapping();

   /* Class destructor. */
   virtual ~FootBotMapping() {}

   struct SDiffusionParams {
      /*
       * Maximum tolerance for the proximity reading between
       * the robot and the closest obstacle.
       * The proximity reading is 0 when nothing is detected
       * and grows exponentially to 1 when the obstacle is
       * touching the robot.
       */
      Real Delta;
      /* Angle tolerance range to go straight. */
      CRange<CRadians> GoStraightAngleRange;

      /* Constructor */
      SDiffusionParams();

      /* Parses the XML section for diffusion */
      void Init(TConfigurationNode& t_tree);
   };

   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   /*
    * The following variables are used as parameters for
    * caging interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_shape_detection_controller><parameters><caging>
    * section.
    */
   // struct SCagingInteractionParams {
   //    /* Target robot-robot distance in cm */
   //    Real TargetRobotDistance;
   //    /* Gain of the robot-robot Lennard-Jones potential */
   //    Real RobotGain;
   //    /* Target robot-object distance in cm */
   //    Real TargetObjectDistance;
   //    /* Gain of the obot-object Lennard-Jones potential */
   //    Real ObjectGain;

   //    /* Exponent of the Lennard-Jones potential */
   //    Real Exponent;

   //    void Init(TConfigurationNode& t_node);
   //    Real GeneralizedLennardJones(Real f_distance,Real TargetDistance,Real Gain);
   // };

   /*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      /* The two possible states in which the controller can be */
      enum EState {
        APPROACH_OBJECT=0,
        CAGE_OBJECT,
        MAP_OBJECT,
        AT_VERTEX,
        CONCAVE_VERTEX,
        CONCAVE_REGION
      } State;

      /* True when the object is visible */
      bool ObjectVisibility;

      /* True when the object is reached */
      bool ObjectReached;

      /* True when the object caging conditions for a robot are met */
      bool ObjectCaged;

      /* True when a caging robot is ahead of us */
      bool RobotAhead;

      /* True when a caging robot is ahead of us */
      bool IncomingRobotSeen;

      /* Distance from object to declare "reached"*/
      Real ReachDistance;

      /*Vector to nearest point on the object*/
      CVector2 ObjVec;

      /*true when facing directly towards objeect while mapping*/  
      bool facing_object;

      /*true when facing object and at the required distance from it*/
      bool aligned_object;

      /*true when at vertex*/
      bool vertex_bot;

      /*true when at concave vertex*/
      bool concave_vertex;

      /*true when inside concave regions*/
      bool concave_region;

      /* NOTE: Should be implemented as two vectors in future, this is a quick and dirty workaround
      One a list of current vertices, another a decay vounter*/
      /*List of vertices*/
      // std::vector<int> vertex_list;//[50] = {0};
      UInt16 step_counter;
      // /*Decay of list of vertices over time*/
      UInt16 vertex_count[40] = {0};

      /*Time counter before starting to broadcast*/
      size_t time_in_map;

      /*time counter before declaring concave*/
      int time_in_concave,time_in_concave_region,time_in_stable,old_number_of_vertex,end_time=0;

      /*Exponential moving average*/
      Real avg_diff_angle, AvgAlpha,avg_left_angle,avg_right_angle;

      /* placeholder timestep variable*/
      // size_t MinimumMoveAroundTime;

      /* placeholder timestep variable*/
      // size_t TimeInMoveAround;

      SStateData();
      void Init(TConfigurationNode& t_node);
      void Reset();
   };

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_shape_detection_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   /* Function to access data in QT user functions*/
   // inline SStateData& GetStateData() {
   //    return m_sStateData;
   // }

private:
   /*
    * Updates the state information.
    */
   void UpdateState();

   /* Get vector for a pure diffsuion type walk*/
   // CVector2 DiffusionVector(bool& );

   /* Get vector pointing towards object, length roughly equal to distance*/
   CVector2 VectorToObject();

   void CheckForVertex();
   bool CheckNoDuplicateVertex();

   /*State specific functions will go here*/
   void MapObject();
   void ApproachObject();
   void CageObject();
   void BroadcastIDs();

   /* Sets the wheel speeds such that it ultimately follows the given vector*/
   void SetWheelSpeedsFromVector(const CVector2&);

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the footbot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_shape_detection_controller> section.
    */

   /* The random number generator */
   // CRandom::CRNG* m_pcRNG;
   /* The controller state information */
   SStateData m_sStateData;
  /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;
   /* The diffusion parameters */
   SDiffusionParams m_sDiffusionParams;
   /* The caging interaction parameters. */
   // SCagingInteractionParams m_sCagingParams;

};

#endif
