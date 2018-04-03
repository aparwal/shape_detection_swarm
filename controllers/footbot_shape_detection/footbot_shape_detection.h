#ifndef FOOTBOT_SHAPE_DETECTION_H
#define FOOTBOT_SHAPE_DETECTION_H

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

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class FootBotShapeDetection : public CCI_Controller {

public:

   /* Class constructor. */
   FootBotShapeDetection();

   /* Class destructor. */
   virtual ~FootBotShapeDetection() {}

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
   struct SCagingInteractionParams {
      /* Target robot-robot distance in cm */
      Real TargetRobotDistance;
      /* Gain of the robot-robot Lennard-Jones potential */
      Real RobotGain;
      /* Target robot-object distance in cm */
      Real TargetObjectDistance;
      /* Gain of the obot-object Lennard-Jones potential */
      Real ObjectGain;

      /* Exponent of the Lennard-Jones potential */
      Real Exponent;

      void Init(TConfigurationNode& t_node);
      Real GeneralizedLennardJones(Real f_distance,Real TargetDistance,Real Gain);
   };

   /*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      /* The two possible states in which the controller can be */
      enum EState {
        SEARCH_OBJECT = 0,
        APPROACH_OBJECT,
        CAGE_OBJECT,
        CHECK_VERTEX,
        COUNT_VERTEX,
      } State;

      /* True when the object is visible */
      bool ObjectVisibility;

      /* True when the object is reached */
      bool ObjectReached;

      /* Distance from object to declare "reached"*/
      Real ReachDistance;

      /* placeholder timestep variable*/
      size_t MinimumMoveAroundTime;

      /* placeholder timestep variable*/
      size_t TimeInMoveAround;

      SStateData();
      void Init(TConfigurationNode& t_node);
      void Reset();

      Real a;
      Real b;
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
   inline SStateData& GetStateData() {
      return m_sStateData;
   }

private:
   /*
    * Updates the state information.
    */
   void UpdateState();

   /* Get vector for a pure diffsuion type walk*/
   CVector2 DiffusionVector(bool& );

   /* Get vector pointing towards object, length roughly equal to distance*/
   CVector2 VectorToObject();

   /*
    * Calculates the Caging interaction vector.
    */
   virtual CVector2 CagingVector();
   CVector2 CagingVector2();

   /*State specific functions will go here*/
   // TODO
   void SearchObject();
   void ApproachObject();
   void CageObject();


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
   SCagingInteractionParams m_sCagingParams;

};

#endif
