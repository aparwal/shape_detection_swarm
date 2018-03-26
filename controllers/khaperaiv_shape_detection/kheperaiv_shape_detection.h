#ifndef KHEPERAIV_SHAPE_DETECTION_H
#define KHEPERAIV_SHAPE_DETECTION_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKheperaIVShapeDetection : public CCI_Controller {

public:

   /* Class constructor. */
   CKheperaIVShapeDetection();

   /* Class destructor. */
   virtual ~CKheperaIVShapeDetection() {}

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
    * Contains all the state information about the controller.
    */
   struct SStateData {
      /* The two possible states in which the controller can be */
      enum EState {
         STATE_SEARCH_OBJECT = 0,
         STATE_APPROACH_OBJECT,
         STATE_CAGE_OBJECT,
         STATE_CHECK_VERTEX,
         STATE_COUNT_VERTEX,
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
   };

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><kheperaiv_shape_detection_controller> section.
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

private:
   /*
    * Updates the state information.
    */
   void UpdateState();

   /* Get vector for a pure diffsuion type walk*/
   CVector2 DiffusionVector(bool& );

   /* Get vector pointing towards object, length roughly equal to distance*/
   // CVector2 Vector2Object();

   /*State specific functions will go here*/
   // TODO
   void SearchObject();


   /* Sets the wheel speeds such that it ultimately follows the given vector*/
   void SetWheelSpeedsFromVector(const CVector2&);

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the Khepera IV proximity sensor */
   CCI_KheperaIVProximitySensor* m_pcProximity;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><kheperaiv_shape_detection_controller> section.
    */
 
   /* The random number generator */
   // CRandom::CRNG* m_pcRNG;
   /* The controller state information */
   SStateData m_sStateData;
  /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;
   /* The diffusion parameters */
   SDiffusionParams m_sDiffusionParams;

};

#endif
