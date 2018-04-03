#include "shape_qtuser_functions.h"
#include <controllers/footbot_shape_detection/footbot_shape_detection.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

ShapeQTUserFunctions::ShapeQTUserFunctions() {
   RegisterUserFunction<ShapeQTUserFunctions,CFootBotEntity>(&ShapeQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void ShapeQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   
   FootBotShapeDetection& cController = dynamic_cast<FootBotShapeDetection&>(c_entity.GetControllableEntity().GetController());
   FootBotShapeDetection::SStateData& sStateData = cController.GetStateData();
   // char const* pchar = sStateData.b.c_str();
   DrawText(CVector3(0.0, 0.0, 0.3),   // position
            std::to_string(sStateData.b).c_str()); // text
   
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(ShapeQTUserFunctions, "shape_qtuser_functions")
