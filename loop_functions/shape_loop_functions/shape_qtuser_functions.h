#ifndef SHAPE_QTUSER_FUNCTIONS_H
#define SHAPE_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class ShapeQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   ShapeQTUserFunctions();

   virtual ~ShapeQTUserFunctions() {}

   void Draw(CFootBotEntity& c_entity);
   
};

#endif
