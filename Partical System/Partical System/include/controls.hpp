#ifndef CONTROLS_HPP
#define CONTROLS_HPP

extern  float screenWidth;
extern float screenHeight;

glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();
void computeMatricesFromInputs();
extern bool mouseClicked;
extern bool CatchTeapot;
#endif
