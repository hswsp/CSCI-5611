#ifndef CONTROLS_HPP
#define CONTROLS_HPP
extern float screenWidth;
extern float screenHeight;
extern double delta;
extern glm::vec3 emitPos;
void computeMatricesFromInputs(SDL_Event& windowEvent, bool& quit);
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();

#endif