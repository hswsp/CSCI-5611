//// Include GLFW
//#include <GLFW/glfw3.h>
#if defined(__APPLE__) || defined(__linux__)
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#else
#include <SDL.h>
#include <SDL_opengl.h>
#endif
extern SDL_Window* window; // The "extern" keyword here is to access the variable "window" declared in tutorialXXX.cpp. This is a hack to keep the tutorials simple. Please avoid this.
#include<stdio.h>
// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "controls.hpp"

glm::mat4 ViewMatrix;
glm::mat4 ProjectionMatrix;

glm::mat4 getViewMatrix(){
	return ViewMatrix;
}
glm::mat4 getProjectionMatrix(){
	return ProjectionMatrix;
}


// Initial position : on +Z
glm::vec3 position = glm::vec3( 0, 5.0, 10.0 ); 
glm::vec3 Inidirection = emitPos- position;
// Initial horizontal angle : toward -Z
float horizontalAngle = 3.14f;
// Initial vertical angle : none
float verticalAngle = 0.0f;
// Initial Field of View
float initialFoV = 45.0f;

float speed = 3.0f; //  units / second
float mouseSpeed = 0.005f;



void computeMatricesFromInputs(SDL_Event& windowEvent, bool& quit)
{
	//static double lastTime = SDL_GetTicks()/1000;
	
	//// Compute time difference between current and last frame
	float deltaTime = delta;
	//double currentTime = SDL_GetTicks()/1000;// in second
	//float deltaTime = float(currentTime - lastTime);

	// Get mouse position
	int xpos, ypos;

	SDL_GetRelativeMouseState(&xpos, &ypos);//get the relative motion according to the customer window

	
	// Compute new orientation
	horizontalAngle += mouseSpeed * float(xpos );
	verticalAngle   += mouseSpeed * float(ypos );

	// Direction : Spherical coordinates to Cartesian coordinates conversion
	glm::vec3 direction(
		cos(verticalAngle) * sin(horizontalAngle), 
		sin(verticalAngle),
		cos(verticalAngle) * cos(horizontalAngle)
	);
	
	// Right vector
	glm::vec3 right = glm::vec3(
		sin(horizontalAngle - 3.14f/2.0f), 
		0,
		cos(horizontalAngle - 3.14f/2.0f)
	);
	
	// Up vector
	glm::vec3 up = glm::normalize(glm::cross( right, direction ));

	// Move forward
	bool ismotion = false;
	while (SDL_PollEvent(&windowEvent))
	{
		switch (windowEvent.type)
		{
		case SDL_QUIT:
			quit = true; //Exit Game Loop
			break;
		case SDL_KEYUP:
			switch (windowEvent.key.keysym.sym)
			{
			case SDLK_ESCAPE:
				quit = true; //Exit Game Loop
				break;
			case SDLK_UP:
				position += direction * deltaTime * speed;
				break;
			case SDLK_DOWN:
				position -= direction * deltaTime * speed;
				break;
			case SDLK_RIGHT:
				position += right * deltaTime * speed;
				break;
			case SDLK_LEFT:
				position -= right * deltaTime * speed;
				break;
			default:
				break;
			}
			ismotion = true;
		default:
			break;
		}
		
	}
	//printf("the position is (%f,%f,%f)\n", position.x, position.y, position.z);
    
	float FoV = initialFoV;// - 5 * glfwGetMouseWheel(); // Now GLFW 3 requires setting up a callback for this. It's a bit too complicated for this beginner's tutorial, so it's disabled instead.

	// Projection matrix : 45?Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	ProjectionMatrix = glm::perspective(glm::radians(FoV), 4.0f / 3.0f, 0.1f, 100.0f);
	// Camera matrix
	ViewMatrix  = glm::lookAt(
								position,           // Camera is here
								position+direction, // and looks here : at the same position, plus "direction"
								up                  // Head is up (set to 0,-1,0 to look upside-down)
						   );

	// For the next frame, the "last time" will be "now"
	//lastTime = currentTime;
}