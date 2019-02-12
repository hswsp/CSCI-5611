// Include GLFW
#include <chrono>
#include <thread>
#include <GLFW/glfw3.h>
extern GLFWwindow* window; 

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "include/controls.hpp"

glm::mat4 ViewMatrix;
glm::mat4 ProjectionMatrix;

glm::mat4 getViewMatrix(){
	return ViewMatrix;
}
glm::mat4 getProjectionMatrix(){
	return ProjectionMatrix;
}


// Initial CameraPos : on +Z
glm::vec3 CameraPos = glm::vec3(0, 5.0f, 5.0f);// glm::vec3(0, 0, 5);
// Initial horizontal angle : toward -Z
float horizontalAngle = 3.14f;//
// Initial vertical angle : none
//-0.785398f -45 degree  -1.57f -90 degree -0.5236 -30 degree
float verticalAngle = 0.0f ;//
// Initial Field of View
float initialFoV = 45.0f;
glm::vec3 Cameradirection;
float speed = 3.0f; // 3 units / second
float mouseSpeed = 0.001f;
// glfwGetTime is called only once, the first time this function is called
static double lastTime = glfwGetTime();
static double xposOld = screenWidth / 2;
static double yposOld = screenHeight / 2;
static bool IsInputMode = false;


void computeMatricesFromInputs()
{

	using namespace std::this_thread; // sleep_for, sleep_until
	using namespace std::chrono; // nanoseconds, system_clock, seconds

	// Compute time difference between current and last frame
	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - lastTime);

	// Get mouse CameraPos
	
	//xpos = xposOld;
	//ypos = yposOld;

	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)//
	{
		sleep_for(nanoseconds(100));
		if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
		{
			IsInputMode = !IsInputMode;
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		}
	}
	if (IsInputMode)
	{
		double xpos, ypos;
		// Hide the mouse and enable unlimited mouvement
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		glfwGetCursorPos(window, &xpos, &ypos);
		horizontalAngle += mouseSpeed * float( xposOld - xpos );
		verticalAngle += mouseSpeed * float(yposOld - ypos );
		xposOld = xpos;
		yposOld = ypos;
	}
	

	// Reset mouse CameraPos for next frame
	//glfwSetCursorPos(window, screenWidth/2,screenHeight/2);

	// Compute new orientation
	//horizontalAngle += mouseSpeed * float(xpos - xposOld);
	//verticalAngle   += mouseSpeed * float(ypos - yposOld);

	// Direction : Spherical coordinates to Cartesian coordinates conversion
	 Cameradirection=vec3(
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
	glm::vec3 up = glm::cross( right, Cameradirection);
	if (IsInputMode)
	{
		// Move forward
		if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS){
			CameraPos += Cameradirection * deltaTime * speed;
		}
		// Move backward
		if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS){
			CameraPos -= Cameradirection * deltaTime * speed;
		}
		// Strafe right
		if (glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS){
			CameraPos += right * deltaTime * speed;
		}
		// Strafe left
		if (glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS){
			CameraPos -= right * deltaTime * speed;
		}
		// put teapot
		if (CatchTeapot&&glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		{
			CatchTeapot = false;
		}
		// set off fireworks
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
		{
			mouseClicked = true;
		}

	}
	

	float FoV = initialFoV;// - 5 * glfwGetMouseWheel(); // Now GLFW 3 requires setting up a callback for this. It's a bit too complicated for this beginner's tutorial, so it's disabled instead.

	// Projection matrix : 45?Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	ProjectionMatrix = glm::perspective(glm::radians(FoV), 4.0f / 3.0f, 0.1f, 100.0f);
	// Camera matrix
	ViewMatrix       = glm::lookAt(
								CameraPos,           // Camera is here
								CameraPos+ Cameradirection, // and looks here : at the same CameraPos, plus "direction"
								up                  // Head is up (set to 0,-1,0 to look upside-down)
						   );

	// For the next frame, the "last time" will be "now"
	lastTime = currentTime;
	
}
