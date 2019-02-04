//Cube Rendering in OpenGL
//(c)Stephen J. Guy, October, 2018
//Compaignion code to: OpenGLCrashCourse.pdf

//This loads a 3D Cube
//Compiling it requries GLAD (https://glad.dav1d.de/), 
//the SDL2 *development libraries* (https://www.libsdl.org/download-2.0.php),
//and the GLM matrix library (https://glm.g-truc.net/)

//New concepts to understand: Uniforms in GLSL, GLM, storing 3D models,
//    model, view, and projection transformations, specifying cameras

#include<stdio.h>
#include "glad/glad.h"  //Include order can matter here
#if defined(__APPLE__) || defined(__linux__)
 #include <SDL2/SDL.h>
 #include <SDL2/SDL_opengl.h>
#else
#include <SDL.h>
#include <SDL_opengl.h>
#endif

#define INIT_HEIGHT 3.5
#define GRAVITY -9.8
// Shader sources
const GLchar* vertexSource =
    "#version 150 core\n"
    "in vec3 position;"
    "in vec3 inColor;"
    "out vec3 Color;"
    "uniform mat4 model;"
    "uniform mat4 view;"
    "uniform mat4 proj;"
    "void main() {"
    "   Color = inColor;"
    "   gl_Position = proj * view * model * vec4(position,1.0);"
    "}";

const GLchar* fragmentSource =
    "#version 150 core\n"
    "in vec3 Color;"
    "out vec4 outColor;"
    "void main() {"
    "   outColor = vec4(Color, 1.0);"  //(Red, Green, Blue, Alpha)
    "}";

bool fullscreen = false;
int screenWidth = 800;
int screenHeight = 600;

void loadShader(GLuint shaderID, const GLchar* shaderSource){
  glShaderSource(shaderID, 1, &shaderSource, NULL); 
  glCompileShader(shaderID);
        
  //Let's double check the shader compiled 
  GLint status; 
  glGetShaderiv(shaderID, GL_COMPILE_STATUS, &status); //Check for errors
  if (!status){
    char buffer[512]; glGetShaderInfoLog(shaderID, 512, NULL, buffer);
    printf("Shader Compile Failed. Info:\n\n%s\n",buffer);
  }
}

#define GLM_FORCE_RADIANS //ensure we are using radians
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

int main(int argc, char *argv[]) {
  SDL_Init(SDL_INIT_VIDEO);  //Initialize Graphics (for OpenGL)
    
  //Print the version of SDL we are using 
  SDL_version comp; SDL_version linked;
  SDL_VERSION(&comp); SDL_GetVersion(&linked);
  printf("\nCompiled against SDL version %d.%d.%d\n", comp.major, comp.minor, comp.patch);
  printf("Linked SDL version %d.%d.%d.\n", linked.major, linked.minor, linked.patch);
      
  //Ask SDL to get a recent version of OpenGL (3.2 or greater)
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    
  //Create a window (offsetx, offsety, width, height, flags)
  SDL_Window* window = SDL_CreateWindow("My OpenGL Program", 100, 100, 
                                        screenWidth, screenHeight, SDL_WINDOW_OPENGL);
  if (!window){
    printf("Could not create window: %s\n", SDL_GetError()); 
    return EXIT_FAILURE; //Exit as SDL failed 
  }
  float aspect = screenWidth/(float)screenHeight; //aspect ratio needs update on resize
          
  SDL_GLContext context = SDL_GL_CreateContext(window); //Bind OpenGL to the window

  if (gladLoadGLLoader(SDL_GL_GetProcAddress)){
    printf("OpenGL loaded\n");
    printf("Vendor:   %s\n", glGetString(GL_VENDOR));
    printf("Renderer: %s\n", glGetString(GL_RENDERER));
    printf("Version:  %s\n", glGetString(GL_VERSION));
  }
  else {
    printf("ERROR: Failed to initialize OpenGL context.\n");
    return -1;
  }

 
  GLfloat vertices[] = {
   // X      Y     Z     R     G      B      U      V
    -0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, //Red face
     0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
     0.5f,  0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f,
     0.5f,  0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f,
    -0.5f,  0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    -0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,

    -0.5f, -0.5f,  0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, //Green face
     0.5f, -0.5f,  0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
     0.5f,  0.5f,  0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
     0.5f,  0.5f,  0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
    -0.5f,  0.5f,  0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
    -0.5f, -0.5f,  0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,

    -0.5f,  0.5f,  0.5f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, //Yellow face
    -0.5f,  0.5f, -0.5f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f,
    -0.5f, -0.5f, -0.5f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f,
    -0.5f, -0.5f, -0.5f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f,
    -0.5f, -0.5f,  0.5f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f,
    -0.5f,  0.5f,  0.5f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f,

     0.5f,  0.5f,  0.5f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, //Blue face
     0.5f,  0.5f, -0.5f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
     0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
     0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
     0.5f, -0.5f,  0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
     0.5f,  0.5f,  0.5f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f,

    -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, //Black face
     0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f,
     0.5f, -0.5f,  0.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
     0.5f, -0.5f,  0.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    -0.5f, -0.5f,  0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,

    -0.5f,  0.5f, -0.5f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, //White face
     0.5f,  0.5f, -0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
     0.5f,  0.5f,  0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
     0.5f,  0.5f,  0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
    -0.5f,  0.5f,  0.5f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f,
    -0.5f,  0.5f, -0.5f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f,

    -0.5f, -0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 0.0f, 0.0f, //Green face
     0.5f, -0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 1.0f, 0.0f,
     0.5f,  0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 1.0f, 1.0f,
     0.5f,  0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 1.0f, 1.0f,
    -0.5f,  0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 0.0f, 1.0f,
    -0.5f, -0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 0.0f, 0.0f,
  };
 
  GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER); 
  loadShader(vertexShader, vertexSource);
  GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  loadShader(fragmentShader, fragmentSource);
          
  //Join the vertex and fragment shaders together into one program
  GLuint shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glBindFragDataLocation(shaderProgram, 0, "outColor"); // set output
  glLinkProgram(shaderProgram); //run the linker   

  GLuint vbo;
  glGenBuffers(1, &vbo);  //Create 1 buffer called vbo
  glBindBuffer(GL_ARRAY_BUFFER, vbo); //(Only one buffer can be bound at a time)
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW); 
  GLuint vao;
  glGenVertexArrays(1, &vao); //Create a VAO
  glBindVertexArray(vao); //Bind the above created VAO to the current context
  GLint posAttrib = glGetAttribLocation(shaderProgram, "position");
  glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 8*sizeof(float), 0);
  //(above params: Attribute, vals/attrib., type, isNormalized, stride, offset)
  glEnableVertexAttribArray(posAttrib); //Set attribute location as active

  GLint colAttrib = glGetAttribLocation(shaderProgram, "inColor");
  glVertexAttribPointer(colAttrib, 3, GL_FLOAT, GL_FALSE, 
                        8*sizeof(float), (void*)(3*sizeof(float)));
  glEnableVertexAttribArray(colAttrib);

  glBindVertexArray(0); //Unbind the VAO 
  
  glEnable(GL_DEPTH_TEST); 
 
  SDL_Event windowEvent;
  bool quit = false;
  float x = 0, y = 0, z = INIT_HEIGHT, speed = 0;
  printf("Press SPACE bar to reset the bouncing\n");
  float lastTime = SDL_GetTicks()/1000.f;
  while (!quit){
    while (SDL_PollEvent(&windowEvent)){
      if (windowEvent.type == SDL_QUIT) quit = true; //Exit Game Loop
      if (windowEvent.type == SDL_KEYUP && windowEvent.key.keysym.sym == SDLK_q) 
        quit = true; //Exit Game Loop
      if (windowEvent.type == SDL_KEYUP && windowEvent.key.keysym.sym == SDLK_ESCAPE) 
        quit = true; //Exit Game Loop
      if (windowEvent.type == SDL_KEYUP && windowEvent.key.keysym.sym == SDLK_f){ 
        fullscreen = !fullscreen; 
        SDL_SetWindowFullscreen(window, fullscreen ? SDL_WINDOW_FULLSCREEN : 0);
      }  
      if (windowEvent.type == SDL_KEYUP && windowEvent.key.keysym.sym == SDLK_SPACE){ 
		  z = INIT_HEIGHT;
		  speed = 0;
	  }
    }
    // Clear the screen to default color
    glClearColor(.2f, 0.4f, 0.8f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    

    glm::mat4 view = glm::lookAt(
      glm::vec3(5.0f, 5.0f, 3.5f),  //Cam Position
      glm::vec3(0.0f, 0.0f, 1.0f),  //Look at point
      glm::vec3(0.0f, 0.0f, 1.0f)); //Up
    GLint uniView = glGetUniformLocation(shaderProgram, "view");
    glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));
    glm::mat4 proj = glm::perspective(3.14f/4, aspect, 1.0f, 30.0f); 
                                      //FOV, aspect ratio, near, far
    GLint uniProj = glGetUniformLocation(shaderProgram, "proj");
    glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));

    glUseProgram(shaderProgram); //Set the active shader program  
    glBindVertexArray(vao);  //Bind the VAO for the shaders we are using
	{
		float time = SDL_GetTicks()/1000.f;
		printf("time - lasttime is %f\n", time-lastTime);
		glm::mat4 model;
		float tmpZ  = z + speed * (time - lastTime);
		if(tmpZ-0.5 > 0){
			speed += GRAVITY * (time - lastTime);
			z = tmpZ;
		} else {
			z = 0.5;
			speed *= -0.95;
		}
		lastTime = time;
		model = glm::translate(model,glm::vec3(x,y,z));
		//printf("speed is %f, and z is %f\n", speed, z);
		GLint uniModel = glGetUniformLocation(shaderProgram, "model");
		glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(model));
		glDrawArrays(GL_TRIANGLES, 0, 36); //Number of vertices
	}
	{
		glm::mat4 model;
		model = glm::scale(model,glm::vec3(4,4,0));
		GLint uniModel = glGetUniformLocation(shaderProgram, "model");
		glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(model));
		glDrawArrays(GL_TRIANGLES, 36, 6); //Number of vertices
    }
    
    SDL_GL_SwapWindow(window); //Double buffering
  }
  glDeleteProgram(shaderProgram);
  glDeleteShader(fragmentShader);
  glDeleteShader(vertexShader);
  glDeleteBuffers(1, &vbo);
  glDeleteVertexArrays(1, &vao);
  SDL_GL_DeleteContext(context);
  SDL_Quit();

  return 0;
}
