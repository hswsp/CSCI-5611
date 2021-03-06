#include <stdio.h>
#include <stdlib.h>
#include<iostream>
#include<fstream>
#include <vector>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <omp.h>
#if defined(__APPLE__) || defined(__linux__)
#include <SDL2/SDL.h>
//#include <SDL2/SDL_opengl.h>
#else
#include <SDL.h>
//#include <SDL_opengl.h>
#endif
using namespace std;

//#include <GL/glew.h>

#include "glad/glad.h"
#include <GLFW/glfw3.h>
GLFWwindow* window;

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "glm/gtx/norm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/simd_vec4.hpp"
using namespace glm;

#include "include/controls.hpp"
#include "include/shader.hpp"
#include "include/texture.hpp"
#include"include/objloader.hpp"

#define GRAVITY -9.81
#define PI 3.141592653
#define N  9999 //precision
#define PATH "FireworkExplore.wav"

#define FOUNTAIN 1
//#define SNOW 2
//#define FIREWORK 3


glm::vec3 waterdopColor = glm::vec3(28.f, 163.f, 236.f);
glm::vec3 splashdopColor = glm::vec3(15, 94, 156);
#define MAX_VELOCX 1
#define MIN_VELOCX 0
#define TAIL_SIZE  80
#define SPLAHNUM 6
//Window relative global
float screenWidth = 800;
float screenHeight =500;
bool mouseClicked = false;
float aspect = screenWidth / (float)screenHeight;

// CPU representation of a particle
struct Particle
{
	glm::vec3 pos, speed;
	unsigned char r,g,b,a; // Color
	float size, angle, weight;
	float life; // Remaining life of the particle. if <0 : dead and unused.
	float cameradistance; // *Squared* distance to the camera. if dead : -1.0f
	float Orientation;
	char cateroty;
	char level;

	bool operator<(const Particle& that) const 
	{
		// Sort in non-descending order : far particles drawn first.
		return this->cameradistance > that.cameradistance;
	}
};

//particle relative global
const int MaxParticles = 200000;
Particle ParticlesContainer[MaxParticles];
const int  ExploreNumMax = 1024;
glm::vec3 ExplorePos[ExploreNumMax];
char ExploreLevel[ExploreNumMax];

struct AudioData
{
	Uint8* position;
	Uint32 length;
};

void audioCallback(void* userData, Uint8* stream, int streamLength)
{
	AudioData* audio = (AudioData*)userData;

	if (audio->length == 0)
	{
		return;
	}

	Uint32 length = (Uint32)streamLength;

	length = (length > audio->length ? audio->length : length);

	SDL_memcpy(stream, audio->position, length);

	audio->position += length;
	audio->length -= length;
}

int LastUsedParticle = 0;

// Finds a Particle in ParticlesContainer which isn't used yet.
// (i.e. life < 0);
int FindUnusedParticle(){

	for(int i=LastUsedParticle; i<MaxParticles; i++){
		if (ParticlesContainer[i].life < 0){
			LastUsedParticle = i;
			return i;
		}
	}

	for(int i=0; i<LastUsedParticle; i++){
		if (ParticlesContainer[i].life < 0){
			LastUsedParticle = i;
			return i;
		}
	}

	return 0; // All particles are taken, override the first one
}

void SortParticles()
{
	std::sort(&ParticlesContainer[0], &ParticlesContainer[MaxParticles]);
}
static GLfloat* g_particule_position_size_data = new GLfloat[MaxParticles * 4 * (TAIL_SIZE + 1)];
static GLubyte* g_particule_color_data = new GLubyte[MaxParticles * 4 * (TAIL_SIZE + 1)];
//struct Particle
//{
//	glm::simdVec4 *pos;
//	glm::simdVec4 *speed;
//	unsigned char *r, *g, *b, *a; // Color
//	float *size;
//	float *angle;
//	float *weight;
//	float *cameradistance;
//	float *life;
//	Particle(int MaxParticles)
//	{
//		pos = (glm::simdVec4 *)_aligned_malloc(sizeof(glm::vec4)*MaxParticles, 16);
//		speed = (glm::simdVec4 *)_aligned_malloc(sizeof(glm::vec4)*MaxParticles, 16);
//		r = (unsigned char*)_mm_malloc(MaxParticles * sizeof(unsigned char), 16);
//		g = (unsigned char*)_mm_malloc(MaxParticles * sizeof(unsigned char), 16);
//		b = (unsigned char*)_mm_malloc(MaxParticles * sizeof(unsigned char), 16);
//		a = (unsigned char*)_mm_malloc(MaxParticles * sizeof(unsigned char), 16);
//		size = (float*)_mm_malloc(MaxParticles * sizeof(float),16);
//		angle = (float*)_mm_malloc(MaxParticles * sizeof(float), 16);
//		weight = (float*)_mm_malloc(MaxParticles * sizeof(float), 16);
//		cameradistance = (float*)_mm_malloc(MaxParticles * sizeof(float), 16);
//		life = (float*)_mm_malloc(MaxParticles * sizeof(float), 16);
//	}
//	Particle()
//	{
//		_aligned_free(pos);
//		_aligned_free(speed);
//		_mm_free(r);
//		_mm_free(g);
//		_mm_free(b);
//		_mm_free(a);
//		_mm_free(size);
//		_mm_free(angle);
//		_mm_free(weight);
//		_mm_free(life);
//		_mm_free(cameradistance);
//	}
//
//};
//Particle *ParticlesContainer = new Particle(MaxParticles);
//int FindUnusedParticle() {
//
//	for (int i = LastUsedParticle; i < MaxParticles; i++) {
//		if (ParticlesContainer->life[i] < 0) {
//			LastUsedParticle = i;
//			return i;
//		}
//	}
//
//	for (int i = 0; i < LastUsedParticle; i++) {
//		if (ParticlesContainer->life[i] < 0) {
//			LastUsedParticle = i;
//			return i;
//		}
//	}
//
//	return 0; // All particles are taken, override the first one
//}
//bool compare(float a, float b)
//{
//	return a > b;
//}
//
//glm::simdVec4  * __restrict pos = ParticlesContainer->pos;
//glm::simdVec4  * __restrict speed = ParticlesContainer->speed;
//float * __restrict psize = ParticlesContainer->size;
//float * __restrict angle = ParticlesContainer->angle;
//float * __restrict weight = ParticlesContainer->weight;
//float * __restrict cameradistance = ParticlesContainer->cameradistance;
//float * __restrict plife = ParticlesContainer->life;
//unsigned char * __restrict pr = ParticlesContainer->r;
//unsigned char * __restrict pg = ParticlesContainer->g;
//unsigned char * __restrict pb = ParticlesContainer->b;
//unsigned char * __restrict pa = ParticlesContainer->a;
//
//void SortParticles()
//{
//	std::sort(ParticlesContainer->cameradistance[0], ParticlesContainer->cameradistance[MaxParticles], compare);
//}

// http://www.rgba.org/articles/sfrand/sfrand.htm
static unsigned int mirand = 1;
float sfrand(void) {
	unsigned int a;
	mirand *= 16807;
	a = (mirand & 0x007fffff) | 0x40000000;
	return(*((float*)&a) - 3.0f);
}
// floor vettex and norm
GLfloat cubevertices[] = {
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

	 -0.5f, -0.5f, -0.5f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, //Black face
	  0.5f, -0.5f, -0.5f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f,
	  0.5f, -0.5f,  0.5f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f,
	  0.5f, -0.5f,  0.5f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f,
	 -0.5f, -0.5f,  0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f,
	 -0.5f, -0.5f, -0.5f, 0.5f, 0.0f, 0.5f, 0.0f, 1.0f,

	 -0.5f,  0.5f, -0.5f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, //White face
	  0.5f,  0.5f, -0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
	  0.5f,  0.5f,  0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
	  0.5f,  0.5f,  0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
	 -0.5f,  0.5f,  0.5f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f,
	 -0.5f,  0.5f, -0.5f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f,

	 -0.5f, -0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 0.0f, 0.0f, //Red face
	  0.5f, -0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 1.0f, 0.0f,
	  0.5f,  0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 1.0f, 1.0f,
	  0.5f,  0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 1.0f, 1.0f,
	 -0.5f,  0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 0.0f, 1.0f,
	 -0.5f, -0.5f,  0.0f, 0.7f, 0.0f, 0.0f, 0.0f, 0.0f,
};
float cubenormals[] = 
{
	0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,
	0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,
	-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,
	1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,
	0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,0.f,-1.f,0.f,
	0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,0.f,1.f,0.f,
};
float cubeUV[] =
{
	0.748573, 0.750412,
	0.749279, 0.501284,
 0.999110, 0.501077,
 0.999455, 0.750380,
 0.250471, 0.500702,
 0.249682, 0.749677,
 0.001085, 0.750380,
 0.001517, 0.499994,
 0.499422, 0.500239,
 0.500149 ,0.750166,
 0.748355, 0.998230,
 0.500193, 0.998728,
 0.498993, 0.250415,
 0.748953, 0.250920,
};
// The VBO containing the 4 vertices of the particles.
	// Thanks to instancing, they will be shared by all particles.
static const GLfloat g_vertex_buffer_data[] = 
{
	 -0.5f, -0.5f, 0.0f,
	  0.5f, -0.5f, 0.0f,
	 -0.5f,  0.5f, 0.0f,
	  0.5f,  0.5f, 0.0f,
};
GLfloat backgroundUV[] =
{
	  0.0f, 0.0f,
	  1.0f, 0.0f,
	  0.0f,  1.0f,
	  1.0f,  1.0f, 
};
void CalculateFrameRate();
//Index of where to model, view, and projection matricies are stored on the GPU
GLint uniModel, uniView, uniProj;
GLint posAttrib,colAttrib, normAttrib, vertexUVAttrib;
//GLint SphereposAttrib, SphereuvAttrib, SpherenormAttrib;
GLint TextureIDAttrib;//-1 for no texture
GLuint FloorTextureID, FloorTexture;
GLuint sphereTextureID, sphereTexture;

vector<glm::vec3> spherevertices;
vector<glm::vec2> sphereuvs;
vector<glm::vec3> spherenormals; // Won't be used at the moment.
GLuint vertexbuffer;
GLuint uvbuffer;

vector<glm::vec3> fountainvertices;
vector<glm::vec2> fountainuvs;
vector<glm::vec3> fountainnormals; // Won't be used at the moment.
GLint fountainTexture, fountainTextureID;
int numVertsTeapot;

GLint BackgroundTexture;
GLuint InitialFloor(GLuint *vbo, GLuint *vao);
void DrawFloor(GLuint programID, GLuint *vao, glm::mat4 view, glm::mat4 proj);

//paticals utility definition
const float emitterR = 1.0f;
glm::vec3 emitPos(void);
glm::vec3 emitDir(void);
void DrawParticles(GLuint particles_position_buffer, GLuint particles_color_buffer, GLuint billboard_vertex_buffer,
	GLfloat*g_particule_position_size_data, GLubyte* g_particule_color_data, int ParticlesCount,
	GLuint programID, GLuint VertexArrayID, GLuint TextureID, GLuint Texture, glm::mat4 ProjectionMatrix, glm::mat4 ViewMatrix,
	GLuint CameraRight_worldspace_ID, GLuint CameraUp_worldspace_ID, GLuint ViewProjMatrixID);


//floor relative global
float basescale = 10.0f;
glm::vec3 Origin = glm::vec3(-5.0f,0,-20.f);//emitter position -10.0f, -20.0f
glm::vec3 emitter_pos = Origin + glm::vec3(basescale*0.4035, basescale *0.32 ,2.0f);
glm::vec3 CameraPosition;
extern glm::vec3 Cameradirection;
extern bool CatchTeapot = false;

//physics normal:
bool computephysics(Particle& p, double delta);

//void computephysics(int Index,double delta);
glm::vec3 FloorN = glm::vec3(0, 1, 0);
float FloorCor = 0.95;
float spherecor = 0.7;
glm::vec3 sphereC = Origin + glm::vec3(18.0f, 2.0f, 2.0f);
float sphereR = 3.0f;



int main(int argc, char *argv[]){
	// Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		getchar();
		return -1;
	}
	if (SDL_Init(SDL_INIT_AUDIO) < 0)
		return 1;
	
	SDL_AudioSpec wavSpec;
	Uint8* wavStart;
	Uint32 wavLength;
	/*if(SDL_LoadWAV(PATH, &wavSpec, &wavStart, &wavLength) == NULL)
	{
		cerr << "Error: file could not be loaded as an audio file." << endl;
	}
	{
		SDL_AudioSpec wavSpec;
		Uint8* wavStart;
		Uint32 wavLength;
		if(SDL_LoadWAV("Ennio Morricone - Lolita.wav", &wavSpec, &wavStart, &wavLength) == NULL)
		{
			cerr << "Error: file could not be loaded as an audio file." << endl;
		}
		AudioData audio;
		audio.position = wavStart;
		audio.length = wavLength;

		wavSpec.callback = audioCallback;
		wavSpec.userdata = &audio;

		SDL_AudioDeviceID audioDevice;
		audioDevice = SDL_OpenAudioDevice(NULL, 0, &wavSpec, NULL, SDL_AUDIO_ALLOW_ANY_CHANGE);

		if (audioDevice == 0)
		{
			cerr << "Error: " << SDL_GetError() << endl;
			return -1;
		}

		SDL_PauseAudioDevice(audioDevice, 0);
	}*/
	

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	window = glfwCreateWindow(mode->width/1.5, mode->height/1.2, "Particules", NULL, NULL);//screenWidth screenHeight
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	srand((unsigned)time(NULL));

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// Set the mouse at the center of the screen
	glfwPollEvents();
	glfwSetCursorPos(window, screenWidth / 2, screenHeight / 2);


	// Dark blue background
//	glClearColor(0.298f, 0.796f, 1.0f, 0.6f);
	glClearColor(0.25f, 0.25f, 0.25f, 0.8f);
	//glClearColor(1.f, 1.f, 1.0f, 0.8f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);
	// Cull triangles which normal is not towards the camera
	//glEnable(GL_CULL_FACE);

	//Initial floor 
	GLuint FloorVAO[10];
	GLuint FloorVBO[10];
	GLint shaderProgram = InitialFloor(FloorVBO, FloorVAO);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);
	glBindVertexArray(0);

	// Create and compile our GLSL program from the shaders

	GLuint programID = LoadShaders("Particle.vertexshader", "Particle.fragmentshader");

	// Vertex shader
	GLuint CameraRight_worldspace_ID = glGetUniformLocation(programID, "CameraRight_worldspace");
	GLuint CameraUp_worldspace_ID = glGetUniformLocation(programID, "CameraUp_worldspace");
	GLuint ViewProjMatrixID = glGetUniformLocation(programID, "VP");

	// fragment shader
	GLuint TextureID = glGetUniformLocation(programID, "myTextureSampler");


	for (int i = 0; i < MaxParticles; i++)
	{
		ParticlesContainer[i].life = -1.0f;
		ParticlesContainer[i].cameradistance = -1.0f;
	}

#ifdef SNOW
	GLuint Texture = loadBMP_custom("SnowFlake.bmp");
#else
	GLuint Texture = loadDDS("particle.DDS");//loadBMP_custom("texture/Droplet.bmp")
#endif


	glBindVertexArray(VertexArrayID);
	GLuint billboard_vertex_buffer;
	glGenBuffers(1, &billboard_vertex_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, billboard_vertex_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

	// The VBO containing the positions and sizes of the particles
	GLuint particles_position_buffer;
	glGenBuffers(1, &particles_position_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
	// Initialize with empty (NULL) buffer : it will be updated later, each frame.
	glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLfloat), NULL, GL_STREAM_DRAW);

	// The VBO containing the colors of the particles
	GLuint particles_color_buffer;
	glGenBuffers(1, &particles_color_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
	// Initialize with empty (NULL) buffer : it will be updated later, each frame.
	glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLubyte), NULL, GL_STREAM_DRAW);

	double lastTime = glfwGetTime();

	glm::vec3 Originmaindir = glm::vec3(0.0f, 10.0f, 0.0f);
	glm::mat4 rotationMat(1);
	rotationMat = glm::rotate(rotationMat,-(float)PI*0.139f, glm::vec3(0.0, 0.0, 1.0));
	Originmaindir = glm::vec3(rotationMat * glm::vec4(Originmaindir, 1.0));

	 
	do
	{
		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		double currentTime = glfwGetTime();
		double delta = currentTime - lastTime;
		lastTime = currentTime;


		computeMatricesFromInputs();
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix = getViewMatrix();

		// We will need the camera's position in order to sort the particles
		// w.r.t the camera's distance.
		// There should be a getCameraPosition() function in common/controls.cpp, 
		// but this works too.
		CameraPosition = glm::vec3(glm::inverse(ViewMatrix)[3]);
		glm::vec3 Cameraright = glm::vec3(glm::inverse(ViewMatrix)[0]);
		glm::mat4 ViewProjectionMatrix = ProjectionMatrix * ViewMatrix;


		// Generate 10 new particule each millisecond,
		// but limit this to 16 ms (60 fps), or if you have 1 long frame (1sec),
		// newparticles will be huge and the next frame even longer.
		int newparticles;
#ifdef FOUNTAIN
		newparticles = (int)(delta*20000.0);
		if (newparticles > (int)(0.029f*20000.0))//fix to 35 FPS
			newparticles = (int)(0.029f*20000.0);
		
#pragma omp parallel for num_threads(3)
		for (int i = 0; i < newparticles; i++)
		{
			int particleIndex = FindUnusedParticle();
			{
				ParticlesContainer[particleIndex].life = 5.0f; // This particle will live 5 seconds.
				glm::vec3 maindir;
				if (!CatchTeapot)
				{
					maindir = Originmaindir;
					ParticlesContainer[particleIndex].pos = emitter_pos;//+ emitPos()
				}
				else
				{
					glm::vec3 Catchmaindir = glm::vec3(0.0f, 10.0f, 0.0f);
					rotationMat = mat4(1);
					rotationMat = glm::rotate(rotationMat, -(float)PI*0.139f, glm::normalize(Cameraright));
					Catchmaindir = glm::vec3(rotationMat * glm::vec4(Catchmaindir, 1.0));
					maindir = Catchmaindir;
					
					ParticlesContainer[particleIndex].pos=
						CameraPosition + Cameradirection * 5.0f + glm::vec3(0.0f, -CameraPosition.y*0.4, 0.0f);
				}
				float spread = 1.5f;
				
				// Very bad way to generate a random direction; 
				// See for instance http://stackoverflow.com/questions/5408276/python-uniform-spherical-distribution instead,
				// combined with some user-controlled parameters (main direction, spread, etc)
				glm::vec3 randomdir = glm::vec3(
						(rand() % 2000 - 1000.0f) / 1000.0f,
						(rand() % 2000 - 1000.0f) / 1000.0f,
						(rand() % 2000 - 1000.0f) / 1000.0f
						);
				//glm::vec3 randomdir = emitDir();
				ParticlesContainer[particleIndex].speed = maindir + randomdir * spread;
				// Very bad way to generate a random color
				ParticlesContainer[particleIndex].r = waterdopColor.r;//rand() % 256
				ParticlesContainer[particleIndex].g = waterdopColor.g;//rand() % 256
				ParticlesContainer[particleIndex].b = waterdopColor.b;//rand() % 256
				ParticlesContainer[particleIndex].a = (rand() % 256) / 3;// 65
				ParticlesContainer[particleIndex].size = (rand() % 1000) / 2500.0f + 0.05f;//rand()
				ParticlesContainer[particleIndex].level = 1;
			}
		}
#endif

#ifdef SNOW 
		 newparticles = (int)(delta*3000.0);
		if (newparticles > (int)(0.016f*3000.0))
			newparticles = (int)(0.016f*3000.0);
		for (int i = 0; i < newparticles; i++){
			glm::vec3 startPos = glm::vec3(
				(rand() % 2000 - 1000.0f) / 100.0f,//(MAX_VELOC - MIN_VELOC)*(float(rand()) / float(RAND_MAX)) + MIN_VELOC,
						13.0f,
				(rand() % 2000 - 1000.0f) / 100.0f//(MAX_VELOC - MIN_VELOC)*(float(rand()) / float(RAND_MAX)) + MIN_VELOC
						);

			int particleIndex = FindUnusedParticle();
			ParticlesContainer[particleIndex].life = 5.0f; // This particle will live 5 seconds.
			ParticlesContainer[particleIndex].pos = startPos;//+ emitPos()
			float spread = 1.5f;
			glm::vec3 maindir = glm::vec3((MAX_VELOCX - MIN_VELOCX)*(float(rand()) / float(RAND_MAX)) + MIN_VELOCX , 0.0f, 0.0f);
			// Very bad way to generate a random direction; 
			// See for instance http://stackoverflow.com/questions/5408276/python-uniform-spherical-distribution instead,
			// combined with some user-controlled parameters (main direction, spread, etc)
			glm::vec3 randomdir = glm::vec3
			(       0,//(MAX_VELOC - MIN_VELOC)*(float(rand()) / float(RAND_MAX)) + MIN_VELOC,
					0,
				    0//(MAX_VELOC - MIN_VELOC)*(float(rand()) / float(RAND_MAX)) + MIN_VELOC
					);
			//glm::vec3 randomdir = emitDir();
			ParticlesContainer[particleIndex].speed = maindir + randomdir * spread;
			// Very bad way to generate a random color
			ParticlesContainer[particleIndex].r = 255;
			ParticlesContainer[particleIndex].g = 255;
			ParticlesContainer[particleIndex].b = 255;
			ParticlesContainer[particleIndex].a = 65;
			ParticlesContainer[particleIndex].size = (rand() % 1000) / 5000.0f+0.01f;
		}
#endif

#ifdef FIREWORK
		{
			static int count = 0;
			//if(count % 160 == 0){
			if(mouseClicked){
				mouseClicked = false;
				{
					SDL_AudioSpec wavSpec;
					Uint8* wavStart;
					Uint32 wavLength;
					if(SDL_LoadWAV("FireWorkUp.wav", &wavSpec, &wavStart, &wavLength) == NULL)
					{
						cerr << "Error: file could not be loaded as an audio file." << endl;
					}
					AudioData audio;
					audio.position = wavStart;
					audio.length = wavLength;

					wavSpec.callback = audioCallback;
					wavSpec.userdata = &audio;

					static SDL_AudioDeviceID audioDevice;
					SDL_CloseAudioDevice(audioDevice);
					audioDevice = SDL_OpenAudioDevice(NULL, 0, &wavSpec, NULL, SDL_AUDIO_ALLOW_ANY_CHANGE);

					if (audioDevice == 0)
					{
						cerr << "Error: " << SDL_GetError() << endl;
						return 1;
					}

					SDL_PauseAudioDevice(audioDevice, 0);
				}

				glm::vec3 startPos = glm::vec3(
						(rand() % 2000 - 1000.0f) / 100.0f,
						-10.0f,
						(rand() % 2000 - 1000.0f) / 100.0f
						);
				int firework = FindUnusedParticle();
				ParticlesContainer[firework].life = 4.0f; // This particle will live 5 seconds.
				ParticlesContainer[firework].pos = startPos;//+ emitPos()
				float spread = 1.5f;
				glm::vec3 randomdir = glm::vec3(
						(rand() % 2000 - 1000.0f) / 500.0f,
						(rand() % 2000 - 1000.0f) / 1200.0f + 12,
						(rand() % 2000 - 1000.0f) / 500.0f
						);
				ParticlesContainer[firework].speed = randomdir * spread;
				ParticlesContainer[firework].r = 226;
				ParticlesContainer[firework].g = 17;
				ParticlesContainer[firework].b = 12;
				ParticlesContainer[firework].a = (rand() % 256) / 1;
				ParticlesContainer[firework].size = (rand() % 1000) / 2000.0f + 0.1f;
				ParticlesContainer[firework].level = 2;
			}
			count++;
		}

		for(int i = 0; i < ExploreNumMax; i++){
			if(ExplorePos[i].y > 0){
				int num = 200;
				if(ExploreLevel[i] == 1) num = 50;
				else {
					static int count = 0;
					count++;
					{
						
						AudioData audio;
						audio.position = wavStart;
						audio.length = wavLength;

						wavSpec.callback = audioCallback;
						wavSpec.userdata = &audio;

						static SDL_AudioDeviceID audioDevice;
						SDL_CloseAudioDevice(audioDevice);
						audioDevice = SDL_OpenAudioDevice(NULL, 0, &wavSpec, NULL, SDL_AUDIO_ALLOW_ANY_CHANGE);

						if (audioDevice == 0)
						{
							cerr << "Error: " << SDL_GetError() << endl;
							return 1;
						}

						SDL_PauseAudioDevice(audioDevice, 0);
					}
				}
				for (int j = 0; j < num; j++){
					int particleIndex = FindUnusedParticle();
					ParticlesContainer[particleIndex].pos = ExplorePos[i];//+ emitPos()
					float spread = 1.5f;
					glm::vec3 maindir = glm::vec3(0.0f, 5.0f, 0.0f);
					glm::vec3 randomdir = glm::vec3(
							(rand() % 2000 - 1000.0f) / 500.0f,
							(rand() % 2000 - 1000.0f) / 500.0f,
							(rand() % 2000 - 1000.0f) / 500.0f
							);
					
					ParticlesContainer[particleIndex].a = (rand() % 256) / 3;

					if(ExploreLevel[i] == 2){
						ParticlesContainer[particleIndex].r = rand() % 256;
						ParticlesContainer[particleIndex].g = rand() % 256;
						ParticlesContainer[particleIndex].b = rand() % 256;
						ParticlesContainer[particleIndex].size = (rand() % 1000) / 3000.0f + 0.1f;
						ParticlesContainer[particleIndex].life = 2.5f; // This particle will live 5 seconds.
						ParticlesContainer[particleIndex].speed = maindir + randomdir * spread;
					} else {
						static int count = 0;
						if(count%2 ==  0){
							ParticlesContainer[particleIndex].r = 255;
							ParticlesContainer[particleIndex].g = 215;
							ParticlesContainer[particleIndex].b = 0;
						}else {
							ParticlesContainer[particleIndex].r = 192;
							ParticlesContainer[particleIndex].g = 192;
							ParticlesContainer[particleIndex].b = 192;
						}
						count++;

						ParticlesContainer[particleIndex].size = (rand() % 1000) / 50000.0f + 0.1f;
						ParticlesContainer[particleIndex].life = 1.f; // This particle will live 5 seconds.
						ParticlesContainer[particleIndex].speed = randomdir * 0.3f;
					}
					ParticlesContainer[particleIndex].level = ExploreLevel[i]-1;
				}
				ExplorePos[i].y = 0;
			}
		}
#endif

		

		// Simulate all particles
		int ParticlesCount = 0;
//		omp_set_num_threads(1);
#pragma omp parallel for num_threads(3)
		for (int i = 0; i < MaxParticles; i++)
		{

			Particle& p = ParticlesContainer[i]; // shortcut

			if (p.life > 0.0f) {

				// Decrease life
				p.life -= delta;
				if (p.life > 0.0f) {

					bool Needsplash = computephysics( p, delta);
					p.cameradistance = glm::length2(p.pos - CameraPosition);
					//cameradistance[i] =(float)glm::length4(pos[i] - glm::simdVec4(CameraPosition,0.0));//
					//ParticlesContainer[i].pos += glm::vec3(0.0f,10.0f, 0.0f) * (float)delta;
					if (Needsplash)
					{
						//ParticlesCount++;
						int newparticles = (SPLAHNUM - 2)*(float(rand()) / float(RAND_MAX)) + 2;
						//int newparticles = 1;
#pragma omp parallel for num_threads(3)
						for (int i = 0; i < newparticles; i++)
						{

							int particleIndex = FindUnusedParticle();
							{
								ParticlesContainer[particleIndex].life = 3.0f; // This particle will live 5 seconds.
								ParticlesContainer[particleIndex].pos = p.pos;//+ emitPos()
								float spread = 1.5f;
								glm::vec3 maindir = p.speed;
								glm::vec3 randomdir = glm::vec3(
									(float(rand()) / float(RAND_MAX)) -0.5,
									0,
									(float(rand()) / float(RAND_MAX)) -0.5
								);
								//glm::vec3 randomdir = emitDir();
								ParticlesContainer[particleIndex].speed = maindir + randomdir * spread;
								// Very bad way to generate a random color
								ParticlesContainer[particleIndex].r = splashdopColor.r;//rand() % 256
								ParticlesContainer[particleIndex].g = splashdopColor.g;//rand() % 256
								ParticlesContainer[particleIndex].b = splashdopColor.b;//rand() % 256
								ParticlesContainer[particleIndex].a = p.a;
								float size = p.size / newparticles;
								ParticlesContainer[particleIndex].size = size;
								ParticlesContainer[particleIndex].level = 2;
								//g_particule_position_size_data[4 * ParticlesCount + 0] = p.pos.x;
								//g_particule_position_size_data[4 * ParticlesCount + 1] = p.pos.y;
								//g_particule_position_size_data[4 * ParticlesCount + 2] = p.pos.z;
								//g_particule_position_size_data[4 * ParticlesCount + 3] = size;
								//g_particule_color_data[4 * ParticlesCount + 0] = splashdopColor.r;
								//g_particule_color_data[4 * ParticlesCount + 1] = splashdopColor.g;
								//g_particule_color_data[4 * ParticlesCount + 2] = splashdopColor.b;
								//g_particule_color_data[4 * ParticlesCount + 3] = p.a;
								//ParticlesCount++;
							}
						}
					}
					else
					//	 Fill the GPU buffer
					{
						g_particule_position_size_data[4 * ParticlesCount + 0] = p.pos.x;
						g_particule_position_size_data[4 * ParticlesCount + 1] = p.pos.y;
						g_particule_position_size_data[4 * ParticlesCount + 2] = p.pos.z;
						g_particule_position_size_data[4 * ParticlesCount + 3] = p.size;
						g_particule_color_data[4 * ParticlesCount + 0] = p.r;
						g_particule_color_data[4 * ParticlesCount + 1] = p.g;
						g_particule_color_data[4 * ParticlesCount + 2] = p.b;
						g_particule_color_data[4 * ParticlesCount + 3] = p.a;
					}

#ifdef FIREWORK
					if(p.level > 0)
					{
					
						for(int tailCnt = 1; tailCnt < TAIL_SIZE; tailCnt++){
							ParticlesCount++;
							g_particule_position_size_data[4 * ParticlesCount + 0] = p.pos.x - p.speed.x * delta * tailCnt/4;
							g_particule_position_size_data[4 * ParticlesCount + 1] = p.pos.y - p.speed.y * delta * tailCnt/4;
							g_particule_position_size_data[4 * ParticlesCount + 2] = p.pos.z - p.speed.z * delta * tailCnt/4;
							g_particule_position_size_data[4 * ParticlesCount + 3] = p.size* (TAIL_SIZE - tailCnt)/ TAIL_SIZE;
							g_particule_color_data[4 * ParticlesCount + 0] = p.r;
							g_particule_color_data[4 * ParticlesCount + 1] = p.g;
							g_particule_color_data[4 * ParticlesCount + 2] = p.b;
							g_particule_color_data[4 * ParticlesCount + 3] = p.a * p.life * (TAIL_SIZE -tailCnt)/TAIL_SIZE;
						}
					}
#endif

				}
				else {
					// Particles that just died will be put at the end of the buffer in SortParticles();
					p.cameradistance = -1.0f;
#ifdef FIREWORK
					if(p.level > 0){
						int pos = -1;
						for(int i = 0; i < ExploreNumMax; i++){
							if(ExplorePos[i].y <= 0){
								pos = i;
								break;
							}
						}
						if(pos != -1){
							ExplorePos[pos] = p.pos;
							ExploreLevel[pos] = p.level;
						}
						p.level = 0;
					}
#endif
				}

				ParticlesCount++;

			}
		}

		SortParticles();

		// draw floor
		DrawFloor(shaderProgram, FloorVAO, ViewMatrix, ProjectionMatrix);

		DrawParticles(particles_position_buffer, particles_color_buffer, billboard_vertex_buffer,
			g_particule_position_size_data, g_particule_color_data, ParticlesCount,
			programID, VertexArrayID, TextureID, Texture, ProjectionMatrix, ViewMatrix,
			CameraRight_worldspace_ID, CameraUp_worldspace_ID, ViewProjMatrixID);

		CalculateFrameRate();
		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

	} // Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0);


	delete[] g_particule_position_size_data;

	// Cleanup VBO and shader
	glDeleteBuffers(1, &particles_color_buffer);
	glDeleteBuffers(1, &particles_position_buffer);
	glDeleteBuffers(1, &billboard_vertex_buffer);
	glDeleteProgram(programID);
	glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);
	glDeleteProgram(shaderProgram);



//	SDL_FreeWAV(wavStart);
	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}


GLuint InitialFloor(GLuint *vbo,GLuint *vao)
{
	GLuint shaderProgram = LoadShaders("shader.vs", "shader.fs");//GLuint shaderProgram 
	
	glUseProgram(shaderProgram);
	glBindFragDataLocation(shaderProgram, 0, "outColor"); // set output

	//load the texture
	FloorTexture = loadBMP_custom("texture/drop.bmp");
	FloorTextureID = glGetUniformLocation(shaderProgram, "myTextureSampler");
	glEnableVertexAttribArray(posAttrib); //Set attribute location as active
	glEnableVertexAttribArray(colAttrib);
	glEnableVertexAttribArray(normAttrib);
	glEnableVertexAttribArray(vertexUVAttrib);
	//background 
	BackgroundTexture = loadBMP_custom("texture/city.bmp");
	glGenVertexArrays(1, &vao[3]); //Create a VAO
	glBindVertexArray(vao[3]); //Bind the above created VAO to the current context

	glGenBuffers(1, &vbo[7]);  //Create 1 buffer called vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo[7]); //(Only one buffer can be bound at a time)
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);
	posAttrib = glGetAttribLocation(shaderProgram, "position");
	glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), 0);
	
	glGenBuffers(1, &vbo[8]);  //Create 1 buffer called vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo[8]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(backgroundUV), backgroundUV, GL_STATIC_DRAW); //upload normals to vbo
	vertexUVAttrib = glGetAttribLocation(shaderProgram, "vertexUV");
	glVertexAttribPointer(vertexUVAttrib, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);//unbind

	//load floor
	glGenVertexArrays(1, &vao[0]); //Create a VAO
	glBindVertexArray(vao[0]); //Bind the above created VAO to the current context

	glGenBuffers(2, vbo);  //Create 1 buffer called vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]); //(Only one buffer can be bound at a time)
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubevertices), cubevertices, GL_STATIC_DRAW);
	
	

	posAttrib = glGetAttribLocation(shaderProgram, "position");
	glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), 0);
	//(above params: Attribute, vals/attrib., type, isNormalized, stride, offset)
	colAttrib = glGetAttribLocation(shaderProgram, "inColor");
	glVertexAttribPointer(colAttrib, 3, GL_FLOAT, GL_FALSE,8 * sizeof(float), (void*)(3 * sizeof(float)));
	
	//make sure One VAO->one VBO!
	glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubenormals), cubenormals, GL_STATIC_DRAW); //upload normals to vbo
	normAttrib = glGetAttribLocation(shaderProgram, "inNormal");
	glVertexAttribPointer(normAttrib, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbo[5]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubeUV), cubeUV, GL_STATIC_DRAW); //upload normals to vbo
	vertexUVAttrib = glGetAttribLocation(shaderProgram, "vertexUV");
	glVertexAttribPointer(vertexUVAttrib, 2, GL_FLOAT, GL_FALSE, 0, 0);

	
	glBindVertexArray(0);//unbind

#ifdef FOUNTAIN
	//Load sphere
	glGenVertexArrays(1, &vao[1]);
	glBindVertexArray(vao[1]);
	// Load the texture
	
	bool res = loadOBJ("models/sphere.obj", spherevertices, sphereuvs, spherenormals);

	// Load it into a VBO
	glGenBuffers(1, &vbo[2]);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
	glBufferData(GL_ARRAY_BUFFER, spherevertices.size() * sizeof(glm::vec3), &spherevertices[0], GL_STATIC_DRAW);

	
	glGenBuffers(1, &vbo[3]);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
	glBufferData(GL_ARRAY_BUFFER, sphereuvs.size() * sizeof(glm::vec2), &sphereuvs[0], GL_STATIC_DRAW);

	
	glGenBuffers(1, &vbo[4]);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[4]);
	glBufferData(GL_ARRAY_BUFFER, spherenormals.size() * sizeof(glm::vec3), &spherenormals[0], GL_STATIC_DRAW);

	posAttrib = glGetAttribLocation(shaderProgram, "position");//vertexPosition_modelspace
	glEnableVertexAttribArray(posAttrib);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
	glVertexAttribPointer(
		posAttrib,                  // attribute
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);

	// 2nd attribute buffer : UVs
	vertexUVAttrib = glGetAttribLocation(shaderProgram, "vertexUV");
	glEnableVertexAttribArray(vertexUVAttrib);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
	glVertexAttribPointer(
		vertexUVAttrib,                                // attribute
		2,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);
	normAttrib = glGetAttribLocation(shaderProgram, "inNormal"); //1;
	glEnableVertexAttribArray(normAttrib);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[4]);
	glVertexAttribPointer(
		normAttrib,                                // attribute
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);
	glBindVertexArray(0);
#endif // FOUNTAIN
	// draw basin
	//Load Model 1
	ifstream Teapot;
	int numLines = 0;
	fountainTexture = loadBMP_custom("texture/3.bmp");
	fountainTextureID = glGetUniformLocation(shaderProgram, "myTextureSampler");
	Teapot.open("models/teapot.txt");
	Teapot >> numLines;
	float* model1 = new float[numLines];
	for (int i = 0; i < numLines; i++) {
		Teapot >> model1[i];
	}
	numVertsTeapot = numLines / 8;
	Teapot.close();
	int startVertTeapot = 0;  //The teapot is the first model in the VBO
	glGenVertexArrays(1, &vao[2]); //Create a VAO
	glBindVertexArray(vao[2]); //Bind the above created VAO to the current context

	glGenBuffers(1, &vbo[6]);  //Create 1 buffer called vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo[6]); //(Only one buffer can be bound at a time)
	glBufferData(GL_ARRAY_BUFFER, numVertsTeapot * 8 * sizeof(float), model1, GL_STATIC_DRAW);

	glEnableVertexAttribArray(posAttrib); //Set attribute location as active
	posAttrib = glGetAttribLocation(shaderProgram, "position");
	glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), 0);

	glEnableVertexAttribArray(normAttrib);
	normAttrib = glGetAttribLocation(shaderProgram, "inNormal");
	glVertexAttribPointer(normAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(5 * sizeof(float)));

	glEnableVertexAttribArray(vertexUVAttrib);
	vertexUVAttrib = glGetAttribLocation(shaderProgram, "vertexUV");
	glVertexAttribPointer(vertexUVAttrib, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glBindVertexArray(0);//unbind

	
	uniView = glGetUniformLocation(shaderProgram, "view");
	uniProj = glGetUniformLocation(shaderProgram, "proj");
	uniModel = glGetUniformLocation(shaderProgram, "model");

	return shaderProgram;
}

void DrawFloor(GLuint programID, GLuint *vao, glm::mat4 view, glm::mat4 proj)//
{
	

	glUseProgram(programID); //Set the active shader program 
	glEnable(GL_BLEND);
	//glBlendEquation(GL_FUNC_ADD);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));

	
	//floor
	glEnable(GL_DEPTH_TEST);
	glBindVertexArray(vao[0]);  //Bind the VAO for the shaders we are using
	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, FloorTexture);
	
	glUniform1i(FloorTextureID, 0);
	glUniform1i(glGetUniformLocation(programID, "texID") , -1);// -1 means using color
	glEnableVertexAttribArray(posAttrib); //Set attribute location as active
	glEnableVertexAttribArray(colAttrib);
	glEnableVertexAttribArray(normAttrib);
	glEnableVertexAttribArray(vertexUVAttrib);

	glm::mat4 model;
	model = glm::translate(model, Origin ); //gglm::vec3(0, -20.0f, -20.0f)
	model = glm::scale(model, glm::vec3(50, 0, 50));
	glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(model));
	glDrawArrays(GL_TRIANGLES, 24, 6); //(Primitives, Which VBO, Number of vertices)
	glm::mat4 ModelMatrix;
#ifdef FOUNTAIN
	// draw sphere 
	glBindVertexArray(vao[1]);
	ModelMatrix = glm::mat4(1.0);
	ModelMatrix = glm::translate(ModelMatrix, sphereC); //
	ModelMatrix = glm::scale(ModelMatrix, glm::vec3(1,1,1));
	glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));
	glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(ModelMatrix));
	//glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, FloorTexture);
	glUniform1i(glGetUniformLocation(programID, "texID"), 1);// -1 means using color
	// Set our "myTextureSampler" sampler to use Texture Unit 0
	glUniform1i(FloorTextureID, 0);

	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(posAttrib);
	glEnableVertexAttribArray(vertexUVAttrib);
	glEnableVertexAttribArray(normAttrib);
	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, spherevertices.size());
#endif 

	//draw fountain
	glBindVertexArray(vao[2]);
	ModelMatrix = glm::mat4(1.0);
	if (CatchTeapot)
	{
		ModelMatrix = glm::translate(ModelMatrix, CameraPosition + Cameradirection*5.0f
			+ glm::vec3(0.0f, -CameraPosition.y*0.4, 0.0f));
		ModelMatrix = glm::scale(ModelMatrix, glm::vec3(basescale*0.3, basescale*0.3, basescale*0.3));
		ModelMatrix = glm::rotate(ModelMatrix, -(float)PI / 2, glm::vec3(1, 0, 0));
		float angle = angle = acos(dot(glm::vec3(1, 0, 0), Cameradirection));
		ModelMatrix = glm::rotate(ModelMatrix, angle, glm::vec3(0, 0, 1));//(float)PI / 2
	}
	else if (glm::length(CameraPosition - (Origin - glm::vec3(0, basescale / 5, 0))) < 10.0f)
	{
		CatchTeapot = true;
	}
	else
	{
		ModelMatrix = glm::translate(ModelMatrix, Origin + glm::vec3(0, basescale / 5, 0)); //
		ModelMatrix = glm::scale(ModelMatrix, glm::vec3(basescale, basescale, basescale));
		ModelMatrix = glm::rotate(ModelMatrix, -(float)PI / 2, glm::vec3(1, 0, 0));
	}
	glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));
	glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(ModelMatrix));
	//glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, fountainTexture);
	glUniform1i(glGetUniformLocation(programID, "texID"), 2);// -1 means using color
	// Set our "myTextureSampler" sampler to use Texture Unit 0
	glUniform1i(fountainTextureID, 0);

	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(posAttrib);
	glEnableVertexAttribArray(vertexUVAttrib);
	glEnableVertexAttribArray(normAttrib);
	// Draw the teapot !
	glDrawArrays(GL_TRIANGLES, 0, numVertsTeapot);

	glDisableVertexAttribArray(posAttrib);
	glDisableVertexAttribArray(vertexUVAttrib);
	glDisableVertexAttribArray(normAttrib);
	glDisableVertexAttribArray(colAttrib);

	
}


glm::vec3 emitPos(void)
{
	// emit form a circle
	
	float x1= 2 * (rand() % (N + 1) / (float)(N + 1)) - 1;
	float x2 = 2 * (rand() % (N + 1) / (float)(N + 1)) - 1;
	float temp = sqrt(1 - x1 * x1 - x2 * x2);
	float x = 2 * x1* temp;
	float y = 2 * x2* temp;
	float z = 1 - 2 * (x1*x1 + x2 * x2);
	return glm::vec3(x, y, z)*emitterR;
}
glm::vec3 emitDir(void)
{
	
	float phi = 2 * rand()*PI / RAND_MAX;
	float costheta = 2 * rand() / RAND_MAX - 1;// rand() % (N + 1) / (float)(N + 1);
	//float u =  rand() / RAND_MAX;// rand() % (N + 1) / (float)(N + 1);
	float theta = acos(costheta);
	//float r = pow(u, 1 / 3);
	float r = 1;
	return glm::vec3(r * sin(theta) * sin(phi), r * cos(theta), r * sin(theta) * cos(phi));
}
void DrawParticles(GLuint particles_position_buffer, GLuint particles_color_buffer, GLuint billboard_vertex_buffer,
	GLfloat*g_particule_position_size_data, GLubyte* g_particule_color_data, int ParticlesCount,
	GLuint programID, GLuint VertexArrayID, GLuint TextureID, GLuint Texture, glm::mat4 ProjectionMatrix, glm::mat4 ViewMatrix,
	GLuint CameraRight_worldspace_ID, GLuint CameraUp_worldspace_ID,GLuint ViewProjMatrixID)
{
	// Use our shader
	glUseProgram(programID);

	glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
	//glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLfloat), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf. See above link for details.
	glBufferSubData(GL_ARRAY_BUFFER, 0, ParticlesCount * sizeof(GLfloat) * 4, g_particule_position_size_data);

	glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
	//glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLubyte), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf. See above link for details.
	glBufferSubData(GL_ARRAY_BUFFER, 0, ParticlesCount * sizeof(GLubyte) * 4, g_particule_color_data);

	//

	glEnable(GL_BLEND);
	//glBlendEquation(GL_FUNC_ADD);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	

	
	glBindVertexArray(VertexArrayID);
	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, Texture);
	// Set our "myTextureSampler" sampler to use Texture Unit 0
	glUniform1i(TextureID, 0);

	// Same as the billboards tutorial
	glm::mat4 ViewProjectionMatrix = ProjectionMatrix * ViewMatrix;
	glUniform3f(CameraRight_worldspace_ID, ViewMatrix[0][0], ViewMatrix[1][0], ViewMatrix[2][0]);
	glUniform3f(CameraUp_worldspace_ID, ViewMatrix[0][1], ViewMatrix[1][1], ViewMatrix[2][1]);

	glUniformMatrix4fv(ViewProjMatrixID, 1, GL_FALSE, &ViewProjectionMatrix[0][0]);

	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, billboard_vertex_buffer);
	glVertexAttribPointer(
		0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);

	// 2nd attribute buffer : positions of particles' centers
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
	glVertexAttribPointer(
		1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		4,                                // size : x + y + z + size => 4
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// 3rd attribute buffer : particles' colors
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
	glVertexAttribPointer(
		2,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		4,                                // size : r + g + b + a => 4
		GL_UNSIGNED_BYTE,                 // type
		GL_TRUE,                          // normalized?    *** YES, this means that the unsigned char[4] will be accessible with a vec4 (floats) in the shader ***
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// These functions are specific to glDrawArrays*Instanced*.
	// The first parameter is the attribute buffer we're talking about.
	// The second parameter is the "rate at which generic vertex attributes advance when rendering multiple instances"
	glVertexAttribDivisor(0, 0); // particles vertices : always reuse the same 4 vertices -> 0
	glVertexAttribDivisor(1, 1); // positions : one per quad (its center)                 -> 1
	glVertexAttribDivisor(2, 1); // color : one per quad                                  -> 1

	// Draw the particules !
	
	glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, ParticlesCount);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
}
bool computephysics(Particle& p,double delta)
{
	
#ifndef SNOW
	// midpoint
	p.speed += glm::vec3(0.0f, GRAVITY, 0.0f) * (float)delta * 0.5f;
	p.pos += p.speed * (float)delta+0.5f*(float)(delta*delta)*glm::vec3(0.0f, GRAVITY, 0.0f);
	// Simulate simple physics : floor
#else
	const float resistFactor = 0.5;
	p.speed += glm::vec3(0.0f, resistFactor * GRAVITY, 0.0f) * (float)delta * 0.5f;
	p.pos += p.speed * (float)delta+0.5f*(float)(delta*delta)*glm::vec3(0.0f, resistFactor * GRAVITY, 0.0f);
	glm::vec3 bigDir= glm::vec3(
			(rand() % 2000 - 1000.0f) / 2000.0f,
			(rand() % 2000 - 1000.0f) / 20000.0f,
			(rand() % 2000 - 1000.0f) / 2000.0f
			);
	glm::vec3 smallDir= glm::vec3(
			(rand() % 2000 - 1000.0f) / 10000.0f,
			(rand() % 2000 - 1000.0f) / 100000.0f,
			(rand() % 2000 - 1000.0f) / 10000.0f
			);
	p.speed += bigDir + smallDir;
#endif

#ifdef FOUNTAIN
	glm::vec3 dis = p.pos - sphereC;
	if (p.pos.y -p.size < Origin.y)
	{
		p.life = -1;
		const float resistFactor = 0.5;
		p.cameradistance = -1.0f;
		if (p.level == 2)
			return false;
		p.pos.y = Origin.y+ p.size;
		glm::vec3 vNorm = dot(p.speed, FloorN)*FloorN;
		p.speed -=(1+ FloorCor* resistFactor)*vNorm ;
		return true;
	}
	if (glm::length(dis)-p.size < sphereR)
	{//Simulate simple physics : sphere
		p.life = -1;
		p.cameradistance = -1.0f;
		const float resistFactor = 0.5;
		if (p.level == 2)
			return false;
		glm::vec3 norm = glm::normalize(dis);
		p.pos = sphereC + norm * sphereR*(1.01f + p.size);
		glm::vec3 vNorm = dot(p.speed, norm)*norm;
		p.speed -= (1 + spherecor * resistFactor)*vNorm;
		return true;
	}
#endif
	return false;
}
//void computephysics(int Index, double delta)
//{
//	__m128 ga = _mm_set_ps1(GRAVITY);
//	__m128 ldt = _mm_set_ps1(delta);
//	__m128 coeff = _mm_set_ps1(0.5);
//	__m128 *pa, *pb, pc, pd;
//	__m128 pe = _mm_set_ps1(Origin.y);
//	__m128 *ps;
//	__m128 c1 = _mm_set_ps1(1);
//	__m128 FCor = _mm_set_ps1(FloorCor);
//	// midpoint
//	//speed[Index] += glm::vec3(0.0f, GRAVITY, 0.0f) * (float)delta * 0.5f;
//	pa = (__m128*)(&speed[Index].y);
//	pc = _mm_mul_ps(*pa, ldt);
//	pd = _mm_mul_ps(pc, coeff);
//	*pa = _mm_add_ps(*pa, pd);
//	
//	//pos[Index] += speed[Index] * (float)delta + 0.5f*(float)(delta*delta)*glm::vec3(0.0f, GRAVITY, 0.0f);
//	pa = (__m128*)(&pos[Index].y);
//	pc  = _mm_mul_ps(*pa, ldt);
//	*pa = _mm_add_ps(*pa, pc);
//	pc = _mm_mul_ps(ldt, ldt);
//	pd = _mm_mul_ps(pc, coeff);
//	pc = _mm_mul_ps(pd, ga);
//	*pa = _mm_add_ps(*pa, pc);
//    
//	// Simulate simple physics : floor
//	if (pos[Index].y - psize[Index] < Origin.y)
//	{  
//		ps= (__m128*)(&psize[Index]);
//		//p.pos.y = Origin.y + p.size;
//		pa = (__m128*)(&pos[Index].y);
//		pc = _mm_mul_ps(pe, *ps);
//		*pa = _mm_add_ps(*pa, pc);
//		//glm::vec3 vNorm = dot(p.speed, FloorN)*FloorN;
//		//p.speed -= (1 + FloorCor)*vNorm;
//		pa = (__m128*)(&speed[Index].y);
//		pc = _mm_mul_ps(c1 , FCor);
//		*pa = _mm_add_ps(*pa, pc);
//	}
//
//	//Simulate simple physics : sphere
//	/*glm::vec3 dis = p.pos - sphereC;
//	if (glm::length(dis) - p.size < sphereR)
//	{
//		glm::vec3 norm = glm::normalize(dis);
//		p.pos = sphereC + norm * sphereR*(1.01f + p.size);
//		glm::vec3 vNorm = dot(p.speed, norm)*norm;
//		p.speed -= (1 + spherecor)*vNorm;
//	}*/
//}
void CalculateFrameRate()
{
	static float framesPerSecond = 0.0f; // This will store our fps
	static float lastframeTime = 0.0f; // This will hold the time from the last frame
	float currentTime = glfwGetTime();// GetTickCount()*0.001f * 0.000001f
	++framesPerSecond;
	char strFrameRate[100];
	if (currentTime - lastframeTime > 1.0f)
	{
		lastframeTime = currentTime;
		
		sprintf(strFrameRate,"Current Frames Per Second: %d\n", int(framesPerSecond));
		//SetWindowText(g_hWnd, strFrameRate);
		glfwSetWindowTitle(window, strFrameRate);
		framesPerSecond = 0;
	}
}
