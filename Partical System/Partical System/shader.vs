 #version 150 core
in vec3 position;
in vec2 vertexUV;
in vec3 inColor;
in vec3 inNormal;

const vec3 inLightDir = normalize(vec3(0,-2,2));
out vec3 Color;
out vec3 normal;
out vec3 lightDir;
out vec2 UV;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
void main() 
{
  Color = inColor;
  gl_Position = proj * view * model * vec4(position,1.0);
  vec4 norm4 = transpose(inverse(model)) * vec4(inNormal,1.0);
  normal = normalize(norm4.xyz);
  lightDir = (view * vec4(inLightDir,0)).xyz; 
  // UV of the vertex. No special space for this one.
  UV = vertexUV;
}