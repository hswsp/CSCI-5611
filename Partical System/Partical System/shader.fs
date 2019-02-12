#version 150 core
in vec3 Color;
in vec3 normal;
in vec3 lightDir;
in vec2 UV;
out vec4 outColor;
// Values that stay constant for the whole mesh.
uniform int texID = -1;
uniform sampler2D myTextureSampler;
const float ambient = .5;
void main() 
{
	if(texID==-1)
	 {
		vec3 diffuseC = Color*max(dot(lightDir,normal),0);
		vec3 ambC = Color*ambient;
		outColor = vec4(ambC, 1.0);  //diffuseC+(Red, Green, Blue, Alpha)
	 }
	 else
	 {
		vec3 texcolor = texture(myTextureSampler,UV).rgb;
		outColor = vec4(texcolor, 1.0); 
	 }
	
}