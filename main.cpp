 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "rayAccelerator.h"
#include "maths.h"
#include "macros.h"

//Enable OpenGL drawing.  
bool drawModeEnabled = false;

bool  firstFrame = true;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene

#define MAX_DEPTH 6  //number of bounces

#define CAPTION "Whitted Ray-Tracer"
#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

unsigned int FrameCount = 0;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];


// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;

Grid* grid_ptr = NULL;
BVH* bvh_ptr = NULL;
accelerator Accel_Struct = NONE;

int RES_X, RES_Y;

int WindowHandle = 0;



/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");
	
	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs

void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);
	
// unbind the VAO
	glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;
	glClear(GL_COLOR_BUFFER_BIT);

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}


// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top, 
			float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'r':
			camX = Eye.x;
			camY = Eye.y;
			camZ = Eye.z;
			r = Eye.length();
			beta = asinf(camY / r) * 180.0f / 3.14f;
			alpha = atanf(camX / camZ) * 180.0f / 3.14f;
			break;

		case 'c':
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
			printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
			break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


/////////////////////////////////////////////////////YOUR CODE HERE///////////////////////////////////////////////////////////////////////////////////////

float mix(const float& a, const float& b, const float& mix)
{
	return b * mix + a * (1 - mix);
}

float schlick(Vector dir, Vector n, float ior1, float ior2) {
	float r0 = (ior1 - ior2)/(ior1+ior2); // ni - nt / ni + nt
	r0 *= r0;
	float cos1 = dir * n * -1;
	if (ior1 > ior2) {
		float temp = ior1 / ior2; // ni/nt
		float sin1 = temp * temp * (1 - cos1 * cos1);
		if (sin1 > 1) return 1;
		cos1 = sqrt(1 - sin1);
	}
	float x = 1 - cos1;
	return r0 + (1 - r0) * x * x * x * x * x;
}


Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{

	

	
	Vector I = ray.direction.operator*((-1, -1, -1));
	float minDist = INFINITY;
	Object* hitObj = NULL;
	float dist;
	Vector pHit;
	int numObjects = scene->getNumObjects();



	if (Accel_Struct == GRID_ACC) {
		grid_ptr->Traverse(ray,&hitObj,pHit);
	}
	if (Accel_Struct ==	BVH_ACC) {
		bvh_ptr->Traverse(ray, &hitObj, pHit);
	}
	else if (Accel_Struct == NONE) {
		for (int i = 0; i < numObjects; i++) {
			if (scene->getObject(i)->intercepts(ray, dist)) {
				if (dist < minDist) {
					hitObj = scene->getObject(i);
					minDist = dist; // update min distance 
				}
			}
		}
	}
	//test if ray hits an object and store the closest hit point to start of ray
	

	//se nao interseta nenhum obj ent a cor � a do bg
	if (!hitObj) { return scene->GetBackgroundColor(); }


	else {
		//computar a normal no hit point
		if (Accel_Struct == NONE) {
			pHit = ray.origin + ray.direction * minDist;
		}
		
		Vector nHit = hitObj->getNormal(pHit).normalize();

		Color pixel_color = Color();
		bool inside = false;
		//If a ray hits from inside, use the reverse direction of the geometric normal for shading
		if (ray.direction * nHit > 0) {
			nHit *= -1;
			inside = true;
		}

		

		//calcular a contribuicao local das luzes da cena para a cor nesse ponto
		int numLights = scene->getNumLights();
		

		//if reflective object
		if (hitObj->GetMaterial()->GetTransmittance() > 0 && depth < MAX_DEPTH) {
			//offset intersections at the secondary rays
			Vector newOrg = pHit + (nHit * EPSILON);
			Vector newOrg2 = pHit - (nHit * EPSILON);

			//calculate ray in the reflected direction
			Vector reflectionDir = ray.direction - nHit * 2 * (ray.direction * nHit);
			Ray rRay = Ray(newOrg, reflectionDir);

			Color rColor = rayTracing(rRay, depth + 1, ior_1);

			float fresneleffect;
			float facingratio = ray.direction * nHit * -1;

			fresneleffect = schlick(ray.direction, nHit, ior_1, hitObj->GetMaterial()->GetRefrIndex());
			float eta = (inside) ? hitObj->GetMaterial()->GetRefrIndex() : 1 / hitObj->GetMaterial()->GetRefrIndex(); // are we inside or outside the surface?
			float cosi = nHit * ray.direction * -1;
			float k = 1 - eta * eta * (1 - cosi * cosi);
			Vector refractionDir = ray.direction * eta + nHit * (eta * cosi - sqrt(k));
			refractionDir = refractionDir.normalize();
			Ray tRay = Ray(newOrg2, refractionDir);
			Color tColor = rayTracing(tRay, depth + 1, ior_1);
			//reduce rColor by the specular reflection coefficient and add to color???
			pixel_color += (rColor * fresneleffect * hitObj->GetMaterial()->GetSpecColor()) + (tColor * (1 - fresneleffect));

		}else if (hitObj->GetMaterial()->GetReflection() > 0 && depth < MAX_DEPTH) {
			//offset intersections at the secondary rays
			Vector newOrg = pHit + (nHit * EPSILON);
			Vector newOrg2 = pHit - (nHit * EPSILON);
			Vector reflectionDir = ray.direction - nHit * 2 * (ray.direction * nHit);

			//calculate ray in the reflected direction
			//NORMAL REFLECTION
			/* NORMAL REFLECTION
			Ray rRay = Ray(newOrg, reflectionDir);
			*/
			//END NORMAL REFLECTION

			//FUZZY REFLECTION
			float roughness_param = 0.4f;
			Vector sphereCenter = newOrg + reflectionDir;
			Vector newSphereCenter = sphereCenter + rnd_unit_sphere() * roughness_param;
			Vector newReflectionDir = newSphereCenter - newOrg;
			Ray rRay = Ray(newOrg, newReflectionDir);
			//END FUZZY REFLECTION
			Color rColor = rayTracing(rRay, depth + 1, ior_1) ;

			pixel_color += (rColor * hitObj->GetMaterial()->GetSpecular()*hitObj->GetMaterial()->GetSpecColor());
		}
		
		for (int l = 0; l < numLights; l++) {
			bool isShadow = false;
			Light* luz = scene->getLight(l);

			//without anti aliasing
			Vector lightArray[9];
			lightArray[0] = luz->position;
			lightArray[1] = Vector(luz->position.x + 0.05, luz->position.y, luz->position.z);
			lightArray[2] = Vector(luz->position.x - 0.05, luz->position.y, luz->position.z);

			lightArray[3] = Vector(luz->position.x, luz->position.y - 0.05, luz->position.z);
			lightArray[4] = Vector(luz->position.x + 0.05, luz->position.y - 0.05, luz->position.z);
			lightArray[5] = Vector(luz->position.x - 0.05, luz->position.y - 0.05, luz->position.z);

			lightArray[6] = Vector(luz->position.x, luz->position.y + 0.04, luz->position.z);
			lightArray[7] = Vector(luz->position.x + 0.05, luz->position.y + 0.05, luz->position.z);
			lightArray[8] = Vector(luz->position.x - 0.05, luz->position.y + 0.05, luz->position.z);
			// 
			//with anti aliasing
			Vector up = (luz->position + Vector(2.0, 0.0, 0.0)).normalize();
			Vector right = (luz->position + Vector(0.0, 2.0, 0.0)).normalize();
			float rndVal1 = rand_float();
			float rndVal2 = rand_float();
			Vector randomPlace = luz->position + up * rndVal1 + right * rndVal2;


			//SOFT SHADOWS WITHOUT ANTI ALIASING
			/*
			for (int k = 0; k < 9; k++) {
				Vector L = (lightArray[k] - pHit).normalize();
				//

				Vector shadowpHit = pHit + nHit * EPSILON;
				float shadowRayDist = INFINITY;
				Ray shadowRay = Ray(shadowpHit, L);
				//if  (!point in shadow); trace shadow ray
				for (int i = 0; i < numObjects; i++) {
					Object* obj = scene->getObject(i);
					if (obj->intercepts(shadowRay, shadowRayDist)) {
						isShadow = true;
						break;
					}
				}

				if (float intensity = (L * nHit)/9.0 > 0) {
					if (!isShadow) {
						Vector H = ((L - ray.direction) / 2).normalize();

						Color diff = luz->color * hitObj->GetMaterial()->GetDiffColor() * hitObj->GetMaterial()->GetDiffuse() * max(0.0f, nHit * L);
						Color spec = luz->color * hitObj->GetMaterial()->GetSpecColor() * hitObj->GetMaterial()->GetSpecular() * pow(max(0.0f, H * nHit), hitObj->GetMaterial()->GetShine());

						pixel_color += (diff + spec)*(1.0/9.0);
						//pixel_color = pixel_color.clamp();

					}
				}
			}
			*/
			// END SOFT SHADOWS WITHOUT ANTI ALIASING
	
			// SOFT SHADOWS WITH ANTI ALIASING
			
			Vector L = (randomPlace - pHit).normalize();
			Vector shadowpHit = pHit + nHit * EPSILON;
			float shadowRayDist = INFINITY;
			Ray shadowRay = Ray(shadowpHit, L);

			if (Accel_Struct == NONE) {
				
				//if  (!point in shadow); trace shadow ray
				for (int i = 0; i < numObjects; i++) {
					Object* obj = scene->getObject(i);
					if (obj->intercepts(shadowRay, shadowRayDist)) {
						isShadow = true;
						break;
					}
				}	
			}

			if (Accel_Struct == GRID_ACC) {
				if (grid_ptr->Traverse(shadowRay)) {
					isShadow = true;
				}
			}

			if (Accel_Struct == BVH_ACC) {
				if (bvh_ptr->Traverse(shadowRay)) {
					isShadow = true;
				}
			}

			if (float intensity = (L * nHit) > 0) {
				if (!isShadow) {
					Vector H = ((L - ray.direction) / 2).normalize();

					Color diff = luz->color * hitObj->GetMaterial()->GetDiffColor() * hitObj->GetMaterial()->GetDiffuse() * max(0.0f, nHit * L);
					Color spec = luz->color * hitObj->GetMaterial()->GetSpecColor() * hitObj->GetMaterial()->GetSpecular() * pow(max(0.0f, H * nHit), hitObj->GetMaterial()->GetShine());

					pixel_color += (diff + spec);
					//pixel_color = pixel_color.clamp();

				}
			}
			
			//END SOFT SHADOWS WITH ANTI ALIASING

			//NORMAL SHADOWS
			/*
			Vector L = (luz->position - pHit).normalize();

			Vector shadowpHit = pHit + nHit * EPSILON;
			float shadowRayDist = INFINITY;
			Ray shadowRay = Ray(shadowpHit, L);


			if (Accel_Struct == NONE) {

				//if  (!point in shadow); trace shadow ray
				for (int i = 0; i < numObjects; i++) {
					Object* obj = scene->getObject(i);
					if (obj->intercepts(shadowRay, shadowRayDist)) {
						isShadow = true;
						break;
					}
				}
			}

			if (Accel_Struct == GRID_ACC) {
				if (grid_ptr->Traverse(shadowRay)) {
					isShadow = true;
				}
			}

			if (Accel_Struct == BVH_ACC) {
				if (bvh_ptr->Traverse(shadowRay)) {
					isShadow = true;
				}
			}

			if (float intensity = L * nHit > 0) {
				if (!isShadow) {
					Vector H = ((L - ray.direction) / 2).normalize();

					Color diff = luz->color * hitObj->GetMaterial()->GetDiffColor() * hitObj->GetMaterial()->GetDiffuse() * max(0.0f, nHit * L);
					Color spec = luz->color * hitObj->GetMaterial()->GetSpecColor() * hitObj->GetMaterial()->GetSpecular() * pow(max(0.0f, H * nHit), hitObj->GetMaterial()->GetShine());

					pixel_color += (diff + spec);
					//pixel_color = pixel_color.clamp();

				}
			}
			*/
			//END NORMAL SHADOWS

		}

		

		return pixel_color;
	}
}



// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	int index_col2 = 0;
	unsigned int counter = 0;
	unsigned int counter2 = 0;
	float nSample = scene->GetSamplesPerPixel();

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}

	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color;

			Vector pixel;  //viewport coordinates

			color = Color(0, 0, 0);

			//DOF START
			Camera* camera = scene->GetCamera();
			//float focalDist = 1.5f;
			float viewDist = camera->GetPlaneDist();

			if (!nSample) nSample = 1;

			
			/*for (int p = 0; p < nSample; p++) {
				Vector ls = rnd_unit_disk() * camera->GetAperture();
				Vector ps = Vector(x, y, -viewDist);
	
				Ray ray = scene->GetCamera()->PrimaryRay(ls, ps);
				color += rayTracing(ray, 1, 1.0).clamp();

			}
			color = color * (1.0f / nSample);*/
			

			//ANTI ALIASING WITH DEPTH OF FIELD RENDER
			/**/
			for (int p = 0; p < nSample; p++) 
				for (int q = 0; q < nSample; q++) {
					Vector ls = rnd_unit_disk() * camera->GetAperture();


					float rndVal = rand_float();

					pixel.x = x + (p + rndVal) / nSample;
					pixel.y = y + (q + rndVal) / nSample;
					pixel.z = -viewDist;

					Ray ray = scene->GetCamera()->PrimaryRay(ls, pixel);   //function from camera.h

					color += rayTracing(ray, 1, 1.0);
				}

			color = color * (1 / (nSample * nSample));
			
			//END ANTI ALIASING WITH DEPTH OF FIELD RENDER


			//NORMAL RENDER
			/* NORMAL
			pixel.x = x + 0.5f;
			pixel.y = y + 0.5f;
			Ray ray = scene->GetCamera()->PrimaryRay(pixel);
			color = rayTracing(ray, 1, 1.0).clamp();
			*/
			//END NORMAL RENDER
			

			//color = scene->GetBackgroundColor(); //TO CHANGE - just for the template

			if (firstFrame) {
				img_Data[counter++] = u8fromfloat((float)color.r());
				img_Data[counter++] = u8fromfloat((float)color.g());
				img_Data[counter++] = u8fromfloat((float)color.b());
			}
			else {
				img_Data[counter++] = u8fromfloat((float)(color.r() + colors[counter2]) / 2.0f);
				counter2++;
				img_Data[counter++] = u8fromfloat((float)(color.g() + colors[counter2]) / 2.0f);
				counter2++;
				img_Data[counter++] = u8fromfloat((float)(color.b() + colors[counter2]) / 2.0f);
				counter2++;

			}
			/*img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());*/

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;

				if (firstFrame) {
					colors[index_col++] = (float)color.r();
					colors[index_col++] = (float)color.g();
					colors[index_col++] = (float)color.b();
				}
				else {
					colors[index_col++] = (float)((color.r() + colors[index_col2]) / 2.0f);
					index_col2++;
					colors[index_col++] = (float)((color.g() + colors[index_col2]) / 2.0f);
					index_col2++;
					colors[index_col++] = (float)((color.b() + colors[index_col2]) / 2.0f);
					index_col2++;

				}

				/*colors[index_col++] = (float)color.r();
				colors[index_col++] = (float)color.g();
				colors[index_col++] = (float)color.b();*/
			}
		}

	}
	if (firstFrame) firstFrame = false;
	if (drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}


///////////////////////////////////////////////////////////////////////  SETUP     ///////////////////////////////////////////////////////

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}
void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			cout << "Input the Scene Name: ";
			cin >> input_user;
			strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
			strcat_s(scene_name, sizeof(scene_name), input_user);

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
		printf("Scene loaded.\n\n");
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}


	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);

	Accel_Struct = scene->GetAccelStruct();   //Type of acceleration data structure

	if (Accel_Struct == GRID_ACC) {
		grid_ptr = new Grid();
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		grid_ptr->Build(objs);
		printf("Grid built.\n\n");
	}
	else if (Accel_Struct == BVH_ACC) {
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();
		bvh_ptr = new BVH();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		bvh_ptr->Build(objs);
		printf("BVH built.\n\n");
	}
	else
		printf("No acceleration data structure.\n\n");

	unsigned int spp = scene->GetSamplesPerPixel();
	if (spp == 0)
		printf("Whitted Ray-Tracing\n");
	else
		printf("Distribution Ray-Tracing\n");

}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int 
		ch;
	if (!drawModeEnabled) {

		do {
			init_scene();

			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X*RES_Y * sizeof(float);
		size_colors = 3 * RES_X*RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
		memset(colors, 0, size_colors);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////