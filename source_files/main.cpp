#pragma warning(disable : 5208)
#define NOMINMAX

#include <limits>
#include <windows.h>
#include <mmsystem.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <vector> 

// OpenGL includes
#include <GL/glew.h>
#include <GL/freeglut.h>

// Assimp includes
#include <assimp/cimport.h> 
#include <assimp/scene.h> 
#include <assimp/postprocess.h> 

// Project includes
#include "maths_funcs.h"
//#include "ik_maths.h"

// GLM includes
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

/*----------------------------------------------------------------------------
MESH TO LOAD
----------------------------------------------------------------------------*/
// this mesh is a dae file format but you should be able to use any other format too, obj is typically what is used
// put the mesh in your project directory, or provide a filepath for it here
#define ARM "U:/animation_proj/Project1/Project1/cone.obj"
#define BODY "U:/animation_proj/Project1/Project1/body.obj"
#define BALL "U:/animation_proj/Project1/Project1/ball.obj"
#define JOINT "U:/animation_proj/Project1/Project1/joint.obj"
/*----------------------------------------------------------------------------
----------------------------------------------------------------------------*/

#pragma region SimpleTypes
typedef struct
{
	size_t mPointCount = 0;
	std::vector<vec3> mVertices;
	std::vector<vec3> mNormals;
	std::vector<vec2> mTextureCoords;
} ModelData;
#pragma endregion SimpleTypes

using namespace std;
GLuint shaderProgramID;

ModelData ball, body, arm, joint;
unsigned int vao1, vao2, vao3, vao4 = 0;

int width = 800;
int height = 600;

GLuint loc1, loc2, loc3;

// camera stuff
glm::vec3 cameraPos = glm::vec3(2.0f, 0.0f, 16.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

int projType = 0;
float fov = 45.0f;

// inverse kinematics
bool a_soln = true;
bool ccd = false;

float arm_length = 2.708;
float last_theta = 0;
glm::vec2 arm_angles(90.0f, 0.0f);

glm::vec3 start_pos(8.0f, 0.0f, 0.0f);

float angles[3] = { 0.0f, 0.0f, 0.0f };
glm::vec3 root_pos = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 end_pos(8.0f, 0.0f, 0.0f);
glm::vec3 links[3] = { glm::vec3(5.4f, 0.0f, 0.0f), glm::vec3(2.7f, 0.0f, 0.0f), root_pos };
int link = 2;

GLfloat rotate_l1, rotate_l2 = 0.0f;
glm::vec2 points[4] = { glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f) };
float t = 0;
int tcount = 0;

glm::vec3 spline_point(0.0f, 0.0f, 0.0f);
glm::vec3 ctrl_points[3] = { glm::vec3(-2.0f, 4.0f, 0.0f), glm::vec3(0.0f, 6.0f, 0.0f), glm::vec3(2.0f, 4.0f, 0.0f) };
int i = 0;
int ctrl_count = 0;

float tx = 0.0f;
float ty = 2.0f;

#pragma region MESH LOADING
/*----------------------------------------------------------------------------
MESH LOADING FUNCTION
----------------------------------------------------------------------------*/

ModelData load_mesh(const char* file_name) {
	ModelData modelData;

	/* Use assimp to read the model file, forcing it to be read as    */
	/* triangles. The second flag (aiProcess_PreTransformVertices) is */
	/* relevant if there are multiple meshes in the model file that   */
	/* are offset from the origin. This is pre-transform them so      */
	/* they're in the right position.                                 */
	const aiScene* scene = aiImportFile(
		file_name,
		aiProcess_Triangulate | aiProcess_PreTransformVertices
	);

	if (!scene) {
		fprintf(stderr, "ERROR: reading mesh %s\n", file_name);
		return modelData;
	}

	printf("  %i materials\n", scene->mNumMaterials);
	printf("  %i meshes\n", scene->mNumMeshes);
	printf("  %i textures\n", scene->mNumTextures);

	for (unsigned int m_i = 0; m_i < scene->mNumMeshes; m_i++) {
		const aiMesh* mesh = scene->mMeshes[m_i];
		printf("    %i vertices in mesh\n", mesh->mNumVertices);
		modelData.mPointCount += mesh->mNumVertices;
		for (unsigned int v_i = 0; v_i < mesh->mNumVertices; v_i++) {
			if (mesh->HasPositions()) {
				const aiVector3D* vp = &(mesh->mVertices[v_i]);
				modelData.mVertices.push_back(vec3(vp->x, vp->y, vp->z));
			}
			if (mesh->HasNormals()) {
				const aiVector3D* vn = &(mesh->mNormals[v_i]);
				modelData.mNormals.push_back(vec3(vn->x, vn->y, vn->z));
			}
			if (mesh->HasTextureCoords(0)) {
				const aiVector3D* vt = &(mesh->mTextureCoords[0][v_i]);
				modelData.mTextureCoords.push_back(vec2(vt->x, vt->y));
			}
			if (mesh->HasTangentsAndBitangents()) {
				/* You can extract tangents and bitangents here              */
				/* Note that you might need to make Assimp generate this     */
				/* data for you. Take a look at the flags that aiImportFile  */
				/* can take.                                                 */
			}
		}
	}

	aiReleaseImport(scene);
	return modelData;
}

#pragma endregion MESH LOADING

// Shader Functions- click on + to expand
#pragma region SHADER_FUNCTIONS
char* readShaderSource(const char* shaderFile) {
	FILE* fp;
	fopen_s(&fp, shaderFile, "rb");

	if (fp == NULL) { return NULL; }

	fseek(fp, 0L, SEEK_END);
	long size = ftell(fp);

	fseek(fp, 0L, SEEK_SET);
	char* buf = new char[size + 1];
	fread(buf, 1, size, fp);
	buf[size] = '\0';

	fclose(fp);

	return buf;
}


static void AddShader(GLuint ShaderProgram, const char* pShaderText, GLenum ShaderType)
{
	// create a shader object
	GLuint ShaderObj = glCreateShader(ShaderType);

	if (ShaderObj == 0) {
		std::cerr << "Error creating shader..." << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}
	const char* pShaderSource = readShaderSource(pShaderText);

	// Bind the source code to the shader, this happens before compilation
	glShaderSource(ShaderObj, 1, (const GLchar**)&pShaderSource, NULL);
	// compile the shader and check for errors
	glCompileShader(ShaderObj);
	GLint success;
	// check for shader related errors using glGetShaderiv
	glGetShaderiv(ShaderObj, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar InfoLog[1024] = { '\0' };
		glGetShaderInfoLog(ShaderObj, 1024, NULL, InfoLog);
		std::cerr << "Error compiling "
			<< (ShaderType == GL_VERTEX_SHADER ? "vertex" : "fragment")
			<< " shader program: " << InfoLog << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}
	// Attach the compiled shader object to the program object
	glAttachShader(ShaderProgram, ShaderObj);
}

GLuint CompileShaders()
{
	//Start the process of setting up our shaders by creating a program ID
	//Note: we will link all the shaders together into this ID
	shaderProgramID = glCreateProgram();
	if (shaderProgramID == 0) {
		std::cerr << "Error creating shader program..." << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}

	// Create two shader objects, one for the vertex, and one for the fragment shader
	AddShader(shaderProgramID, "U:/animation_proj/Project1/Project1/simpleVertexShader.txt", GL_VERTEX_SHADER);
	AddShader(shaderProgramID, "U:/animation_proj/Project1/Project1/simpleFragmentShader.txt", GL_FRAGMENT_SHADER);

	GLint Success = 0;
	GLchar ErrorLog[1024] = { '\0' };
	// After compiling all shader objects and attaching them to the program, we can finally link it
	glLinkProgram(shaderProgramID);
	// check for program related errors using glGetProgramiv
	glGetProgramiv(shaderProgramID, GL_LINK_STATUS, &Success);
	if (Success == 0) {
		glGetProgramInfoLog(shaderProgramID, sizeof(ErrorLog), NULL, ErrorLog);
		std::cerr << "Error linking shader program: " << ErrorLog << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}

	// program has been successfully linked but needs to be validated to check whether the program can execute given the current pipeline state
	glValidateProgram(shaderProgramID);
	// check for program related errors using glGetProgramiv
	glGetProgramiv(shaderProgramID, GL_VALIDATE_STATUS, &Success);
	if (!Success) {
		glGetProgramInfoLog(shaderProgramID, sizeof(ErrorLog), NULL, ErrorLog);
		std::cerr << "Invalid shader program: " << ErrorLog << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}
	// Finally, use the linked shader program
	// Note: this program will stay in effect for all draw calls until you replace it with another or explicitly disable its use
	glUseProgram(shaderProgramID);
	return shaderProgramID;
}
#pragma endregion SHADER_FUNCTIONS

// VBO Functions - click on + to expand
#pragma region VBO_FUNCTIONS
void generateObjectBufferMesh1(GLuint vao, ModelData mesh_data, GLuint programID) {
	/*----------------------------------------------------------------------------
	LOAD MESH HERE AND COPY INTO BUFFERS
	----------------------------------------------------------------------------*/

	//Note: you may get an error "vector subscript out of range" if you are using this code for a mesh that doesnt have positions and normals
	//Might be an idea to do a check for that before generating and binding the buffer.

	unsigned int vp_vbo = 0;
	unsigned int vn_vbo = 0;
	unsigned int vt_vbo = 0;

	loc1 = glGetAttribLocation(programID, "vertex_position");
	loc2 = glGetAttribLocation(programID, "vertex_normal");
	loc3 = glGetAttribLocation(programID, "vertex_texture");

	glGenBuffers(1, &vp_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vp_vbo);
	glBufferData(GL_ARRAY_BUFFER, mesh_data.mPointCount * sizeof(vec3), &mesh_data.mVertices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &vn_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vn_vbo);
	glBufferData(GL_ARRAY_BUFFER, mesh_data.mPointCount * sizeof(vec3), &mesh_data.mNormals[0], GL_STATIC_DRAW);

	glGenBuffers(1, &vt_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vt_vbo);
	glBufferData(GL_ARRAY_BUFFER, mesh_data.mPointCount * sizeof(vec2), &mesh_data.mTextureCoords[0], GL_STATIC_DRAW);

	glBindVertexArray(vao);

	glEnableVertexAttribArray(loc1);
	glBindBuffer(GL_ARRAY_BUFFER, vp_vbo);
	glVertexAttribPointer(loc1, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	glEnableVertexAttribArray(loc2);
	glBindBuffer(GL_ARRAY_BUFFER, vn_vbo);
	glVertexAttribPointer(loc2, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	glEnableVertexAttribArray(loc3);
	glBindBuffer(GL_ARRAY_BUFFER, vt_vbo);
	glVertexAttribPointer(loc3, 2, GL_FLOAT, GL_FALSE, 0, NULL);

}
#pragma endregion VBO_FUNCTIONS

#pragma region SIMPLE_IK
glm::vec2 analytical_soln(glm::vec3 target_pos) {

	float d = 0;
	float l1 = 0, l2 = 0;
	float ex = 0, ey = 0;
	float theta_t = 0, theta1 = 0, theta2 = 0;

	float angle = 0;

	float cos2 = 0, sin2 = 0, tan1 = 0;
	float angle1 = 0, angle2 = 0;

	ex = target_pos.x;
	ey = target_pos.y;

	l1 = arm_length;
	l2 = arm_length;

	cos2 = ((ex * ex) + (ey * ey) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2);

	if (cos2 >= -1.0 && cos2 <= 1.0) {

		d = glm::sqrt((ex * ex) + (ey * ey));

		theta_t = glm::acos(ex / d);

		theta1 = glm::acos(((l1 * l1) + (ex * ex) + (ey * ey) - (l2 * l2)) / (2 * l1 * d)) + theta_t;
		std::cout << "Theta 1: " << glm::degrees(theta1) << endl;

		//upper_arm_angle = theta1;

		theta2 = -1.0f * (glm::radians(180.0f) - glm::acos(((l1 * l1) + (l2 * l2) - (d * d)) / (2 * l1 * l2)));
		std::cout << "Theta 2: " << glm::degrees(theta2) << endl;

		std::cout << "Start Pos: " << start_pos.x << ", " << start_pos.y << endl;
		//lower_arm_angle = theta2;
	}

	else {

		angle = glm::tan(start_pos.y/start_pos.x);
		theta1 = angle;
		theta2 = 0.0f;

		//std::cout << "Other Theta 1: " << glm::degrees(theta1) << endl;
		//std::cout << "Other Theta 2: " << glm::degrees(theta2) << endl;
	}

	last_theta = theta1;

	return glm::vec2(glm::degrees(theta1), glm::degrees(theta2));
}

#pragma endregion SIMPLE_IK

#pragma region CCD

float calc_angle(glm::vec3 target_pos, int i) {

	glm::vec3 v0, v1, norm_vec;
	float mag_v0, mag_v1;
	float end_length;
	float angle;

	v0 = target_pos - links[i];
	//std::cout << "links: " << links[i].x << ", " << links[i].y << ", " << links[i].z << endl;
	//std::cout << "v0: " << v0.x << ", " << v0.y << ", " << v0.z << endl;
	v1 = end_pos - links[i];
	//std::cout << "v1: " << v1.x << ", " << v1.y << ", " << v1.z << endl;

	end_length = glm::distance(end_pos, links[i]);
	std::cout << "End Length: " << end_length << endl;

	mag_v0 = glm::sqrt(glm::pow(v0.x, 2) + glm::pow(v0.y, 2) + glm::pow(v0.z, 2));
	//std::cout << " Mag v0: " << mag_v0 << endl;
	mag_v1 = glm::sqrt(glm::pow(v1.x, 2) + glm::pow(v1.y, 2) + glm::pow(v1.z, 2));
	//std::cout << " Mag v1: " << mag_v1 << endl;

	norm_vec = v0 / mag_v0;
	//std::cout << "Norm Vec: " << norm_vec.x << ", " << norm_vec.y << ", " << norm_vec.z << endl;
	end_pos = links[0] + (end_length * norm_vec);
	std::cout << "End Pos: " << end_pos.x << ", " << end_pos.y << ", " << end_pos.z << endl;

	angle = glm::degrees(glm::acos(glm::dot(v0, v1) / (mag_v0 * mag_v1)));
	//glm::degrees(glm::acos(glm::dot(v0, v1) / (mag_v0 * mag_v1)));

	return(angle);

}

void CCD() {

	float target_dist = glm::distance(start_pos, end_pos);;
	int i = 0;

	//target_dist = glm::distance(start_pos, end_pos);
	//std::cout << "target_dist: " << target_dist << endl;

	while (target_dist > 0.05 && i < 3) {

		angles[i] = calc_angle(start_pos, i);

		target_dist = glm::distance(start_pos, end_pos);
		std::cout << "Target Dist: " << target_dist << endl;
		i++;

	}

	std::cout << "Angles: " << angles[0] << ", " << angles[1] << ", " << angles[2] << endl;
}

#pragma endregion CCD

#pragma region SPLINES

glm::vec2 calc_spline_point(float t) {

	int p0, p1, p2, p3;
	glm::vec2 new_t(0.0f, 0.0f);

	p1 = (int)t + 1;
	p2 = p1 + 1;
	p3 = p2 + 1;
	p0 = p1 - 1;

	t = t - (int)t;

	float tt = t * t;
	float ttt = t * t * t;

	float q1 = -ttt + 2.0 * tt - t;
	float q2 = 3.0f * ttt - 5.0f * tt + 2.0f;
	float q3 = -3.0f * ttt + 4.0f * tt + t;
	float q4 = ttt - t;

	new_t.x = 0.5f * (points[p0].x * q1 + points[p1].x * q2 + points[p2].x * q3 + points[p3].x * q4);
	new_t.y = 0.5f * (points[p0].y * q1 + points[p1].y * q2 + points[p2].y * q3 + points[p3].y * q4);

	return new_t;
}

#pragma endregion SPLINES

void display() {

	// tell GL to only draw onto a pixel if the shape is closer to the viewer
	glEnable(GL_DEPTH_TEST); // enable depth-testing
	glDepthFunc(GL_LESS); // depth-testing interprets a smaller value as "closer"
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// --------------------------------- CAMERA --------------------------------------

	//setting up projection matrix
	glm::mat4 persp_proj = glm::perspective(glm::radians(fov), (float)width / (float)height, 1.0f, 100.0f);
	if (projType == 0) {
		persp_proj = glm::perspective(45.0f, (float)width / (float)height, 1.0f, 100.0f);
	}

	else if (projType == 1) {
		persp_proj = glm::ortho(-16.0f, 16.0f, -12.0f, 12.0f, 1.0f, 100.0f);
	}

	//setting up camera
	//lookAt(position, target, up vector);
	glm::mat4 view = glm::mat4(1.0f);
	view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);

	glUseProgram(shaderProgramID);

	//Declare your uniform variables that will be used in your shader
	int matrix_location = glGetUniformLocation(shaderProgramID, "model");
	int view_mat_location = glGetUniformLocation(shaderProgramID, "view");
	int proj_mat_location = glGetUniformLocation(shaderProgramID, "proj");

	// --------------------------------- BALL --------------------------------------

	glm::mat4 ball_model = glm::mat4(1.0f);
	
	ball_model = glm::translate(glm::mat4(1.0f), start_pos);
		
	glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(ball_model));
	glBindVertexArray(vao3);
	glDrawArrays(GL_TRIANGLES, 0, ball.mPointCount);

	// --------------------------------- BODY --------------------------------------

	glm::mat4 body_model = glm::mat4(1.0f);
	body_model = glm::translate(glm::mat4(1.0f), glm::vec3(-0.5f, -1.0f, 0.0f));
	body_model = glm::rotate(body_model, glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

	glm::mat4 global0 = body_model;

	// update uniforms & draw
	glUniformMatrix4fv(proj_mat_location, 1, GL_FALSE, glm::value_ptr(persp_proj));
	glUniformMatrix4fv(view_mat_location, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(global0));

	glBindVertexArray(vao1);
	//glDrawArrays(GL_TRIANGLES, 0, body.mPointCount);

	if (a_soln) {

		// --------------------------------- UPPER ARM --------------------------------------

		glm::mat4 upper_arm = glm::mat4(1.0f);
		upper_arm = glm::rotate(upper_arm, glm::radians(arm_angles.x), glm::vec3(0.0f, 0.0f, 1.0f));

		glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(upper_arm));
		glBindVertexArray(vao2);
		glDrawArrays(GL_TRIANGLES, 0, arm.mPointCount);

		// --------------------------------- LOWER ARM --------------------------------------

		glm::mat4 lower_arm = glm::mat4(1.0f);
		lower_arm = glm::translate(glm::mat4(1.0f), glm::vec3(2.7f, 0.0f, 0.0f));
		lower_arm = glm::rotate(lower_arm, glm::radians(arm_angles.y), glm::vec3(0.0f, 0.0f, 1.0f));
		lower_arm = upper_arm * lower_arm;

		glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(lower_arm));
		glBindVertexArray(vao2);
		glDrawArrays(GL_TRIANGLES, 0, arm.mPointCount);
	}

	else if (ccd) {

		// --------------------------------- LINK 1 --------------------------------------

		glm::mat4 link1 = glm::mat4(1.0f);
		//link1 = glm::translate(link1, glm::vec3(10.0f, 10.0f, 0.0f));
		link1 = glm::rotate(link1, glm::radians(angles[2]), glm::vec3(0.0f, 0.0f, 1.0f));

		glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(link1));
		glBindVertexArray(vao2);
		glDrawArrays(GL_TRIANGLES, 0, arm.mPointCount);

		// --------------------------------- LINK 2 --------------------------------------

		glm::mat4 link2 = glm::mat4(1.0f);
		link2 = glm::translate(glm::mat4(1.0f), glm::vec3(2.7f, 0.0f, 0.0f));
		link2 = glm::rotate(link2, glm::radians(angles[1]), glm::vec3(0.0f, 0.0f, 1.0f));
		link2 = link1 * link2;

		glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(link2));
		glBindVertexArray(vao2);
		glDrawArrays(GL_TRIANGLES, 0, arm.mPointCount);

		// --------------------------------- LINK 3 --------------------------------------

		glm::mat4 link3 = glm::mat4(1.0f);
		link3 = glm::translate(glm::mat4(1.0f), glm::vec3(2.7f, 0.0f, 0.0f));
		link3 = glm::rotate(link3, glm::radians(angles[0]), glm::vec3(0.0f, 0.0f, 1.0f));
		link3 = link2 * link3;

		glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(link3));
		glBindVertexArray(vao2);
		glDrawArrays(GL_TRIANGLES, 0, arm.mPointCount);


	}

	glutSwapBuffers();
}


void updateScene() {

	/*static DWORD last_time = 0;
	DWORD curr_time = timeGetTime();
	if (last_time == 0)
		last_time = curr_time;
	float delta = (curr_time - last_time) * 0.001f;
	last_time = curr_time;*/

	//if (0.0f <= tx <= 2.0f) {
	//	
	//	tx += 0.02f;
	//	//ty += 0.02f;
	//	std::cout << "tx: " << tx << endl;
	//}


	//else {

	//	tx = 2.0f;
	//	//ty -= 2.0f;
	//}


	// Draw the next frame
	glutPostRedisplay();
}


void init()
{

	// Set up the shaders
	GLuint shaderProgramID = CompileShaders();
	// load mesh into a vertex buffer array

	body = load_mesh(BODY);
	glGenVertexArrays(1, &vao1);
	generateObjectBufferMesh1(vao1, body, shaderProgramID);

	arm = load_mesh(ARM);
	glGenVertexArrays(1, &vao2);
	generateObjectBufferMesh1(vao2, arm, shaderProgramID);

	ball = load_mesh(BALL);
	glGenVertexArrays(1, &vao3);
	generateObjectBufferMesh1(vao3, ball, shaderProgramID);

	joint = load_mesh(JOINT);
	glGenVertexArrays(1, &vao4);
	generateObjectBufferMesh1(vao4, joint, shaderProgramID);

}

// Placeholder code for the keypress
void keypress(unsigned char key, int x, int y) {

	switch (key) {
	case 'z':
		// move camera backwards
		cameraPos += glm::vec3(0.0f, 0.0f, 2.0f);
		std::cout << "camera pos: " << cameraPos.x << ", " << cameraPos.y << ", " << cameraPos.z << endl;
		break;
	case 'x':
		// move camera forewards
		cameraPos -= glm::vec3(0.0f, 0.0f, 2.0f);
		std::cout << "camera pos: " << cameraPos.x << ", " << cameraPos.y << ", " << cameraPos.z << endl;
		break;
	case 'w':
		// move camera upwards
		cameraPos += glm::vec3(0.0f, 2.0f, 0.0f);
		std::cout << "camera pos: " << cameraPos.x << ", " << cameraPos.y << ", " << cameraPos.z << endl;
		break;
	case 's':
		// move camera downwards
		cameraPos -= glm::vec3(0.0f, 2.0f, 0.0f);
		std::cout << "camera pos: " << cameraPos.x << ", " << cameraPos.y << ", " << cameraPos.z << endl;
		break;
	case 'a':
		// move camera left
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp));
		std::cout << "camera pos: " << cameraPos.x << ", " << cameraPos.y << ", " << cameraPos.z << endl;
		break;
	case 'd':
		// move camera right
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp));
		std::cout << "camera pos: " << cameraPos.x << ", " << cameraPos.y << ", " << cameraPos.z << endl;
		break;

		// inverse kinematics

	case 'o':
		start_pos.x += 0.5f;

		if (a_soln) {

			arm_angles = analytical_soln(start_pos);
		}

		else if (CCD) {

			CCD();
		}

		break;

	case 'p':
		start_pos.x -= 0.5f;

		if (a_soln) {

			arm_angles = analytical_soln(start_pos);
		}

		else if (CCD) {

			CCD();
		}

		break;

	case 'l':
		start_pos.y += 0.5f;

		if (a_soln) {

			arm_angles = analytical_soln(start_pos);
		}

		else if (CCD) {

			CCD();
		}

		break;

	case 'k':
		start_pos.y -= 0.5f;

		if (a_soln) {

			arm_angles = analytical_soln(start_pos);
		}

		else if (CCD) {

			CCD();
		}

		break;

	case 'f':
		//arm_angles = analytical_soln(start_pos);
		a_soln = true;
		ccd = false;
		break;

	case 'c':
		//CCD();
		a_soln = false;
		ccd = true;
		break;

	}
}


int main(int argc, char** argv) {

	// Set up the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(width, height);
	glutCreateWindow("Hello Triangle");

	// Tell glut where the display function is
	glutDisplayFunc(display);
	glutIdleFunc(updateScene);
	glutKeyboardFunc(keypress);
	//glutSpecialFunc(specialKeys);

	// A call to glewInit() must be done after glut is initialized!
	GLenum res = glewInit();
	// Check for any errors
	if (res != GLEW_OK) {
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
		return 1;
	}
	// Set up your objects and shaders
	init();
	// Begin infinite event loop
	glutMainLoop();
	return 0;
}
