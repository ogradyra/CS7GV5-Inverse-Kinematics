#include "ik_maths.h"
#define _USE_MATH_DEFINES

#include <limits>
#include <windows.h>
#include <mmsystem.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <vector> 

// GLM includes
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

using namespace std;

float arm_length = 2.708;
float last_theta = 0;

int effectors = 3;
glm::vec3 links[3];

void setUpLinks() {
	links[0] = glm::vec3(0.0f, 0.0f, 0.0f);
	links[1] = glm::vec3(3.9f, 0.0f, 0.0f);
	links[2] = glm::vec3(7.8f, 0.0f, 0.0f);
}

#pragma region SIMPLE_IK
glm::vec2 analytical_soln(glm::vec3 starting_pos) {

	float d = 0;
	float l1 = 0, l2 = 0;
	float ex = 0, ey = 0;
	float theta_t = 0, theta1 = 0, theta2 = 0;

	float cos2 = 0, sin2 = 0, tan1 = 0;
	float angle1 = 0, angle2 = 0;

	ex = starting_pos.x;
	ey = starting_pos.y;

	l1 = arm_length;
	l2 = arm_length;

	cos2 = ((ex * ex) + (ey * ey) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2);

	if (cos2 >= -1.0 && cos2 <= 1.0) {
		d = glm::sqrt((ex * ex) + (ey * ey));

		theta_t = glm::acos(ex / d);

		theta1 = glm::acos(((l1 * l1) + (ex * ex) + (ey * ey) - (l2 * l2)) / (2 * l1 * d)) + theta_t;
		std::cout << "Theta 1: " << glm::degrees(theta1) << endl;

		//upper_arm_angle = theta1;

		theta2 = 3.14 - (((l1 * l1) + (l2 * l2) - (d * d)) / (2 * l1 * l2));
		std::cout << "Theta 2: " << glm::degrees(theta2) << endl;

		//lower_arm_angle = theta2;
	}

	else {

		theta1 = last_theta;
		theta2 = theta1 - glm::radians(150.0f);
		std::cout << "Other Theta 1: " << glm::degrees(theta1) << endl;
		std::cout << "Other Theta 2: " << glm::degrees(theta2) << endl;
	}

	last_theta = theta1;

	return glm::vec2(glm::degrees(theta1), glm::degrees(theta2));
}

#pragma endregion SIMPLE_IK

#pragma region VEC_FUNCS

double VectorSquaredDistance(glm::vec3* v1, glm::vec3* v2)
{
	return(((v1->x - v2->x) * (v1->x - v2->x)) +
		((v1->y - v2->y) * (v1->y - v2->y)) +
		((v1->z - v2->z) * (v1->z - v2->z)));
}

double VectorSquaredLength(glm::vec3* v)
{
	return((v->x * v->x) + (v->y * v->y) + (v->z * v->z));
}

double VectorLength(glm::vec3* v)
{
	return(sqrt(VectorSquaredLength(v)));
}

void NormalizeVector(glm::vec3* v)
{
	float len = (float)VectorLength(v);
	if (len != 0.0)
	{
		v->x /= len;
		v->y /= len;
		v->z /= len;
	}
}

double DotProduct(glm::vec3* v1, glm::vec3* v2)
{
	return ((v1->x * v2->x) + (v1->y * v2->y) + (v1->z + v2->z));
}

void CrossProduct(glm::vec3* v1, glm::vec3* v2, glm::vec3* result)
{
	result->x = (v1->y * v2->z) - (v1->z * v2->y);
	result->y = (v1->z * v2->x) - (v1->x * v2->z);
	result->z = (v1->x * v2->y) - (v1->y * v2->x);
}

#pragma endregion VEC_FUNCS

#pragma region CCD

glm::vec3 ComputeCCD(int x, int y) {

	glm::vec3 rootPos, curEnd, desiredEnd, targetVector, curVector, crossResult;
	glm::vec3 angles(90.0f, 0.0f, 0.0f);

	double cosAngle, turnAngle, turnDeg;
	int link, tries;
	bool found = false;

	link = effectors - 1;
	tries = 0;

	while (tries < 100 && link >= 0) {
		rootPos.x = links[link].x;
		rootPos.y = links[link].y;
		rootPos.z = links[link].z;

		curEnd.x = links[effectors].x;
		curEnd.y = links[effectors].y;
		curEnd.z = 0.0f;

		desiredEnd.x = float(x);
		desiredEnd.y = float(y);
		desiredEnd.z = 0.0f;

		if (VectorSquaredDistance(&curEnd, &desiredEnd) > 1.0f) {
			curVector.x = curEnd.x - rootPos.x;
			curVector.y = curEnd.y - rootPos.y;
			curVector.z = curEnd.z - rootPos.z;

			targetVector.x = x - rootPos.x;
			targetVector.y = y - rootPos.y;
			targetVector.z = 0.0f;

			NormalizeVector(&curVector);
			NormalizeVector(&targetVector);

			cosAngle = DotProduct(&targetVector, &curVector);

			if (cosAngle < 0.99999) {
				CrossProduct(&targetVector, &curVector, &crossResult);
				if (crossResult.z > 0.0f) {
					turnAngle = glm::acos((float)cosAngle);
					turnDeg = glm::degrees(turnAngle);
					angles[link] -= (float)turnDeg;
				}
				else if (crossResult.z < 0.0f) {
					turnAngle = glm::acos((float)cosAngle);
					turnDeg = glm::degrees(turnAngle);
					angles[link] += (float)turnDeg;
				}
			}
		}

		tries++;
		link--;
	}

	std::cout << "Angles: " << angles.x << ", " << angles.y << ", " << angles.z << endl;	
	return(angles);
}

#pragma endregion CCD
