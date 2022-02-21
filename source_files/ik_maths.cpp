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

#pragma region IK
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

#pragma endregion IK

