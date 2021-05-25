#include <glm/gtx/quaternion.hpp>
#include "Curve.h"

using namespace glm;
using namespace std;


/*****************************************************************************
		* Written by Srivastchavan Rengarajan starting October 08, 2020.
******************************************************************************/


float arr[16] = { -0.5, 1.5,-1.5, 0.5,
					 1.0,-2.5, 2.0,-0.5,
					-0.5, 0.0, 0.5, 0.0,
					 0.0, 1.0, 0.0, 0.0 };
Curve::Curve()
{
}

Curve::~Curve()
{
}

void Curve::init()
{
	this->control_points_pos = 
	{
		{ 0.0, 8.5, -2.0 },
		{ -3.0, 11.0, 2.3 },
		{ -6.0, 8.5, -2.5 },
		{ -4.0, 5.5, 2.8 },
		{ 1.0, 2.0, -4.0 },
		{ 4.0, 2.0, 3.0 },
		{ 7.0, 8.0, -2.0 },
		{ 3.0, 10.0, 3.7 }
	};
	//calculate_curve();
	
	this->control_points_quaternion = {
		{0.13964   , 0.0481732 , 0.831429 , 0.541043 , },
		{0.0509038 , -0.033869 , -0.579695, 0.811295 , },
		{-0.502889 , -0.366766 , 0.493961 , 0.592445 , },
		{-0.636    , 0.667177  , -0.175206, 0.198922 , },
		{0.693492  , 0.688833  , -0.152595, -0.108237, },
		{0.752155  , -0.519591 , -0.316988, 0.168866 , },
		{0.542054  , 0.382705  , 0.378416 , 0.646269 , },
		{0.00417342, -0.0208652, -0.584026, 0.810619   }
	};
}

void Curve::calculate_curve()
{
	/*this->curve_points_pos = {
	{ 0.0, 8.5, -2.0 },
	{ -3.0, 11.0, 2.3 },
	{ -6.0, 8.5, -2.5 },
	{ -4.0, 5.5, 2.8 },
	{ 1.0, 2.0, -4.0 },
	{ 4.0, 2.0, 3.0 },
	{ 7.0, 8.0, -2.0 },
	{ 3.0, 10.0, 3.7 }
	};*/


	int control_points_num = control_points_pos.size();
	vec3 P0;
	vec3 P1;
	vec3 P2;
	vec3 P3;
	vector<vec3> segment_Points;
	vector<vec3>::iterator itr;

	for (int i = 0; i < control_points_num; i++) {

		P0 = control_points_pos[((i + 7) % control_points_num)];
		P1 = control_points_pos[i % control_points_num];
		P2 = control_points_pos[((i + 1) % control_points_num)];
		P3 = control_points_pos[((i + 2) % control_points_num)];

		segment_Points = catmull_rom(P0, P1, P2, P3);

		itr = curve_points_pos.end();

		curve_points_pos.insert(itr, segment_Points.begin(), segment_Points.end());
		if (i == control_points_num-1 ) {
			curve_points_pos.push_back(P2);
		}
	}
}

vector<vec3> Curve::catmull_rom(const vec3& P0, const vec3& P1, const vec3& P2, const vec3& P3)
{
	mat4 cubic_mat = make_mat4(arr);
	mat4x3 control_points(P0, P1, P2, P3);
	vec4 uVar;
	vec3 single_point;
	vector<vec3> segment_Points;

	for (int i = 0; i < num_points_per_segment; i++) {
		float u = (1.0 / (float)num_points_per_segment) * (i);
		uVar = vec4(u * u * u, u * u, u, 1);
		single_point = control_points * cubic_mat * uVar;
		segment_Points.push_back(single_point);
	}
	return segment_Points;
}


glm::mat4 Curve::rotateCubes(glm::mat4 cube_mat, int index) {

	mat4 quaternion_mat = createRotMatrixFromQuaternion(control_points_quaternion[index], control_points_pos[index]);

	glm::mat4 rotated_cube_mat = quaternion_mat * cube_mat;

	return rotated_cube_mat;

}

glm::mat4 Curve::createRotMatrixFromQuaternion(glm::quat quaternion, glm::vec3 position) {

	
	float x = quaternion[0];
	float y = quaternion[1];
	float z = quaternion[2];
	float s = quaternion[3];

	float rotMatArray[16] = { (1 - 2 * y * y - 2 * z * z), (2 * x * y + 2 * s * z),     (2 * x * z - 2 * s * y), 0,
							  (2 * x * y - 2 * s * z),     (1 - 2 * x * x - 2 * z * z), (2 * y * z + 2 * s * x), 0,
							  (2 * x * z + 2 * s * y),     (2 * y * z - 2 * s * x),     (1 - 2 * x * x - 2 * y * y), 
										0, position[0], position[1], position[2], 1 };

	mat4 rotMatrix = make_mat4(rotMatArray);
	
	return rotMatrix;
}