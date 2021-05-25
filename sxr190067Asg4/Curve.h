#pragma once
#include <vector>
#include <iostream>

 #define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

using namespace glm;
using namespace std;

class Curve
{
public:

	Curve();
	~Curve();
	bool curve_on = true;
	
	void init();
	void calculate_curve();
	vector<vec3> catmull_rom(const vec3& P0, const vec3& P1, const vec3& P2, const vec3& P3);

	mat4 createRotMatrixFromQuaternion(quat quaternion, vec3 position);

	mat4 rotateCubes(mat4 cube_mat, int index);
	
public:
	float tau = 0.5; // Coefficient for catmull-rom spline
	int num_points_per_segment = 200;

	std::vector<glm::vec3> control_points_pos;
	std::vector<glm::vec3> curve_points_pos;
	std::vector<glm::quat> control_points_quaternion;
};