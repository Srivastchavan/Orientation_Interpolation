#pragma once

#include <vector>
#include <iostream>

 #define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Curve.h"

class Aircraft_Animation
{

public:
	float total_moving_time = 10;
	float t1 = 0.1;
	float t2 = 0.7;

	float anim_Time;
	float anim_Dist;
	bool is_moving;
	bool move_end;

private:
	int interpolant_u;
	float total_Dist;

	glm::mat4 quaternion_mat;
	glm::mat4 m_model_mat;
	Curve* m_animation_curve = nullptr;


	std::vector<glm::vec4> position_entities;
	std::vector<glm::vec3> quaterion_entities;
	std::vector<glm::vec4>::iterator cur_pos;
	std::vector<glm::vec3>::iterator cur_quaternion;

	glm::vec3 position;
	glm::quat quaternion;

	float calculateNormalDist(float time_updated);

	// ---------- Calculate point location
	void generatePositionTable();
	glm::vec3 findNextPosition(float distance);
	glm::vec3 interpolation(float distance, glm::vec4 start, glm::vec4 end);

	// ---------- Calculate point orientation
	void generateOrientationTable();
	glm::quat findNextQuaternion(float distance);
	glm::quat slerp(float length_delta, float length_segment, glm::quat start, glm::quat end);


public:
	Aircraft_Animation();
	~Aircraft_Animation();

	void init();
	void init(Curve* animation_curve);

	void update(float delta_time);

	void reset();
	glm::mat4 get_model_mat() { return m_model_mat; };
};


