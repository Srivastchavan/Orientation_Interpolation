#include "Aircraft_Animation.h"
#include <glm/gtx/quaternion.hpp>
#include "glm/ext.hpp"

/*****************************************************************************
        * Written by Srivastchavan Rengarajan starting October 08, 2020.
******************************************************************************/

Aircraft_Animation::Aircraft_Animation()
{
	this->m_model_mat = glm::mat4(1.0f);
}


Aircraft_Animation::~Aircraft_Animation()
{
}

void Aircraft_Animation::init()
{
	reset();
}

void Aircraft_Animation::init(Curve* animation_curve)
{
	m_animation_curve = animation_curve;

    interpolant_u = m_animation_curve->num_points_per_segment;
    reset();
}

void Aircraft_Animation::reset()
{
    /*m_model_mat = glm::mat4(1.0f);
    if (m_animation_curve != nullptr && m_animation_curve->control_points_pos.size() > 0)
    {
        m_model_mat = glm::translate(m_model_mat, m_animation_curve->control_points_pos[0]);
    }*/


    is_moving = false;
    move_end = true;

    anim_Time = 0.0;
    anim_Dist = 0.0;

    cur_pos = position_entities.begin();
    cur_quaternion = quaterion_entities.begin();


    position = m_animation_curve->control_points_pos[0];
    quaternion = m_animation_curve->control_points_quaternion[0];
    quaternion_mat = m_animation_curve->createRotMatrixFromQuaternion(quaternion, position);
    m_model_mat = quaternion_mat * glm::mat4(1.0f);
}


void Aircraft_Animation::update(float delta_time)
{
    float normalizedTime = 0.0;
    float normalizedDist = 0.0;

    if (m_animation_curve->curve_on == true && m_animation_curve->curve_points_pos.empty() == false && position_entities.empty() == true && quaterion_entities.empty() == true) {
        generatePositionTable();
        generateOrientationTable();
        cur_pos = position_entities.begin();
        cur_quaternion = quaterion_entities.begin();
        total_Dist = position_entities[position_entities.size() - 1][3];
    }
    else if (is_moving == true && position_entities.empty() == false && quaterion_entities.empty() == false) {
        anim_Time = anim_Time + delta_time;
        normalizedTime = anim_Time / total_moving_time;
        normalizedDist = calculateNormalDist(normalizedTime);
        anim_Dist = total_Dist * normalizedDist;

        if (anim_Dist == 0.0 || anim_Dist >= total_Dist) {
            position = m_animation_curve->control_points_pos[0];
            quaternion = m_animation_curve->control_points_quaternion[0];
        }
        else {
            position = findNextPosition(anim_Dist);
            quaternion = findNextQuaternion(anim_Dist);
        }

        quaternion_mat = m_animation_curve->createRotMatrixFromQuaternion(quaternion, position);
        m_model_mat = quaternion_mat * glm::mat4(1.0f);
    }
    else if (is_moving == false && anim_Dist >= total_Dist) {
        reset();
    }

}

glm::quat Aircraft_Animation::slerp(float delta_len, float segment_len, glm::quat start, glm::quat end)
{
    float ratio = delta_len / segment_len;
    glm::quat ipl_quat = glm::slerp(start, end, ratio);

    return ipl_quat;
}

glm::vec3 Aircraft_Animation::interpolation(float distance, glm::vec4 start, glm::vec4 end)
{
    float total_len = end[3] - start[3];
    float delta_len = distance - start[3];
    float ratio = delta_len / total_len;
    glm::vec3 ipl_pt = { (start[0] + ratio * (end[0] - start[0])),
                         (start[1] + ratio * (end[1] - start[1])),
                         (start[2] + ratio * (end[2] - start[2])) };

    return ipl_pt;
}

float Aircraft_Animation::calculateNormalDist(float time_updated)
{
    float vel;
    float dist;
    float v0 = 2 / (1 - t1 + t2);

    if (time_updated > 0 && time_updated <= t1) {
        vel = v0 * (time_updated / t1);
        dist = (v0 * time_updated * time_updated) / (2 * t1);
    }
    else if (time_updated > t1&& time_updated <= t2) {
        vel = v0;
        dist = (v0 * t1) / 2 + v0 * (time_updated - t1);
    }
    else if (time_updated > t2&& time_updated <= 1) {
        vel = v0 * (1 - (time_updated - t2) / (1 - t2));
        dist = 1 - vel * (1 - time_updated) / 2;
    }
    else if (time_updated > 1) {
        vel = 0;
        dist = 1;
    }
    else {
        vel = 0;
        dist = 0;
    }

    return dist;
}


glm::vec3 Aircraft_Animation::findNextPosition(float distance)
{
    glm::vec3 pos;

    if (distance == (*cur_pos)[3]) {
        pos = { (*cur_pos)[0], (*cur_pos)[1], (*cur_pos)[2] };
    }
    else if (distance < (*cur_pos)[3]) {
        if (cur_pos == position_entities.begin()) {
            pos = interpolation(distance, glm::vec4(0.0, 8.5, -2.0, 0.0), *cur_pos);
        }
        else {
            pos = interpolation(distance, *(cur_pos - 1), *cur_pos);
        }
    }
    else {
        cur_pos = cur_pos + 1;
        pos = findNextPosition(distance);
    }

    return pos;
}

glm::quat Aircraft_Animation::findNextQuaternion(float distance)
{
    int start;
    int end;
    float seg_dist;
    float delta_dist;

    glm::quat orientation;

    if (distance > (*cur_quaternion)[1]) {
        cur_quaternion = cur_quaternion + 1;
        orientation = findNextQuaternion(distance);
    }
    else {
        if ((*cur_quaternion)[2] == (*cur_quaternion)[0] * interpolant_u) {
            start = (int)(*cur_quaternion)[0] - 1;
            end = (int)(*cur_quaternion)[0];
        }
        else {
            start = (int)(*cur_quaternion)[0];
            end = (int)(*cur_quaternion)[0] + 1;
        }

        seg_dist = quaterion_entities[end * interpolant_u][1] - quaterion_entities[start * interpolant_u][1];
        delta_dist = distance - quaterion_entities[start * interpolant_u][1];

        orientation = slerp(delta_dist, seg_dist, m_animation_curve->control_points_quaternion[start],m_animation_curve->control_points_quaternion[(end % 8)]);
    }

    return orientation;
}

void Aircraft_Animation::generatePositionTable()
{
    float piece_length;
    float arc_length = 0;
    std::vector<glm::vec3> curve_points = m_animation_curve->curve_points_pos;
    std::vector<vec3>::iterator itr;

    for (itr = curve_points.begin(); itr != curve_points.end(); itr++) {

        if (itr == curve_points.begin()) {
            arc_length = 0;
        }
        else {
            piece_length = sqrt(pow((*(itr))[0] - (*(itr - 1))[0], 2) +
                                pow((*(itr))[1] - (*(itr - 1))[1], 2) +
                                pow((*(itr))[2] - (*(itr - 1))[2], 2) * 1.0);
            arc_length = arc_length + piece_length;
            position_entities.push_back({ *itr, arc_length });
        }
    }
}

void Aircraft_Animation::generateOrientationTable()
{
    int point_index;
    int segment;
    int u_index;
    float piece_length;
    float partial_length = 0;
    float total_length = 0;
    std::vector<glm::vec3> curve_points = m_animation_curve->curve_points_pos;
    std::vector<vec3>::iterator itr;

    for (itr = curve_points.begin(); itr != curve_points.end(); itr++) {

        point_index = (int)std::distance(curve_points.begin(), itr);
        segment = point_index / interpolant_u;
        u_index = segment * interpolant_u + 1;

        if (itr == curve_points.begin()) {
            total_length = 0;
            piece_length = 0;
        }
        else {
            piece_length = sqrt(pow((*itr)[0] - (*(itr - 1))[0], 2) + 
			                    pow((*itr)[1] - (*(itr - 1))[1], 2) + 
								pow((*itr)[2] - (*(itr - 1))[2], 2) * 1.0);
            total_length = total_length + piece_length;
        }

        quaterion_entities.push_back({ segment, total_length, point_index });
    }
}




