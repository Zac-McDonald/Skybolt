/* Copyright 2012-2020 Matthew Reid
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */


#pragma once

#include "Box2.h"
#include "Box3.h"
#include "MathUtility.h"
#include <optional>
#include <vector>

namespace skybolt {

//! @param direction must be normalized
//! @param hitDistance is negative if origin is inside box
//! @returns true if intersection occured. Returns min hitDistance
bool intersectRayAabb(const glm::vec3 &origin, const glm::vec3 &direction, const Box3 &box, float &hitDistance);

//! @param direction must be normalized
//! @param hitDistance is negative if origin is inside box
//! @returns true if intersection occured. Returns min hitDistance
bool intersectRayAabb(const glm::vec2 &origin, const glm::vec2 &direction, const Box2 &box, float &hitDistance);

struct Grid
{
	glm::vec2 origin;
	glm::vec2 cellSize;
	int countX;
	int countY;
};

void intersectRayGrid(const Grid& grid, const glm::vec2 &origin, const glm::vec2 &direction, float length, std::vector<glm::ivec2>& intersectedCells);

//! @param r0 specifies ray origin
//! @param rd specifies normalized ray direction
//! @param s0 specifies sphere center
//! @param sr specifies sphere radius
//! @returns Returns distances to intersections t0 and t1. Sets t0 to -1 if there was no intersection.
std::optional<std::pair<float, float>> intersectRaySphere(const glm::vec3& r0, const glm::vec3& rd, const glm::vec3& s0, float sr);

template <typename T>
float nearestPointOnLine(const T &lineP0, const T &lineP1, const T &point)
{
	T v0 = point - lineP0;
	T v1 = lineP1 - lineP0;

	float v1_v1 = glm::dot(v1, v1);
	if (v1_v1 == 0.0f) // if line has 0 length
		return 0.f;

	float v0_v1 = glm::dot(v0, v1);

	float t = v0_v1 / v1_v1;

	if (t < 0.0f)
		t = 0.0f;
	else if (t > 1.0f)
		t = 1.0f;

	return t;
}

} // namespace skybolt
