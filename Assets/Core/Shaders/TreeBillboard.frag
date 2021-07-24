/* Copyright 2012-2020 Matthew Reid
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#version 330 core
#pragma import_defines ( CAST_SHADOWS )
#pragma import_defines ( ENABLE_SHADOWS )

#include "AtmosphericScatteringWithClouds.h"
#include "DepthPrecision.h"
#include "Trees.h"
#include "Brdfs/Lambert.h"
#include "Shadows/Shadows.h"

in vec2 texCoord;
in float perTreeUnitRandom;
in vec3 normal;
in float logZ;
in AtmosphericScattering scattering;
in vec3 shadowTexCoord;
in float fragmentViewDistance;
out vec4 color;

uniform sampler2D albedoSampler;
uniform vec3 lightDirection;
uniform vec3 ambientLightColor;

void main()
{
	color = texture(albedoSampler, texCoord.xy);

	if (color.a < 0.5)
		discard;

#ifdef CAST_SHADOWS
	return;
#endif

#ifdef ENABLE_SHADOWS
	float dotLN = 1.0;
	float lightVisibility = sampleShadowsAtTexCoord(shadowTexCoord.xy, shadowTexCoord.z-0.0004, dotLN, fragmentViewDistance);
#else
	float lightVisibility = 1.0;
#endif
	
	color.rgb = randomizeColor(color.rgb, perTreeUnitRandom);
	color.rgb *= calcLambertDirectionalLight(lightDirection, normal) * scattering.sunIrradiance * lightVisibility
			   + calcLambertAmbientLight(normal, calcGroundIrradiance(scattering.sunIrradiance, scattering.skyIrradiance), scattering.skyIrradiance) + ambientLightColor;	
	color.rgb = color.rgb * scattering.transmittance + scattering.skyRadianceToPoint;

	gl_FragDepth = logarithmicZ_fragmentShader(logZ);
}