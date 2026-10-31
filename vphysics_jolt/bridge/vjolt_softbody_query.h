#pragma once

#include <vjolt/softbody_tools.h>

namespace JPH { class Body; }

bool VJoltTraceSoftBody( const JPH::Body &body, bool closed,
	const VJoltSoftBodyRay &ray, VJoltSoftBodyHit &hit );
