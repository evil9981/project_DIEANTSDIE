#include "minion.h"

using namespace chai3d;

minion::~minion()
{
}

cVector3d minion::calculateForce(const cVector3d &globalPosition)
{
	cVector3d p_w = globalPosition; //device world frame pos
	cVector3d r_bw = getGlobalPos(); //pos of body orig in world
	cMatrix3d w_R_b = getGlobalRot(); //orintation of body with respect to world frame
	cMatrix3d b_R_w = w_R_b; b_R_w.trans(); //inverse of above

	cVector3d p_b = b_R_w * (p_w - r_bw);

	const double k = 1000.0;
	double len = p_b.length();
	// THIS IS CHEATING
	// Because the radius of the object and the radius of the cursor are both 0.01, we can simply add a 0.01 to the collision detection
	if (len - 0.005 < m_radius)
	{
		cVector3d dir = p_b;
		dir.normalize();
		double mag = k * (m_radius - len + 0.005);
		cVector3d F_b = mag * dir;

		cVector3d F_w = w_R_b * F_b;
		return F_w;
	}
	else
	{
		return cVector3d(0, 0, 0);
	}
}