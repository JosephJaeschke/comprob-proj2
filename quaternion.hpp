#ifndef QUATER_H
#define QUATER_H

typedef struct quaternion
{
	PQP_REAL w;
	PQP_REAL x;
	PQP_REAL y;
	PQP_REAL z;
} quat;

namespace Quat
{
	quat random();
	float distance(quat,quat);
	quat slerp(quat,quat,float);
	void toMatrix(quat,PQP_REAL (*)[3][3]);
}
#endif
