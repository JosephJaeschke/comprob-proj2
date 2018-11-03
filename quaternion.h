#ifndef QUATER_H
#define QUATER_H

typedef struct quaternion
{
	float w;
	float x;
	float y;
	float z;
} quat;

namespace Quat
{
	quat random();
	float dist(quat,quat);
	quat slerp(quat,quat);
	float** toMatrix(quat);
}
#endif
