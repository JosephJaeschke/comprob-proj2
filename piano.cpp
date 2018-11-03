#include <cstdio>
#include "quaternion.h"

int main()
{
	int a;
	for(a=0;a<10;a++)
	{
		quat q1=Quat::random();
		float** mat=Quat::toMatrix(q1);
		int i,j;
		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				printf("%f\t",mat[i][j]);
			}
			printf("\n");
		}
		printf("\n\n");
		delete mat;
	}
	return 0;
}
