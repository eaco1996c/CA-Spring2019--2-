#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
void getNearestEdge(std::vector<Util::Vector>& simplex, float& distance, Util::Vector& normal, int& index)

{

	distance = FLOAT_MAX;
	int i;
	for (int j = 0; j < simplex.size(); j++)
	{
		
		if (j + 1 == simplex.size())
			i = 0;
		else
			i = j + 1;

		Util::Vector v1 = simplex[i];
		Util::Vector v2 = simplex[j];
		Util::Vector edge = v2 - v1;
		Util::Vector origin1 = v1;

		Util::Vector vn = origin1*(edge*edge) - edge*(edge*origin1); 
		vn = vn / sqrt(pow(vn.x, 2) + pow(vn.y, 2) + pow(vn.z, 2)); 

		float dist = vn*v1; 

		if (dist < distance)
		{

			distance = dist;
			index = i;
			normal = vn;
		}
	}
}

int GetFarthestIndexInDirection(Util::Vector direction, const std::vector<Util::Vector>& ShapeA)

{
	double Max = direction*ShapeA[0];
	int Farthest = 0;

	for (unsigned int i = 1; i < ShapeA.size(); i++)

	{
		double CurrentDot = direction*ShapeA[i];

		if (CurrentDot>Max)
		{
			Max = CurrentDot;
			Farthest = i;
		}
	}
	return Farthest;
}




bool CheckContainsOrigin(Util::Vector& Direction, std::vector<Util::Vector>& simplex)

{

	Util::Vector v1 = simplex.back();
	Util::Vector v2;
	Util::Vector v3;

	Util::Vector v12;
	Util::Vector v13;
    Util::Vector v1fromOri = -1 * v1;


	if (simplex.size() == 3)

	{
		v2 = simplex[1];
		v3 = simplex[0];
		v12 = v2 - v1;
		v13 = v3 - v1;



		Direction = Util::Vector(v12.z, v12.y, -1 * v12.x);

		if (Direction * v3 > 0)

		{
			Direction = Direction * -1;
		}

		if (Direction * v1fromOri > 0)

		{
			simplex.erase(simplex.begin() + 0);

			return false;

		}



		Direction = Util::Vector(v13.z, v13.y, -1 * v13.x);



		if (Direction * v1fromOri > 0)

		{
			simplex.erase(simplex.begin() + 1);
			return false;
		}

		return true;

	}

	else 

	{

		v2 = simplex[0];
		v12 = v2 - v1;

		Direction = Util::Vector(v12.z, v12.y, -1 * v12.x);



		if (Direction * v1fromOri < 0)
		{
			Direction = -1 * Direction;
		}
	}
	return false;

}

bool GJK(const std::vector<Util::Vector>& ShapeA, const std::vector<Util::Vector>& ShapeB, std::vector<Util::Vector>& simplex)

{

	Util::Vector DirectionVector(1, 0, -1);
	
	Util::Vector FirstPoint = ShapeA[GetFarthestIndexInDirection(DirectionVector, ShapeA)];
	Util::Vector newDirection = -1 * DirectionVector;
	Util::Vector SecondPoint = ShapeB[GetFarthestIndexInDirection(newDirection, ShapeB)];
	Util::Vector MDifference = FirstPoint - SecondPoint;

	
	simplex.push_back(MDifference);





	while (true)
	{
		Util::Vector p1 = ShapeA[GetFarthestIndexInDirection(newDirection, ShapeA)];
		Util::Vector new2 = -1 * newDirection;
		Util::Vector p2 = ShapeB[GetFarthestIndexInDirection(new2, ShapeB)];
		 MDifference = p1 - p2;
		
		simplex.push_back(MDifference);

		if (simplex.back() * newDirection <= 0)

		{
			return false;
		}

		else
		{
			if (CheckContainsOrigin(newDirection, simplex))

			{
				return true;
			}
		}
	}
}



bool EPA(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, std::vector<Util::Vector>& simplex, float& penetration_depth, Util::Vector& penetration_vector)

{

	while (true)

	{
		float distance;
		int index;
		Util::Vector normal;

		getNearestEdge(simplex, distance, normal, index);


		Util::Vector FirstPoint = ShapeA[GetFarthestIndexInDirection(normal, ShapeA)];
		Util::Vector newDirection = -1 * normal;
		Util::Vector SecondPoint = ShapeB[GetFarthestIndexInDirection(newDirection, ShapeB)];
		Util::Vector sup = FirstPoint - SecondPoint;

		float d = sup*normal;
		if (d - distance <= 0)
		{
			penetration_vector = normal;
			penetration_depth = distance;
			return true;
		}

		else

		{
			simplex.insert(simplex.begin() + index, sup);

		}

	}

}


bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	
	std::vector<Util::Vector> simplex;
	bool InACollision = GJK(_shapeA, _shapeB, simplex);
	if (InACollision)
	{
		EPA(_shapeA, _shapeB, simplex, return_penetration_depth, return_penetration_vector);
	}
	return InACollision;
}