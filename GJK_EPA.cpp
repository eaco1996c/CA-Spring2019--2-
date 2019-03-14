#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
void nearestEdge(std::vector<Util::Vector>& simplex, float& distance, Util::Vector& normal, int& index)

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

int farthestIndex(Util::Vector direction, const std::vector<Util::Vector>& ShapeA)

{
	double Max = direction*ShapeA[0];
	int far = 0;

	for (unsigned int i = 1; i < ShapeA.size(); i++)

	{
		double currentDot = direction*ShapeA[i];

		if (Max<currentDot)
		{
			Max = currentDot;
			far = i;
		}
	}
	return far;
}




bool containsOrigin(Util::Vector& Direction, std::vector<Util::Vector>& simplex)

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
	bool collide = true;
	Util::Vector DirectionVector(1, 0, -1);
	
	Util::Vector FirstPoint = ShapeA[farthestIndex(DirectionVector, ShapeA)];
	Util::Vector newDirection = -1 * DirectionVector;
	Util::Vector SecondPoint = ShapeB[farthestIndex(newDirection, ShapeB)];
	Util::Vector MDifference = FirstPoint - SecondPoint;

	
	simplex.push_back(MDifference);





	while (collide==true)
	{
		Util::Vector p1 = ShapeA[farthestIndex(newDirection, ShapeA)];
		Util::Vector new2 = -1 * newDirection;
		Util::Vector p2 = ShapeB[farthestIndex(new2, ShapeB)];
		MDifference = p1 - p2;
		
		simplex.push_back(MDifference);

		if (simplex.back() * newDirection <= 0)

		{
		 return collide = false;
		}

		else
		{
			if (containsOrigin(newDirection, simplex))

			{
			return collide = true;
			}
		}
	}
}



bool EPA(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, std::vector<Util::Vector>& simplex, float& penetration_depth, Util::Vector& penetration_vector)

{
	bool collide = true;
	while (collide=true)

	{
		int index;
		float distance;
		Util::Vector normal;

		nearestEdge(simplex, distance, normal, index);


		Util::Vector p1 = ShapeA[farthestIndex(normal, ShapeA)];
		Util::Vector newDirection = -1 * normal;
		Util::Vector p2= ShapeB[farthestIndex(newDirection, ShapeB)];
		Util::Vector sup = p1 - p2;

		float dist = sup*normal;
		if (dist - distance <= 0)
		{
			
			penetration_depth = distance;
			penetration_vector = normal;
			return collide=true;
		}

		else

		{
			simples.insert(simplex.begin() + index, sup);

		}

	}

}


bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	
	std::vector<Util::Vector> simplex;
	bool collide = GJK(_shapeA, _shapeB, simplex);
	if (collide)
	{
		EPA(_shapeA, _shapeB, simplex, return_penetration_depth, return_penetration_vector);
	}
	return collide;
}