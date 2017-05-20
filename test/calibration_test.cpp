#include <iostream>
#include <string>
#include <vector>

#include "../sfm/sfm.h"

int main(int argc, char const *argv[])
{
	CalibrationSolution solution;
	solution.readFile("img_locations.txt","gcp.txt");
	solution.run();
	return 0;
}