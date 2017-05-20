#include "../sfm/sfm.h"

#include <string>

int main(int argc, char const *argv[])
{
	SparseSolution solution;
	solution.readFile("intrinsic.txt","external.txt","png_locations.txt");
	solution.run("dst.txt");
	return 0;
}