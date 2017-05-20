#ifndef _SPARSE_SOLUTION_H_
#define _SPARSE_SOLUTION_H_ 

#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <set>
#include <map>

#include "../feature_detect/feature_detect.h"

class SparseSolution
{
public:
	SparseSolution(){}
	~SparseSolution()
	{
		if(_vecs)
		{
			delete []_vecs;
		}
		if(_camera)
		{
			delete []_camera;
		}
		if(_grand)
		{
			delete []_grand;
		}
	}

	SparseSolution(const SparseSolution& rhs)=delete;
	SparseSolution(SparseSolution&& rhs) noexcept=delete;

	SparseSolution& operator= (SparseSolution rhs_copy)=delete;

	void readFile(const std::string& intrinsic_file,
				  const std::string& external_file,
				  const std::string& img_locations);

	void run(const std::string& grand_file);
private:
	std::vector<std::string> _img_filenames;
	double* _vecs;
	double* _camera;
	double* _grand;	
};

#endif