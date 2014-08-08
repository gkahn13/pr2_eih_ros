#ifndef OCCLUDED_REGION_FINDER_H_INCLUDED
#define OCCLUDED_REGION_FINDER_H_INCLUDED

#include <tsdf_converter.h>
#include <cluster_extraction.h>
#include <cluster_projection.h>

namespace occluded_region_finder {

void find_occluded_regions(std::vector<float> tsdf_distances, std::vector<short> tsdf_weights, Eigen::Matrix4d transformation_matrix, bool saving, std::string outfile);

}


#endif
