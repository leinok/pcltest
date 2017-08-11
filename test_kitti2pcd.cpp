#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>

using namespace pcl;
using namespace std;

namespace po = boost::program_options;

int main(int argc, char** argv) {
    std::string infile, outfile;
    po::options_description desc("Program options");
    desc.add_options()
	("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
	("outfile", po::value<string>(&outfile)->required(), "the file to write the DoN point cloud and normals to")
	;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    fstream input(infile.c_str(), ios::in | ios::binary);
    if (!input.good()) {
	cerr << "could not read file!: " << infile << endl;
  	exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);

    pcl::PointCloud<PointXYZ>::Ptr points (new pcl::PointCloud<PointXYZ>);

    int i = 0;
    for (i = 0; input.good() && !input.eof(); ++i) {
	PointXYZ point;
	input.read((char*)&point.x, 3 * sizeof(float));
	points->push_back(point);
    }
    input.close();

    cout << "Read KITTI point cloud with " << i << " points, writing to " << outfile << endl;

    pcl::PCDWriter writer;
    writer.write<PointXYZ> (outfile, *points, false);

    return 0;
}
