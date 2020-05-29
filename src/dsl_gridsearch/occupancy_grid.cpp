#include "dsl_gridsearch/occupancy_grid.h"

using namespace Eigen;

namespace dsl_gridsearch
{
 
OccupancyGrid::OccupancyGrid(double* occupancy_map, int length, int width, int height, Vector3d pmin, Vector3d pmax, double scale) :
  occupancy_map_(occupancy_map),
  length_(length),
  width_(width),
  height_(height),
  scale_(scale),
  pmin_(pmin),
  pmax_(pmax)
{

}

OccupancyGrid::OccupancyGrid(int length, int width, int height, Vector3d pmin, Vector3d pmax, double scale, double default_val) :
  length_(length),
  width_(width),
  height_(height),
  scale_(scale),
  pmin_(pmin),
  pmax_(pmax)
{
  occupancy_map_ = new double[length*width*height];
  for(int i = 0; i < length*width*height; i++)
  {
    occupancy_map_[i] = default_val;
  } 
}

OccupancyGrid::~OccupancyGrid()
{
  delete[] occupancy_map_;
}

double OccupancyGrid::getCost(const Vector3i& gp)
{
  assert(gp(0) >= 0 && gp(0) < length_ && 
         gp(1) >= 0 && gp(1) < width_ && 
         gp(2) >= 0 && gp(2) < height_);
  return occupancy_map_[gp(0) + gp(1)*length_ + gp(2)*length_*width_]; 
}

void OccupancyGrid::setCost(double cost, const Vector3i& gp)
{
  assert(gp(0) >= 0 && gp(0) < length_ && 
         gp(1) >= 0 && gp(1) < width_ && 
         gp(2) >= 0 && gp(2) < height_);
  occupancy_map_[gp(0) + gp(1)*length_ + gp(2)*length_*width_] = cost; 
}

bool OccupancyGrid::isOccupied(const Vector3i& gp)
{
  assert(gp(0) >= 0 && gp(0) < length_ && 
         gp(1) >= 0 && gp(1) < width_ && 
         gp(2) >= 0 && gp(2) < height_);
  return occupancy_map_[gp(0) + gp(1)*length_ + gp(2)*length_*width_] > 0; 
}

bool OccupancyGrid::isOccupied(const Vector3d& p)
{
  int idx = positionToIndex(p);
  assert((idx >= 0) && (idx < length_*width_*height_));

  return occupancy_map_[idx] > 0;
}

bool OccupancyGrid::isInGrid(const Eigen::Vector3i& gp)
{
  return gp(0) >= 0 && gp(0) < length_ &&
         gp(1) >= 0 && gp(1) < width_ && 
         gp(2) >= 0 && gp(2) < height_;
}

void OccupancyGrid::setOccupied(const Vector3d& p, bool set)
{
  int idx = positionToIndex(p);
  assert((idx >= 0) && (idx < length_*width_*height_));

  if(set)
    occupancy_map_[idx] = DSL_OCCUPIED;
  else
    occupancy_map_[idx] = 0;
}

int OccupancyGrid::positionToIndex(const Vector3d& p)
{
  Vector3d offset = (p-pmin_)*scale_;
  int x = (int)floor(offset(0));
  int y = (int)floor(offset(1));
  int z = (int)floor(offset(2));
  return x + y*length_ + z*length_*width_;
}

Vector3i OccupancyGrid::positionToGrid(const Vector3d& p)
{
  Vector3d offset = (p-pmin_)*scale_;
  Vector3i g((int)floor(offset(0)), (int)floor(offset(1)),(int)floor(offset(2)));
  return g;
}

Vector3d OccupancyGrid::positionToDslPosition(const Vector3d& p)
{
  return p - getPmin();
}

Vector3d OccupancyGrid::gridToDslPosition(const Vector3i& p)
{
  return gridToPosition(p) - getPmin();
}

Vector3d OccupancyGrid::gridToPosition(const Vector3i& gp)
{
  double xmin = this->getPmin()(0);
  double ymin = this->getPmin()(1);
  double zmin = this->getPmin()(2);
  double offset = 1.0/(2*scale_);
  return Vector3d(gp(0)/scale_ + offset + xmin,  
                  gp(1)/scale_ + offset + ymin, 
                  gp(2)/scale_ + offset + zmin);
}

int OccupancyGrid::getLength()
{
  return length_;
}

int OccupancyGrid::getWidth()
{
  return width_;
}

int OccupancyGrid::getHeight()
{
  return height_;
}
  
double OccupancyGrid::getScale()
{
  return scale_;
}

double* OccupancyGrid::getOccupancyMap()
{
  return occupancy_map_;
}

Vector3d OccupancyGrid::getPmin()
{
  return pmin_;
}

Vector3d OccupancyGrid::getPmax()
{
  return pmax_;
}

void OccupancyGrid::mergeGrid(OccupancyGrid* ogrid)
{
  int new_length = length_;
  int new_width = width_;
  int new_height = height_;
  Vector3d new_pmax = pmax_;
  Vector3d new_pmin = pmin_;

  Vector3i min_gpos = positionToGrid(ogrid->getPmin());
  Vector3i max_gpos = positionToGrid(ogrid->getPmax());

  if(min_gpos(0) < 0)
  {
    new_length -= min_gpos(0);
    new_pmin(0) = min_gpos(0);
  }
  if(min_gpos(1) < 0)
  {
    new_width -= min_gpos(1);
    new_pmin(1) = min_gpos(1);
  }
  if(min_gpos(2) < 0)
  {
    new_height -= min_gpos(2);
    new_pmin(2) = min_gpos(2);
  }
  if(max_gpos(0) >= length_)
  {
    new_length += max_gpos(0) - (length_-1);
    new_pmax(0) = max_gpos(0);
  }
  if(max_gpos(1) >= width_)
  {
    new_width += max_gpos(1) - (width_-1);
    new_pmax(1) = max_gpos(1);
  }
  if(max_gpos(2) >= height_)
  {
    new_height += max_gpos(2) - (height_-1);
    new_pmax(2) = max_gpos(2);
  }

  double* new_occupancy_map = new double[new_length*new_width*new_height];
  for(int i = 0; i < new_length*new_width*new_height; i++)
  {
    new_occupancy_map[i] = 0;
  }

  // Merge given grid
  int mlength = ogrid->getLength();
  int mwidth = ogrid->getWidth();
  int mheight = ogrid->getHeight();
  double mscale = ogrid->getScale();
  double offset = 1.0/(2.*mscale);
  for(int x = 0; x < mlength; x++)
  {
    for(int y = 0; y < mwidth; y++)
    {
      for(int z = 0; z < mheight; z++)
      {
        int idx = x + y*mlength + z*mlength*mwidth;
        double cost = ogrid->getOccupancyMap()[idx];
        Eigen::Vector3d p(x/mscale + offset + ogrid->getPmin()(0),
                          y/mscale + offset + ogrid->getPmin()(1),
                          z/mscale + offset + ogrid->getPmin()(2));

        Vector3d new_gp = (p-new_pmin)*scale_;
        int new_x = (int)floor(new_gp(0));
        int new_y = (int)floor(new_gp(1));
        int new_z = (int)floor(new_gp(2));
        int new_idx = new_x + new_y*new_length + new_z*new_length*new_width;
        new_occupancy_map[new_idx] = cost;
      }
    }
  }

  // Add old grid
  offset = 1.0/(2.*scale_);
  for(int x = 0; x < length_; x++)
  {
    for(int y = 0; y < width_; y++)
    {
      for(int z = 0; z < height_; z++)
      {
        int idx = x + y*length_ + z*length_*width_;
        double cost = this->getOccupancyMap()[idx];
        Eigen::Vector3d p(x/scale_ + offset + this->getPmin()(0),
                          y/scale_ + offset + this->getPmin()(1),
                          z/scale_ + offset + this->getPmin()(2));

        Vector3d new_gp = (p-new_pmin)*scale_;
        int new_x = (int)floor(new_gp(0));
        int new_y = (int)floor(new_gp(1));
        int new_z = (int)floor(new_gp(2));
        int new_idx = new_x + new_y*new_length + new_z*new_length*new_width;
        new_occupancy_map[new_idx] = cost;
      }
    }
  }

  length_ = new_length;
  width_ = new_width;
  height_ = new_height;
  pmin_ = new_pmin;
  pmax_ = new_pmax;
  delete[] occupancy_map_;
  occupancy_map_ = new_occupancy_map;
}

} // namespace
