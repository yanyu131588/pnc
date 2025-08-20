#include "inflationLayer.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>


InflationLayer::InflationLayer()
: inflation_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  inflate_unknown_(false),
  inflate_around_unknown_(false),
  cell_inflation_radius_(0),
  cached_cell_inflation_radius_(0),
  resolution_(0),
  cache_length_(0),
  last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
  ,initialized_(false)
{
}

InflationLayer::~InflationLayer()
{
  inflation_cells_.clear();
  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_reinflation_ = false;

  distance_matrix_.clear();
}

void InflationLayer::onInitialize()
{
  if (!initialized_)
  {
    initialized_ = true;

    inflation_radius_ = INFLATION_RADIUS;
    cost_scaling_factor_ = COST_SCALING_FACTOR;

    enabled_ = true;
    inflate_unknown_ = false;
    inflate_around_unknown_ = false;

    seen_.clear();
    cached_distances_.clear();
    cached_costs_.clear();
    need_reinflation_ = false;

    matchSize();
  }    
}

void InflationLayer::matchSize()
{
    Costmap2D * costmap = layered_costmap_->getCostmap(); 
    resolution_ = costmap->getResolution();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    computeCaches();
    seen_ = std::vector<bool>(costmap->getSizeInCellsX() * costmap->getSizeInCellsY(), false);
}

void InflationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
    if (need_reinflation_) {
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;

      *min_x = std::numeric_limits<double>::lowest();
      *min_y = std::numeric_limits<double>::lowest();
      *max_x = std::numeric_limits<double>::max();
      *max_y = std::numeric_limits<double>::max();
      need_reinflation_ = false;
    } else {
      double tmp_min_x = last_min_x_;
      double tmp_min_y = last_min_y_;
      double tmp_max_x = last_max_x_;
      double tmp_max_y = last_max_y_;
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
      *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
      *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
      *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
    }
}

void InflationLayer::onFootprintChanged()
{
    inscribed_radius_ = layered_costmap_->getInscribedRadius();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    computeCaches();
    need_reinflation_ = true;
}

void InflationLayer::updateCosts(
  Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
    if (!initialized_) {
      return;
    }

    if (!enabled_ || (cell_inflation_radius_ == 0)) {
        return;
    }

    unsigned char * master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

    if (seen_.size() != size_x * size_y) {
      seen_ = std::vector<bool>(size_x * size_y, false);
    }

    std::fill(begin(seen_), end(seen_), false);

    const int base_min_i = min_i;
    const int base_min_j = min_j;
    const int base_max_i = max_i;
    const int base_max_j = max_j;

    min_i -= static_cast<int>(cell_inflation_radius_);
    min_j -= static_cast<int>(cell_inflation_radius_);
    max_i += static_cast<int>(cell_inflation_radius_);
    max_j += static_cast<int>(cell_inflation_radius_);
 
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);

    auto & obs_bin = inflation_cells_[0];
    obs_bin.reserve(200);
    for (int j = min_j; j < max_j; j++) {
      for (int i = min_i; i < max_i; i++) {
        int index = static_cast<int>(master_grid.getIndex(i, j));
        unsigned char cost = master_array[index];

        if (cost == LETHAL_OBSTACLE || (inflate_around_unknown_ && cost == NO_INFORMATION)) {
          obs_bin.emplace_back(i, j, i, j);
        }
      }
    }

    for (auto & dist_bin : inflation_cells_) {
      dist_bin.reserve(200);

      for (std::size_t i = 0; i < dist_bin.size(); ++i) {
        const CellData & cell = dist_bin[i];
  
        unsigned int mx = cell.x_;
        unsigned int my = cell.y_;
       
        unsigned int sx = cell.src_x_;
        unsigned int sy = cell.src_y_;
        
        unsigned int index = master_grid.getIndex(mx, my);

        if (seen_[index]) {
          continue;
        }

        seen_[index] = true;

        unsigned char cost = costLookup(mx, my, sx, sy);
        unsigned char old_cost = master_array[index];

        if (static_cast<int>(mx) >= base_min_i &&
          static_cast<int>(my) >= base_min_j &&
          static_cast<int>(mx) < base_max_i &&
          static_cast<int>(my) < base_max_j)
        {
          if (old_cost == NO_INFORMATION &&
            (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
          {
            master_array[index] = cost;
          } else {
            master_array[index] = std::max(old_cost, cost);
          }
        }

        if (mx > 0) {
          enqueue(index - 1, mx - 1, my, sx, sy);
        }
        if (my > 0) {
          enqueue(index - size_x, mx, my - 1, sx, sy);
        }
        if (mx < size_x - 1) {
          enqueue(index + 1, mx + 1, my, sx, sy);
        }
        if (my < size_y - 1) {
          enqueue(index + size_x, mx, my + 1, sx, sy); 
        }
      }

      dist_bin = std::vector<CellData>();
    }
}

void InflationLayer::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index]) {
    double distance = distanceLookup(mx, my, src_x, src_y);

    if (distance > cell_inflation_radius_) {
      return;
    }

    const unsigned int r = cell_inflation_radius_ + 2;

    const auto dist = distance_matrix_[mx - src_x + r][my - src_y + r];
    inflation_cells_[dist].emplace_back(mx, my, src_x, src_y);
  }
}

void InflationLayer::computeCaches()
{
    if (cell_inflation_radius_ == 0) {
        return;
      }
    
    cache_length_ = cell_inflation_radius_ + 2;

    if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
      cached_costs_.resize(cache_length_ * cache_length_);
      cached_distances_.resize(cache_length_ * cache_length_);

      for (unsigned int i = 0; i < cache_length_; ++i) {
        for (unsigned int j = 0; j < cache_length_; ++j) {
          
          cached_distances_[i * cache_length_ + j] = hypot(i, j);
        }
      }

      cached_cell_inflation_radius_ = cell_inflation_radius_;
    }

    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
 
        cached_costs_[i * cache_length_ + j] = computeCost(cached_distances_[i * cache_length_ + j]);
      }
    }
    
    int max_dist = generateIntegerDistances();
    inflation_cells_.clear();
    inflation_cells_.resize(max_dist + 1);
}

int InflationLayer::generateIntegerDistances()
{
    const int r = cell_inflation_radius_ + 2;
    const int size = r * 2 + 1;

    std::vector<std::pair<int, int>> points;

    for (int y = -r; y <= r; y++) {
      for (int x = -r; x <= r; x++) {
        if (x * x + y * y <= r * r) {
          points.emplace_back(x, y);
        }
      }
    }

    std::sort(
      points.begin(), points.end(),
      [](const std::pair<int, int> & a, const std::pair<int, int> & b) -> bool {
        return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
      }
    );

    std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
    std::pair<int, int> last = {0, 0};
    int level = 0;
    for (auto const & p : points) {
      if (p.first * p.first + p.second * p.second !=
        last.first * last.first + last.second * last.second)
      {
        level++;
      }

      distance_matrix[p.first + r][p.second + r] = level;
      last = p;
    }

    distance_matrix_ = distance_matrix;
    return level;
}

