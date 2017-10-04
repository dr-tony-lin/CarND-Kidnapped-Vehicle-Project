#ifndef _MAP_PARTITION2D_H_
#define _MAP_PARTITION2D_H_
#include <math.h>
#include <vector>
#include <tuple>
#include <iostream>
#include <assert.h>
#include "../utils/helper_functions.h"
#include "Map.h"

template<typename T> class Partition2D {
  private:
    // The world's bounding box
    float world_x0;
    float world_y0;
    float world_x1;
    float world_y1;

    // The maximum level of cell to search, each level will increse search range by 2, so we have maximum of
    // 1x1, 3x3, 5x5, 7x7 cells to search for level 1, 2, 3, 4 respectively
    int search_levels;

    // The dimension of the partition, the world is partitioned into dim_x by dim_y cells of equal size
    int dim_x, dim_y;

    float cell_size;

    // The partition structure
    std::vector<std::vector<T*>*> cells;

  protected:
    int cellIndex(int x, int y) const {
      return x + y * dim_x;
    }

  public:
    Partition2D() {}
    
    ~Partition2D() {
      cells.clear();
    }

    /**
     * Initialize partition
     * @param x0 the left coordinate of the world
     * @param y0 the lower coordinate of the world
     * @param x1 the right coordinate of the world
     * @param y1 the upper coordinate of the world
     * @param cell_size the width and height of cells
     * @param max_dist the maximum distance to search
     */ 
    void initialize(float x0, float y0, float x1, float y1, float cell_size, float max_dist) {
      world_x0 = x0;
      world_y0 = y0;
      world_x1 = x1;
      world_y1 = y1;
      this->cell_size = cell_size;
      assert(x0 < x1 && y0 < y1);
      assert(cell_size > 0);
      this->dim_x = std::ceil((x1 - x0) / cell_size);
      this->dim_y = std::ceil((y1 - y0) / cell_size);
      search_levels = max_dist / cell_size + 0.5;
      cells.clear();
      cells.resize(dim_x * dim_y, NULL);
    }

    /**
     * Clear the partition
     */ 
    void clear() {
      for (int i = 0; i < cells.size(); i++) {
        if (cells[i]) {
          delete cells[i];
        }
      }
      cells.clear();
    }

    /**
     * Find the nearest object to the given coordinate
     * @param x the x coordinate
     * @param y the y coordinate
     * @param max_dist the distance threshold, after that, the search will give up
     * @return pointer to the closest object or null if none is found
     */  
    std::tuple<T*, double, int> findNearest(double x, double y) const {
      int cx0 = (x - world_x0) / cell_size;
      int cy0 = (y - world_y0) / cell_size;
      int cx1 = cx0 + 1;
      int cy1 = cy0 + 1;
      int level = 0;
      int searched = 0;
      T* found = NULL;
      double min_dist = 1.E20; // big enough
      while(!found && level++ < search_levels) {
        cx0 = std::max(0, cx0);
        cy0 = std::max(0, cy0);
        cx1 = std::min(dim_x, cx1);
        cy1 = std::min(dim_y, cy1);
        for (int j = cy0; j < cy1; j++) {
          for (int i = cx0; i <cx1; i++) {
            if (i > cx0 && j > cy0 && i < cx1 - 1 && j < cy1 - 1) { // in the previous level
              continue;
            }
            std::vector<T*> *objects = cells[cellIndex(i, j)];
            if (objects) {
              for (typename std::vector<T*>::iterator obj = objects->begin(); obj != objects->end(); obj++) {
                searched++;
                double dis = dist(x, y, (*obj)->x(), (*obj)->y());
                if (min_dist > dis) {
                  min_dist = dis;
                  found = *obj;
                }
              }
            }
          }
        }

        --cx0;
        --cy0;
        ++cx1;
        ++cy1;
      }
      return std::make_tuple(found, found? sqrt(min_dist): -1, searched);
    }

    /** Add a point object. A point object has x and y coordinate, and provides accessor x() and y().
     * @param object pointer to the object
     */ 
    void addPointObject(T *object) {
      int idx_x = (object->x() - world_x0) / cell_size;
      int idx_y = (object->y() - world_y0) / cell_size;
      int index = cellIndex(idx_x, idx_y);
      if (!cells[index]) {
        cells[index] = new std::vector<T*>();
      }
      cells[index]->push_back(object);
    }

    /** Add a point objects. A point object has x and y coordinate, and provides accessor x() and y().
     * @param object pointer to the object
     */ 
    void addPointObjects(std::vector<T> &objects) {
      for (int i = 0; i < objects.size(); i++) {
        addPointObject(&objects[i]);
      }
    }
};

#endif