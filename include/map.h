#ifndef MAP_H
#define MAP_H
#include <vector>

using namespace std;

/*
 * Note that this map begins at (x, y) = (0, 0). Calling functions on negative
 * coordinates results in undefined behavior
 */
class Map {
  public:
    /*
     * CONSTRUCTOR: initializes map with given parameters
     * ARGUMENTS
     * rows, cols: map size
     * cell_length: the map is a grid of square cells. What is each cell's side length?
     * cells: vector of length rows*cols, where a 1 indicates that a cell is occupied by
     * an obstacle, 0 indicates that a cell is free, and 2 indicates that we don't know
     * whether the cell is occupied or free
     */
    Map(int rows, int cols, double cell_length, const vector<int> &cells);

    /*
     * getMapIdx: returns the index into the cells vector corresponding to the given 
     * coordinate
     * ARGUMENTS
     * x, y: coordinate
     * RETURN: index in cells vector corresponding to a cell containing the given coordinates
     */
    int getMapIdx(double x, double y) const;

    /*
     * getStatus: returns the cell status corresponding to the given coordinates
     * ARGUMENTS
     * x, y: coordinate
     * RETURN: cells[get_map_idx(x, y)]
     */
    int getStatus(double x, double y) const;

    /*
     * getStatus: returns the cell status corresponding to the given index
     * ARGUMENTS
     * index: index into cells vector
     * RETURN: cells[index]
     */
    int getStatus(int index) const;

    /*
     * getCellLength: the map is a grid of square cells. What is each cell's side length?
     * RETURN: length of each cell
     */
    double getCellLength() const;

    /*
     * getCellExtent: get extent of the given cell
     * ARGUMENTS
     * left: populated with lowest x coordinate of cell
     * right: populated with highest x coordinate of cell
     * bottom: populated with lowest y coordinate of cell
     * top: populated with highest y coordinate of cell
     * index: index of the cell
     */
    void getCellExtent(double &left, double &right, double &bottom, double &top, int index) const;

    /*
     * getCellPos: get position of the bottom-left corner of the cell
     * ARGUMENTS
     * left: populated with lowest x coordinate of cell
     * right: populated with highest x coordinate of cell
     * index: index of the cell
     */
    void getCellPos(double &left, double &bottom, int index) const;

  private:
    int rows;
    int cols;
    double cell_length;
    double xdim;
    double ydim;
    vector<int> cells;
};
#endif
