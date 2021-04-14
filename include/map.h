#ifndef MAP_H
#define MAP_H
#include <vector>

using namespace std;

enum CellStatus {FREE, OCCUPIED, UNMAPPED};

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
     * cells: vector of length rows*cols
     */
    Map(int rows, int cols, double cell_length, const vector<CellStatus> &cells);

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
    CellStatus getStatus(double x, double y) const;

    /*
     * getStatus: returns the cell status corresponding to the given index
     * ARGUMENTS
     * index: index into cells vector
     * RETURN: cells[index]
     */
    CellStatus getStatus(int index) const;

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

    /*
     * setStatus: sets status of this cell
     * ARGUMENTS
     * index: index of cell
     * val: new cell status
     */
    void setStatus(int index, CellStatus val);

    /*
     * inMap: checks if the given x, y coordinates are in the map
     * ARGUMENTS
     * x, y: coordinates to check
     * RETURN: true if they're in the map, false if not
     */
    bool inMap(double x, double y);

  private:
    int rows;
    int cols;
    double cell_length;
    double xdim;
    double ydim;
    vector<CellStatus> cells;
};
#endif
