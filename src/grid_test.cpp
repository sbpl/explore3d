#include <iostream>
#include "Grid.h"

int main(int argc, char* argv[])
{
    // Create a 2x3 grid filled with 0's

    const int dims = 2;
    au::Grid<dims, int> grid;

    int size_x = 2;
    int size_y = 3;
    grid.resize(size_x, size_y);

    grid.assign(0);

    for (auto git = grid.begin(); git != grid.end(); ++git) {
        std::cout << *git << std::endl;
    }

    std::cout << std::endl;

    // Fill the diagonal with 1's

    for (int i = 0; i < std::min(size_x, size_y); ++i) {
        grid(i, i) = 1;
    }

    for (auto git = grid.begin(); git != grid.end(); ++git) {
        std::cout << *git << std::endl;
    }

    std::cout << std::endl;

    // Iterate over the top-left 2x2 corner

    typedef au::Grid<dims, int>::GridIndex GridIndex;

    GridIndex begin(0, 0);
    GridIndex end(1, 1);
    for (auto git = grid.grid_begin(begin, end); git != grid.grid_end(begin, end); ++git) {
        std::cout << *git << std::endl;
    }

    return 0;
}
