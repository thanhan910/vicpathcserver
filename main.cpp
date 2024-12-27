#include <chrono>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

#define RUN_SERVER

#ifdef TEST_QUADTREE
#include "quadtree.h"
#endif
#ifdef TEST_SEARCHGRAPH
#include "searchgraph.h"
#endif
#ifdef RUN_SERVER
#include "server.h"
#endif

int main()
{
#ifdef TEST_QUADTREE
    QuadTree quadtree;
    quadtree.gen_quadtree();
    quadtree.test_quadtree();
#endif

#ifdef TEST_SEARCHGRAPH
    test_searchgraph();
#endif

#ifdef RUN_SERVER
    const std::string address = "0.0.0.0";
    const unsigned short port = 8080;

    try
    {
        Server server(address, port);
        server.run();
    }
    catch (std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
#endif

    return 0;
}

// docker exec -it searchc-dev-container-1 /bin/bash
// g++ search-path-server.cpp -o local-server.exe $(pkg-config --cflags --libs libpqxx libpq libmongocxx-static) -I/vcpkg/installed/x64-linux/include -L/vcpkg/installed/x64-linux/lib && ./local-server.exe
// http://localhost:8080/nearestsegment?x=144.866&y=-37.751275
// http://localhost:8080/searchpath?x1=144.9631&y1=-37.8136&x2=145.0458&y2=-37.8768
// cd /workspace && rm -rf build && mkdir build && cd build && cmake .. && make && ./local-server