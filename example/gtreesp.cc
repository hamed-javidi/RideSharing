#include <iostream>

#include "gtree.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: gtreesp [gtree]" << std::endl;
    return 1;
  }
  std::cout << "Loading " << argv[1] << "... ";
  GTree::load(argv[1]);
  GTree::G_Tree gt = GTree::get();
  std::cout << "Done" << std::endl;

  while (1) {
    int a, b;
    std::cout << "a: ";
    std::cin >> a;
    std::cout << "b: ";
    std::cin >> b;
    std::cout << gt.search(a, b) << std::endl;
    std::vector<int> route;
    gt.find_path(a,b,route);
    for (const int& i : route)
        std::cout << i << " ";
    std::cout << std::endl;
  }

  return 0;
}
