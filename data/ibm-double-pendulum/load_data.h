#include <array>
#include <fstream>
#include <vector>

template<typename T>
auto LoadFile(const std::string& filename) {
  std::vector<std::array<T, 6>> out;

  std::ifstream f(filename);

  std::string buffer;
  while (f.good()) {
    out.emplace_back();
    for (int i = 0; i < 6; ++i) {
      f >> out.back(i);
      f.get();
    }
  }

  return out;
}
