#ifndef UTILS
#define UTILS

#include <array>
#include <functional>
#include <iomanip>
#include <iterator>
#include <ostream>

template <class T, std::size_t N>
std::ostream &operator<<(std::ostream &o, const std::array<T, N> &arr) {
  std::cout << std::setprecision(3); // TODO make configurable
  std::copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, " "));
  return o;
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    os << v[i];
    if (i != v.size() - 1) {
      os << ", ";
    }
  }
  os << "]";
  return os;
}

namespace bcolors {
const std::string HEADER = "\033[95m";
const std::string OKBLUE = "\033[94m";
const std::string OKCYAN = "\033[96m";
const std::string OKGREEN = "\033[92m";
const std::string WARNING = "\033[93m";
const std::string FAIL = "\033[91m";
const std::string ENDC = "\033[0m";
const std::string BOLD = "\033[1m";
const std::string UNDERLINE = "\033[4m";
} // namespace bcolors

class DoStuffOnDestruction {
public:
  DoStuffOnDestruction(std::function<void()> func) : func(func) {}
  ~DoStuffOnDestruction() { func(); }

private:
  std::function<void()> func;
};

#endif