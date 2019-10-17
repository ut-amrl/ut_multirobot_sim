#ifndef UTILITY
#define UTILITY

#include <functional>

template <class T>
struct lessPtr : public std::binary_function<T, T, bool> {
      bool operator() (const T& x, const T& y) const
      {
          return *x < *y;
      };
};

#endif
