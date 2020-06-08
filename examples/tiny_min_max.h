#ifndef TINY_MIN_MAX_H
#define TINY_MIN_MAX_H
template <class T>
inline const T& TinyMin(const T& a, const T& b) {
  return a < b ? a : b;
}

template <class T>
inline const T& TinyMax(const T& a, const T& b) {
  return a > b ? a : b;
}

template <class T>
inline void TinyClamp(T& a, const T& lb, const T& ub) {
  if (a < lb) {
    a = lb;
  } else if (ub < a) {
    a = ub;
  }
}

#endif  // TINY_MIN_MAX_H
