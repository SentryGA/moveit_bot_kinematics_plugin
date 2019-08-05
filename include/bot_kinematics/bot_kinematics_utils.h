#ifndef BOT_UTILITIES_H
#define BOT_UTILITIES_H

#include <cmath>

namespace bot_kinematics
{

template <typename T>
inline bool isValid(const T* qs)
{
  return std::isfinite(qs[0]) && std::isfinite(qs[1]) && std::isfinite(qs[2]);
}

template <typename T>
inline void harmonizeTowardZero(T* qs)
{
  const static T pi = T(M_PI);
  const static T two_pi = T(2.0 * M_PI);

  for (int i = 0; i < 3; i++) // TODO: depends on the number of joint angles
  {
    if (qs[i] > pi) qs[i] -= two_pi;
    else if (qs[i] < -pi) qs[i] += two_pi;
  }
}

}

#endif // BOT_UTILITIES_H
