//==============================================================================
// Pixmap.cpp
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "Pixmap.hpp"

//==============================================================================
// operators for rgb_t
//==============================================================================
bool operator==(const rgb_t &c1, const rgb_t &c2)
{
 if( (c1.b == c2.b) && (c1.g == c2.g) && (c1.r == c2.r) )
  return true;
 return false;
}

bool operator!=(const rgb_t &c1, const rgb_t &c2)
{
 return (!(c1 == c2));
}

std::ostream &operator<< (std::ostream &out, const rgb_t &rgb)
{
 out << (int)(rgb.r) << " " << (int)(rgb.g) << " " << (int)(rgb.b);
 return out;
}

std::istream &operator>> (std::istream &in, rgb_t &rgb)
{
 unsigned short int p[3];
 for(int i = 0; i < 3; i++) {
  while((in.peek()=='\n')||(in.peek()=='\r')||(in.peek()==' '))
   in.ignore(1);
  in >> p[i];
  if(p[i] > 0xff) p[i] = 0xff;
 }
 rgb.r = (uint8_t)(p[0]);
 rgb.g = (uint8_t)(p[1]);
 rgb.b = (uint8_t)(p[2]);
 return in;
}
