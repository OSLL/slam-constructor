#ifndef SLAM_CTOR_TESTS_DIRECTIONS
#define SLAM_CTOR_TESTS_DIRECTIONS

class Directions {
private:
  constexpr static int Left_Id  = 1 << 0;
  constexpr static int Right_Id = 1 << 1;
  constexpr static int Top_Id    = 1 << 2;
  constexpr static int Bot_Id  = 1 << 3;
public:
  Directions& set_left()  { _data |= Left_Id; return *this; }
  Directions& set_right() { _data |= Right_Id; return *this; }
  Directions& set_top()   { _data |= Top_Id; return *this; }
  Directions& set_bot()   { _data |= Bot_Id; return *this; }

  bool left() const  { return _data & Left_Id; }
  bool right() const { return _data & Right_Id; }
  bool top() const   { return _data & Top_Id; }
  bool bot() const   { return _data & Bot_Id; }
  int horz() const {
    if (left() ^ right()) { return left() ? -1 : 1; }
    return 0;
  }
  int vert() const {
    if (bot() ^ top()) { return bot() ? -1 : 1; }
    return 0;
  }

private:
  int _data = 0;
};

#endif
