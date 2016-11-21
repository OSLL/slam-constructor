#pragma once
#include <vector>
#include <math.h>
#include <memory>
#include <iostream>
#include <map>
#include <string>

#ifdef GL_MODE
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

template <typename T>
inline T& get(std::vector<T>& array, long ind) {
  if (0 <= ind && ind < (long) array.size())
    return array[ind];
  long ind_cut = ind % (long) array.size();
  if (ind_cut < 0)
    ind_cut += array.size();
  return array[ind_cut];
}

template <typename T>
inline const T& get(const std::vector<T>& array, long ind) {
  if (0 <= ind && ind < (long) array.size())
    return array[ind];
  long ind_cut = ind % (long) array.size();
  if (ind_cut < 0)
    ind_cut += array.size();
  return array[ind_cut];
}

template <typename T>
inline T scalar_mul(const std::vector<T>& v1,
                    const std::vector<T>& v2,
                    long ind1 = 0, long ind2 = 0) {
  size_t v1size = v1.size();
  size_t v2size = v2.size();
  if (v1size != v2size) {
    v1size = std::min(v1size,v2size);
  }
  T out = 0;
  for (long i = 0; i < (long)v1size; i++)
    out += get(v1,ind1+i)*get(v2,ind2+i);
  return out;
}

template <typename T>
inline T difference(const std::vector<T>& v1,
                    const std::vector<T>& v2,
                    long ind1 = 0, long ind2 = 0) {
  size_t v1size = v1.size();
  size_t v2size = v2.size();
  if (v1size != v2size) {
    v1size = std::min(v1size,v2size);
  }
  T out = 0;
  for (long i = 0; i < (long)v1size; i++) {
    T curr = std::abs(get(v1, ind1+i) - get(v2, ind2+i));
    out += curr*curr;
  }
  return out;
}

template <typename T>
inline std::shared_ptr<std::vector<T>> shift(const std::vector<T>& v, int shift){
  std::shared_ptr<std::vector<T>> out(new std::vector<T>(v.size()));
  for (size_t i = 0; i < v.size(); i++) {
    get(*out, i-shift) = v[i];
  }
  return out;
}

template<typename T>
inline T my_max(std::vector<T> array) {
  T max_local  = array[0];
  for (size_t i = 1; i < array.size(); i++) {
    if (max_local < array[i])
      max_local = array[i];
  }
  return max_local;
}

template<typename T>
inline T my_min(std::vector<T> array) {
  T min_local  = array[0];
  for (size_t i = 1; i < array.size(); i++) {
    if (min_local > array[i])
      min_local = array[i];
  }
  return min_local;
}

#ifdef GL_MODE
class WindowPoisition{
public:
  static int x_position;
  static int y_position;
};

inline GLvoid ReSizeGLScene( GLsizei w, GLsizei h ){
  if( h == 0 ) {
     h = 1;
  }
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  glOrtho(0.0, w, 0.0, h, -1.0, 1.0);
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  glFlush();
  return;
}

inline int create_window(long long w, long long h,const char* name) {

  int div = 2;
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(glutGet(GLUT_SCREEN_WIDTH)/div,
                     glutGet(GLUT_SCREEN_HEIGHT)/div);
  glutInitWindowPosition(WindowPoisition::x_position, WindowPoisition::y_position);
  WindowPoisition::y_position += glutGet(GLUT_SCREEN_HEIGHT)/div;
  WindowPoisition::x_position += WindowPoisition::y_position / glutGet(GLUT_SCREEN_HEIGHT) * glutGet(GLUT_SCREEN_WIDTH)/div;
  WindowPoisition::y_position = WindowPoisition::y_position % glutGet(GLUT_SCREEN_HEIGHT);
  int discriptor = glutCreateWindow(name);

  glutReshapeFunc(ReSizeGLScene);
  glClearColor (1.0, 1.0, 1.0, 0.0);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0.0, w, 0.0, h, -1.0, 1.0);
  glClear (GL_COLOR_BUFFER_BIT);
  return discriptor;
}

inline void printRect(int bot, int top, int left, int right, double r, double g, double b) {
  glColor3f(r,g,b);
  glBegin(GL_QUADS);
    glVertex3i(left,bot,0);
    glVertex3i(right,bot,0);
    glVertex3i(right,top,0);
    glVertex3i(left,top,0);
  glEnd();
}

inline void printRect(int bot, int top, int left, int right, double gray) {
  printRect(bot,top,left,right,gray,gray,gray);
}

template<typename T>
inline void print(std::vector<T>& y, int windowID, bool is_first = true, float r=0, float g=0, float b=0) {
  glutSetWindow(windowID);

  if (is_first) {
    glClearColor (1.0, 1.0, 1.0, 0.0);
    glClear (GL_COLOR_BUFFER_BIT);
    glFlush();
  }

  glColor3f(0.5,0.5,0.5);
  glBegin(GL_LINE_STRIP);
    glVertex3f(1.0,1.0,0.0);
    glVertex3f(y.size(),1.0,0.0);
  glEnd();
  T max_y = my_max(y);
  T min_y = my_min(y);
  glBegin(GL_LINE_STRIP);
    glVertex3f(1.0,1.0,0.0);
    glVertex3f(1.0,max_y,0.0);
  glEnd();

  glColor3f(r,g,b);
  for (size_t x = 0; x < y.size()-1; x++) {
    glBegin(GL_LINE_STRIP);
      glVertex3f(x+1,y[x]/(double)max_y+1,0.0);
      glVertex3f(x+1+1,y[x+1]/(double)max_y+1,0.0);
    glEnd();
    if (y[x] == min_y) {
      glColor3f(1.0,0.0,0.0);
      glBegin(GL_LINE_STRIP);
        glVertex3f(x+1,y[x]/(double)max_y+1,0.0);
        glVertex3f(x+1,1.0,0.0);
      glEnd();
      glColor3f(r,g,b);
    }
  }
  glFlush();
}

template<typename T>
inline void print(std::vector<T>& y, std::string name, bool is_first = true, float r=0, float g=0, float b=0) {
  static std::map<std::string,int> window_store;
  if (window_store.find(name) == window_store.end()) {
    int windID = create_window(y.size()+2,3,name.c_str());
    window_store.insert({name,windID});
  }
  /*if (window_store.find(name)->second == 3) {
    for (auto elem : y) {
      std::cout << elem << " ";
    }
  }*/
  print(y, window_store.find(name)->second, is_first, r, g, b);
}
#endif
