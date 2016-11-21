#include "hough.h"
#include "extra_util_functions.h"
#include <iostream>
#include <iomanip>
#include <sstream>

#ifdef GL_MODE
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <random>

using namespace std;

#ifdef GL_MODE
  int WindowPoisition::x_position = 0;
  int WindowPoisition::y_position = 0;
#endif
int HoughTransform::count = 0;

HoughTransform::HoughTransform(int theta_partition = 20,double delta_ro = 0.5) :
                           rg(2*M_PI/theta_partition, delta_ro), window_id(-1) {
  _cells =  Array2d(theta_partition);
}

void HoughTransform::update_size(const size_t new_height) {
  if (new_height+1 <= height())
    return;
  size_t delta = new_height+1 - height();
  Array2d new_cols(width(), Array_cells(delta,0));
  for (size_t i = 0; i < width(); i++) {
    _cells[i].insert(_cells[i].end(), new_cols[i].begin(), new_cols[i].end());
  }
}

long long HoughTransform::invariant_function(long long x) {
  return x*x;
}

size_t HoughTransform::width()  const { return _cells.size(); }
size_t HoughTransform::height() const { return _cells[0].size(); }

void HoughTransform::transform(const PointD& p){
  Array_points points = rg.raster_build(0, 2*M_PI,
                                        [p](double fi)->double {
                                          return p.x*cos(fi) + p.y*sin(fi);
                                        });
  int max_y = 0;
  for (auto pt : points) {
    if (max_y < pt.y)
      max_y = pt.y;
  }
  update_size(max_y);
  for (size_t i = 0; i < points.size(); i++) {
    if (0 <= points[i].x && points[i].x < (int) width() &&
        0 <= points[i].y && points[i].y < (int) height()) {
      _cells[points[i].x][points[i].y]++;
    }
  }
}

shared_ptr<HoughTransform::Array_cov> HoughTransform::spectrum() {
  shared_ptr<vector<long long>> cov(new vector<long long>(width(),0));
  shared_ptr<Array_cov> corr(new Array_cov(width(),0));
  long long max = 0;
  for (size_t x = 0; x < width(); x++) {
    for (size_t y = 0; y < height(); y++) {
      cov->at(x) += invariant_function(_cells[x][y]);
    }
    if (max < cov->at(x))
      max = cov->at(x);
  }
  for (size_t i = 0; i < cov->size(); i++) {
    corr->at(i) = cov->at(i)/(Cov_type)max;
  }
  return corr;
}

shared_ptr<HoughTransform::Array_cov> HoughTransform::spectrumRO() {
  shared_ptr<Array_cov> out(new Array_cov(height(),0));
  for (size_t y = 0; y < height(); y++) {
    for (size_t x = 0; x < width(); x++) {
      out->at(y) += invariant_function(_cells[x][y]);
    }
  }
  return out;
}

HoughTransform::Array2d& HoughTransform::getCells() { return _cells; }
const HoughTransform::Array2d& HoughTransform::getCells() const
                                                    { return _cells; }
HoughTransform::Cells_type& HoughTransform::getCell(int x, int y)
                                               { return _cells[x][y]; }
const HoughTransform::Cells_type& HoughTransform::getCell(int x, int y) const
                                               { return _cells[x][y]; }

HoughTransform::DOUBLE HoughTransform::delta_theta() const {
  return rg.delta_x();
}

HoughTransform::DOUBLE HoughTransform::delta_ro() const {
  return rg.delta_y();
}

std::ostream& operator<<(std::ostream& ostr, const HoughTransform& h) {
  ostr << endl;
  for (size_t y = 0; y < h.height(); y++) {
    for (size_t x = 0; x < h.width(); x++) {
      ostr << std::setw(1) << h.getCells()[x][y];
    }
    ostr << endl;
  }
  return ostr;
}

#ifdef GL_MODE
int HoughTransform::printOpenGL(int col) {
  if (window_id == -1) {
    stringstream sstr;
    sstr << "hough " << ++count;
    window_id = create_window(2*width(), 2*height(),sstr.str().c_str());
  }
  else {
    glutSetWindow(window_id);
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glFlush();
  }
  int maxVal = 0;
  for (size_t y = 0; y < height(); y++) {
    for (size_t x = 0; x < width(); x++) {
      if (maxVal < _cells[x][y])
        maxVal = _cells[x][y];
    }
  }
  for (long int y = height()-1; 0 <= y; y--) {
    for (size_t x = 0; x < width(); x++) {
      if (_cells[x][y])
      printRect(y+height()/2,y+1+height()/2,x+width()/2,x+1+width()/2,
                (maxVal-_cells[x][y])/((double) maxVal));
    }

  }
  glColor3f(0.5,0.5,0.5);
  glBegin(GL_LINE_LOOP);
    glVertex3i(width()/2, height()/2, 0);
    glVertex3i(width()/2*3, height()/2, 0);
    glVertex3i(width()/2*3, height()/2*3, 0);
    glVertex3i(width()/2, height()/2*3, 0);
  glEnd();
  glFlush();
  return window_id;
}
#endif
