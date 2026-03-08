/*
  interpolator.h

  Create interpolated motion.

  Revision 1 - Alla and Kiran, Jan 18, 2002
  Revision 2 - Jernej Barbic, Yili Zhao, Feb 2012
*/


#ifndef _INTERPOLATOR_H
#define _INTERPOLATOR_H

#include "motion.h"
#include "quaternion.h"

enum InterpolationType
{
  LINEAR = 0, BEZIER = 1
};

enum AngleRepresentation
{
  EULER = 0, QUATERNION = 1
};

class Interpolator
{
public:
  //constructor, destructor
  Interpolator();
  ~Interpolator();

  //Set interpolation type
  void SetInterpolationType(InterpolationType interpolationType) {m_InterpolationType = interpolationType;};
  //Set angle representation for interpolation
  void SetAngleRepresentation(AngleRepresentation angleRepresentation) {m_AngleRepresentation = angleRepresentation;};

  //Create interpolated motion and store it into pOutputMotion (which will also be allocated)
  void Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N);

  // linear interpolation
  static vector Lerp(double t, const vector& a, const vector& b);

  // quaternion interpolation
  static Quaternion<double> Slerp(double t, const Quaternion<double>& qStart, const Quaternion<double>& qEnd);
  static const vector Slerp(double t, const vector& qStart, const vector& qEnd);
  static Quaternion<double> Double(Quaternion<double> p, Quaternion<double> q);
protected:
  InterpolationType m_InterpolationType; //Interpolation type (Linear, Bezier)
  AngleRepresentation m_AngleRepresentation; //Angle representation (Euler, Quaternion)

  // conversion routines
  // angles are given in degrees; assume XYZ Euler angle order
  static void Rotation2Euler(double R[9], double angles[3]);
  static void Euler2Rotation(const double angles[3], double R[9]);
  static void Euler2Quaternion(const double angles[3], Quaternion<double> & q);
  static void Quaternion2Euler(Quaternion<double> & q, double angles[3]);


  // interpolation routines
  void LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N);
  void BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N);
  void LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N);
  void BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N);

  // Bezier spline evaluation
  vector DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3); // evaluate Bezier spline at t, using DeCasteljau construction, vector version
  Quaternion<double> DeCasteljauQuaternion(double t, const Quaternion<double>& p0, const Quaternion<double>& p1, const Quaternion<double>& p2, const Quaternion<double>& p3); // evaluate Bezier spline at t, using DeCasteljau construction, Quaternion version
  vector DeCasteljauQuaternion(double t, const vector& p0, const vector& p1, const vector& p2, const vector& p3);

};

#endif

