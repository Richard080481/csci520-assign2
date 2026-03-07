#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for (int frame = startKeyframe + 1; frame < inputLength; frame++)
  {
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
  }
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    // angles[0] = theta1 (X), angles[1] = theta2 (Y), angles[2] = theta3 (Z)
    double t1 = angles[0] * M_PI / 180.0;
    double t2 = angles[1] * M_PI / 180.0;
    double t3 = angles[2] * M_PI / 180.0;

    double c1 = cos(t1), s1 = sin(t1);
    double c2 = cos(t2), s2 = sin(t2);
    double c3 = cos(t3), s3 = sin(t3);


    R[0] = c2 * c3;
    R[1] = s1 * s2 * c3 - c1 * s3;
    R[2] = c1 * s2 * c3 + s1 * s3;

    R[3] = c2 * s3;
    R[4] = s1 * s2 * s3 + c1 * c3;
    R[5] = c1 * s2 * s3 - s1 * c3;

    R[6] = -s2;
    R[7] = s1 * c2;
    R[8] = c1 * c2;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
				Quaternion<double> qStart, qEnd;
                double angles[3];
				Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, qEnd);
				Quaternion2Euler(Slerp(t, qStart, qEnd), angles);
				interpolatedPosture.bone_rotation[bone] = vector(angles[0], angles[1], angles[2]);
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
    {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  double R[9];
  Euler2Rotation(angles, R);
  q = Quaternion<double>::Matrix2Quaternion(R);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
	double R[9];
	q.Quaternion2Matrix(R);
	Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
    double cosTheta = qStart.Gets() * qEnd_.Gets() + 
                      qStart.Getx() * qEnd_.Getx() + 
                      qStart.Gety() * qEnd_.Gety() + 
                      qStart.Getz() * qEnd_.Getz();

    Quaternion<double> qEnd = qEnd_;

    if (cosTheta < 0.0) 
    {
        qEnd = -1.0 * qEnd_;
        cosTheta = -cosTheta;
    }

    Quaternion<double> result;

    if (cosTheta > 0.9999)
    {
        result = (1.0 - t) * qStart + t * qEnd;
        result.Normalize();
    }
    else 
    {
        double theta = acos(cosTheta);
        double sinTheta = sin(theta);
        
        double weightStart = sin((1.0 - t) * theta) / sinTheta;
        double weightEnd = sin(t * theta) / sinTheta;

        result = weightStart * qStart + weightEnd * qEnd;
    }

    return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
    double dot = p.Gets() * q.Gets() + p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz();

    Quaternion<double> result = (2.0 * dot) * q - p;
    return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

