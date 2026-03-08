#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>

#include <vector>

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
      {
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;
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

///@note We don't use the entire Posture struct for interpolation, since bone_translation and bone_length are unused.
///      We create a reduced posture struct that only contains what we need for interpolation.
struct ReducedPosture {
    vector root_pos;
    vector bone_rotation[MAX_BONES_IN_ASF_FILE];

    ReducedPosture(const Posture& p) {
        root_pos = p.root_pos;
        std::copy(std::begin(p.bone_rotation), std::end(p.bone_rotation), std::begin(bone_rotation));
    }

    ReducedPosture() = default;

    static ReducedPosture Lerp(double t, const ReducedPosture& start, const ReducedPosture& end) {
        ReducedPosture res;
        res.root_pos = ::Lerp(t, start.root_pos, end.root_pos);
        for (int i = 0; i < MAX_BONES_IN_ASF_FILE; ++i) {
            res.bone_rotation[i] = ::Lerp(t, start.bone_rotation[i], end.bone_rotation[i]);
        }
        return res;
    }
};

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    const int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    const int step = N + 1;
    const int numKeyFrames = (inputLength / step) + 1;

    // prepare reduced posture for key frames
    std::vector<ReducedPosture> keyFrames;
    keyFrames.reserve(numKeyFrames);

    for (int i = 0; i < numKeyFrames; ++i)
    {
        keyFrames.emplace_back(*pInputMotion->GetPosture(i * (N + 1)));
    }

    // calculate Control points
    struct ControlPoint
    {
        ReducedPosture a;
        ReducedPosture b;
    };

    std::vector<ControlPoint> controlPoints;
    controlPoints.reserve(numKeyFrames);

    const double oneThird = 1.0 / 3.0;

    for (int i = 0; i < numKeyFrames; ++i) {
        ControlPoint cp{};

        if (i == 0) {
            const auto& q0 = keyFrames[0];
            const auto& q1 = keyFrames[1];
            const auto& q2 = keyFrames[2];
            cp.a = ReducedPosture::Lerp(oneThird, q0, ReducedPosture::Lerp(2.0, q2, q1));
        }
        else if (i == numKeyFrames - 1) {
            const auto& qn = keyFrames[i];
            const auto& qn_1 = keyFrames[i - 1];
            const auto& qn_2 = keyFrames[i - 2];
            cp.b = ReducedPosture::Lerp(oneThird, qn, ReducedPosture::Lerp(2.0, qn_2, qn_1));
        }
        else {
            const auto& q_prev = keyFrames[i - 1];
            const auto& q_curr = keyFrames[i];
            const auto& q_next = keyFrames[i + 1];

            // a_bar = (1 - 0.5) * (q_curr + (q_curr - q_prev)) + 0.5 * q_next
            ReducedPosture a_bar = ReducedPosture::Lerp(0.5, ReducedPosture::Lerp(2.0, q_prev, q_curr), q_next);

            cp.a = ReducedPosture::Lerp(oneThird, q_curr, a_bar);
            cp.b = ReducedPosture::Lerp(-oneThird, q_curr, a_bar);
        }
        controlPoints.push_back(std::move(cp));
    }

	// interpolate between key frames
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
			int keyFrameIndex = startKeyframe / step;
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            interpolatedPosture.root_pos = DeCasteljauEuler(
                t,
                keyFrames[keyFrameIndex].root_pos,
                controlPoints[keyFrameIndex].a.root_pos,
                controlPoints[keyFrameIndex+1].b.root_pos,
                keyFrames[keyFrameIndex+1].root_pos);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(
                    t,
                    keyFrames[keyFrameIndex].bone_rotation[bone],
                    controlPoints[keyFrameIndex].a.bone_rotation[bone],
                    controlPoints[keyFrameIndex + 1].b.bone_rotation[bone],
                    keyFrames[keyFrameIndex + 1].bone_rotation[bone]);
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

vector Lerp(double t, const vector& a, const vector& b)
{
    return a * (1 - t) + b * t;
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
    auto q0 = Lerp(t, p0, p1);
    auto q1 = Lerp(t, p1, p2);
    auto q2 = Lerp(t, p2, p3);
    auto r0 = Lerp(t, q0, q1);
    auto r1 = Lerp(t, q1, q2);
    auto result = Lerp(t, r0, r1);
    return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
    auto q0 = Slerp(t, p0, p1);
    auto q1 = Slerp(t, p1, p2);
    auto q2 = Slerp(t, p2, p3);
    auto r0 = Slerp(t, q0, q1);
    auto r1 = Slerp(t, q1, q2);
    auto result = Slerp(t, r0, r1);
    return result;
}

