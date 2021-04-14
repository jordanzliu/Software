#pragma once
#include "software/proto/spline_msg.pb.h"
#include "software/geom/spline2d.h"

Polynomial1dMsg toPolynomial1dMsg(const Polynomial1d& poly);

Polynomial2dMsg toPolynomial2dMsg(const Polynomial2d& poly);

SplineSegment2dMsg toSplineSegment2dMsg(const SplineSegment2d& spline);

Spline2dMsg toSpline2dMsg(const Spline2d& spline);
