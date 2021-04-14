
#include "software/proto/message_translation/spline.h"
#include "software/proto/message_translation/tbots_geometry.h"

Polynomial1dMsg toPolynomial1dMsg(const Polynomial1d& poly)
{
    Polynomial1dMsg msg;
    for (unsigned i = 0; i < poly.getOrder(); i++) {
        msg.mutable_coeffs()->Add(poly.getCoeff(i));
    }
    return msg;
}

Polynomial2dMsg toPolynomial2dMsg(const Polynomial2d& poly)
{
    Polynomial2dMsg msg;
    *msg.mutable_x_poly() = toPolynomial1dMsg(poly.getPolyX());
    *msg.mutable_y_poly() = toPolynomial1dMsg(poly.getPolyY());
    return msg;
}

SplineSegment2dMsg toSplineSegment2dMsg(const SplineSegment2d& spline)
{
    SplineSegment2dMsg msg;
    msg.set_start_val(spline.getParametrizationStartVal());
    msg.set_end_val(spline.getParametrizationEndVal());
    *msg.mutable_polynomial() = toPolynomial2dMsg(spline.getPolynomial());
    return msg;
}


Spline2dMsg toSpline2dMsg(const Spline2d& spline)
{
    Spline2dMsg msg;
    for (const auto& seg : spline.getSplineSegments())
    {
        msg.mutable_spline_segments()->Add(std::move(toSplineSegment2dMsg(seg)));
    }

    for (const auto& pt : spline.getKnots())
    {
        msg.mutable_knots()->Add(std::move(*createPointProto(pt)));
    }
    return msg;
}
