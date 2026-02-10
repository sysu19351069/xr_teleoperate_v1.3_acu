#include "transforms.h"

using namespace AcuRobot::rcg;

// Constructors

MotionTransforms::MotionTransforms()
 :     fr_base0_X_ee(),
    fr_j6_X_ee(),
    fr_base0_X_link1_end(),
    fr_base0_X_link2_end(),
    fr_base0_X_link3_end(),
    fr_base0_X_link4_end(),
    fr_base0_X_link5_end(),
    fr_base0_X_fr_j1(),
    fr_base0_X_fr_j2(),
    fr_base0_X_fr_j3(),
    fr_base0_X_fr_j4(),
    fr_base0_X_fr_j5(),
    fr_base0_X_fr_j6(),
    fr_base0_X_fr_j7(),
    fr_base0_X_fr_j8(),
    fr_base0_X_fr_link6(),
    fr_base0_X_fr_link1(),
    fr_base0_X_fr_link2(),
    fr_base0_X_fr_link3(),
    fr_base0_X_fr_link4(),
    fr_base0_X_fr_link5(),
    fr_link1_X_fr_base0(),
    fr_link2_X_fr_link1(),
    fr_link1_X_fr_link2(),
    fr_link3_X_fr_link2(),
    fr_link2_X_fr_link3(),
    fr_link4_X_fr_link3(),
    fr_link3_X_fr_link4(),
    fr_link5_X_fr_link4(),
    fr_link4_X_fr_link5(),
    fr_link6_X_fr_link5(),
    fr_link5_X_fr_link6(),
    fr_link7_X_fr_link6(),
    fr_link6_X_fr_link7(),
    fr_link8_X_fr_link7(),
    fr_link7_X_fr_link8()
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_base0_X_ee(),
    fr_j6_X_ee(),
    fr_base0_X_link1_end(),
    fr_base0_X_link2_end(),
    fr_base0_X_link3_end(),
    fr_base0_X_link4_end(),
    fr_base0_X_link5_end(),
    fr_base0_X_fr_j1(),
    fr_base0_X_fr_j2(),
    fr_base0_X_fr_j3(),
    fr_base0_X_fr_j4(),
    fr_base0_X_fr_j5(),
    fr_base0_X_fr_j6(),
    fr_base0_X_fr_j7(),
    fr_base0_X_fr_j8(),
    fr_base0_X_fr_link6(),
    fr_base0_X_fr_link1(),
    fr_base0_X_fr_link2(),
    fr_base0_X_fr_link3(),
    fr_base0_X_fr_link4(),
    fr_base0_X_fr_link5(),
    fr_link1_X_fr_base0(),
    fr_link2_X_fr_link1(),
    fr_link1_X_fr_link2(),
    fr_link3_X_fr_link2(),
    fr_link2_X_fr_link3(),
    fr_link4_X_fr_link3(),
    fr_link3_X_fr_link4(),
    fr_link5_X_fr_link4(),
    fr_link4_X_fr_link5(),
    fr_link6_X_fr_link5(),
    fr_link5_X_fr_link6(),
    fr_link7_X_fr_link6(),
    fr_link6_X_fr_link7(),
    fr_link8_X_fr_link7(),
    fr_link7_X_fr_link8()
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_base0_X_ee(),
    fr_j6_X_ee(),
    fr_base0_X_link1_end(),
    fr_base0_X_link2_end(),
    fr_base0_X_link3_end(),
    fr_base0_X_link4_end(),
    fr_base0_X_link5_end(),
    fr_base0_X_fr_j1(),
    fr_base0_X_fr_j2(),
    fr_base0_X_fr_j3(),
    fr_base0_X_fr_j4(),
    fr_base0_X_fr_j5(),
    fr_base0_X_fr_j6(),
    fr_base0_X_fr_j7(),
    fr_base0_X_fr_j8(),
    fr_base0_X_fr_link6(),
    fr_base0_X_fr_link1(),
    fr_base0_X_fr_link2(),
    fr_base0_X_fr_link3(),
    fr_base0_X_fr_link4(),
    fr_base0_X_fr_link5(),
    fr_link1_X_fr_base0(),
    fr_link2_X_fr_link1(),
    fr_link1_X_fr_link2(),
    fr_link3_X_fr_link2(),
    fr_link2_X_fr_link3(),
    fr_link4_X_fr_link3(),
    fr_link3_X_fr_link4(),
    fr_link5_X_fr_link4(),
    fr_link4_X_fr_link5(),
    fr_link6_X_fr_link5(),
    fr_link5_X_fr_link6(),
    fr_link7_X_fr_link6(),
    fr_link6_X_fr_link7(),
    fr_link8_X_fr_link7(),
    fr_link7_X_fr_link8()
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_base0_X_ee::Type_fr_base0_X_ee()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_ee& MotionTransforms::Type_fr_base0_X_ee::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = (((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(0,1) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(1,1) = (((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * sin_q_j8)+(((((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(2,1) = (((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * cos_q_j8);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,0) = (((((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) * sin_q_j8)+(((((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+(((((( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(3,1) = (((((((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+((((((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j4)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * sin_q_j4)+((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) * cos_q_j8);
    (*this)(3,2) = ((((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tx_j8 * sin_q_j1 * cos_q_j4)) * cos_q_j6)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,3) = (((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(3,4) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,0) = (((((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) * sin_q_j8)+(((((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+(((((( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(4,1) = (((((((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+((((((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j4)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * sin_q_j4)+((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) * cos_q_j8);
    (*this)(4,2) = ((((( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * cos_q_j1 * cos_q_j4)) * cos_q_j6)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(4,4) = (((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,0) = (((((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+((( tx_j8 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j2 * cos_q_j3)) * cos_q_j5)) * sin_q_j8)+(((((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) *  q(J7))+(((((( tz_j7+ tz_ee) * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * sin_q_j6)+((((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(5,1) = (((((((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * cos_q_j6)) *  q(J7))+((((((- tz_j7- tz_ee) * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+(((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * cos_q_j4)) * sin_q_j6)+(((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+((( tx_j8 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j2 * cos_q_j3)) * cos_q_j5)) * cos_q_j8);
    (*this)(5,2) = (((((- tx_j8 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * sin_q_j4 * cos_q_j6)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5);
    (*this)(5,3) = (((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * sin_q_j8)+(((((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(5,4) = (((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * cos_q_j8);
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
MotionTransforms::Type_fr_j6_X_ee::Type_fr_j6_X_ee()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_j6_X_ee& MotionTransforms::Type_fr_j6_X_ee::update(const state_t& q)
{
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    (*this)(0,1) = (-cos_q_j6 * sin_q_j8)-(sin_q_j6 * cos_q_j8);
    (*this)(1,0) = (cos_q_j6 * sin_q_j8)+(sin_q_j6 * cos_q_j8);
    (*this)(1,1) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    (*this)(3,0) = ((((- tz_j7- tz_ee) * cos_q_j6)-(cos_q_j6 *  q(J7))) * sin_q_j8)+((((- tz_j7- tz_ee) * sin_q_j6)-(sin_q_j6 *  q(J7))) * cos_q_j8);
    (*this)(3,1) = (((sin_q_j6 *  q(J7))+(( tz_j7+ tz_ee) * sin_q_j6)) * sin_q_j8)+((((- tz_j7- tz_ee) * cos_q_j6)-(cos_q_j6 *  q(J7))) * cos_q_j8);
    (*this)(3,2) =  tx_j8 * sin_q_j6;
    (*this)(3,3) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    (*this)(3,4) = (-cos_q_j6 * sin_q_j8)-(sin_q_j6 * cos_q_j8);
    (*this)(4,0) = ((((- tz_j7- tz_ee) * sin_q_j6)-(sin_q_j6 *  q(J7))) * sin_q_j8)+(((cos_q_j6 *  q(J7))+(( tz_j7+ tz_ee) * cos_q_j6)) * cos_q_j8);
    (*this)(4,1) = ((((- tz_j7- tz_ee) * cos_q_j6)-(cos_q_j6 *  q(J7))) * sin_q_j8)+((((- tz_j7- tz_ee) * sin_q_j6)-(sin_q_j6 *  q(J7))) * cos_q_j8);
    (*this)(4,2) = - tx_j8 * cos_q_j6;
    (*this)(4,3) = (cos_q_j6 * sin_q_j8)+(sin_q_j6 * cos_q_j8);
    (*this)(4,4) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    (*this)(5,0) =  tx_j8 * sin_q_j8;
    (*this)(5,1) =  tx_j8 * cos_q_j8;
    return *this;
}
MotionTransforms::Type_fr_base0_X_link1_end::Type_fr_base0_X_link1_end()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base0_X_link1_end& MotionTransforms::Type_fr_base0_X_link1_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,1) = -sin_q_j1;
    (*this)(1,0) = sin_q_j1;
    (*this)(1,1) = cos_q_j1;
    (*this)(3,0) = - tz_link1_end * sin_q_j1;
    (*this)(3,1) = - tz_link1_end * cos_q_j1;
    (*this)(3,3) = cos_q_j1;
    (*this)(3,4) = -sin_q_j1;
    (*this)(4,0) =  tz_link1_end * cos_q_j1;
    (*this)(4,1) = - tz_link1_end * sin_q_j1;
    (*this)(4,3) = sin_q_j1;
    (*this)(4,4) = cos_q_j1;
    return *this;
}
MotionTransforms::Type_fr_base0_X_link2_end::Type_fr_base0_X_link2_end()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_link2_end& MotionTransforms::Type_fr_base0_X_link2_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j1 * cos_q_j2;
    (*this)(0,1) = -cos_q_j1 * sin_q_j2;
    (*this)(0,2) = sin_q_j1;
    (*this)(1,0) = sin_q_j1 * cos_q_j2;
    (*this)(1,1) = -sin_q_j1 * sin_q_j2;
    (*this)(1,2) = -cos_q_j1;
    (*this)(2,0) = sin_q_j2;
    (*this)(2,1) = cos_q_j2;
    (*this)(3,0) = (- tz_link2_end * cos_q_j1 * sin_q_j2)-( tz_j2 * sin_q_j1 * cos_q_j2);
    (*this)(3,1) = ( tz_j2 * sin_q_j1 * sin_q_j2)-( tz_link2_end * cos_q_j1 * cos_q_j2);
    (*this)(3,2) =  tz_j2 * cos_q_j1;
    (*this)(3,3) = cos_q_j1 * cos_q_j2;
    (*this)(3,4) = -cos_q_j1 * sin_q_j2;
    (*this)(3,5) = sin_q_j1;
    (*this)(4,0) = ( tz_j2 * cos_q_j1 * cos_q_j2)-( tz_link2_end * sin_q_j1 * sin_q_j2);
    (*this)(4,1) = (- tz_j2 * cos_q_j1 * sin_q_j2)-( tz_link2_end * sin_q_j1 * cos_q_j2);
    (*this)(4,2) =  tz_j2 * sin_q_j1;
    (*this)(4,3) = sin_q_j1 * cos_q_j2;
    (*this)(4,4) = -sin_q_j1 * sin_q_j2;
    (*this)(4,5) = -cos_q_j1;
    (*this)(5,0) =  tz_link2_end * cos_q_j2;
    (*this)(5,1) = - tz_link2_end * sin_q_j2;
    (*this)(5,3) = sin_q_j2;
    (*this)(5,4) = cos_q_j2;
    return *this;
}
MotionTransforms::Type_fr_base0_X_link3_end::Type_fr_base0_X_link3_end()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_link3_end& MotionTransforms::Type_fr_base0_X_link3_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,1) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(0,2) = -sin_q_j1;
    (*this)(1,0) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,1) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(1,2) = cos_q_j1;
    (*this)(2,0) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,1) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(3,0) = ((( tz_link3_end * cos_q_j1 * sin_q_j2)-( tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+((( tz_j2 * sin_q_j1 * sin_q_j2)+( tz_link3_end * cos_q_j1 * cos_q_j2)) * cos_q_j3);
    (*this)(3,1) = (((- tz_j2 * sin_q_j1 * sin_q_j2)-( tz_link3_end * cos_q_j1 * cos_q_j2)) * sin_q_j3)+((( tz_link3_end * cos_q_j1 * sin_q_j2)-( tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3);
    (*this)(3,2) = (- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1);
    (*this)(3,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(3,4) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,5) = -sin_q_j1;
    (*this)(4,0) = ((( tz_link3_end * sin_q_j1 * sin_q_j2)+( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)+((( tz_link3_end * sin_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1 * sin_q_j2)) * cos_q_j3);
    (*this)(4,1) = ((( tz_j2 * cos_q_j1 * sin_q_j2)-( tz_link3_end * sin_q_j1 * cos_q_j2)) * sin_q_j3)+((( tz_link3_end * sin_q_j1 * sin_q_j2)+( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3);
    (*this)(4,2) = (- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1);
    (*this)(4,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,4) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,5) = cos_q_j1;
    (*this)(5,0) = ( tz_link3_end * sin_q_j2 * cos_q_j3)-( tz_link3_end * cos_q_j2 * sin_q_j3);
    (*this)(5,1) = (- tz_link3_end * sin_q_j2 * sin_q_j3)-( tz_link3_end * cos_q_j2 * cos_q_j3);
    (*this)(5,2) = - ty_j3 * sin_q_j2;
    (*this)(5,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(5,4) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    return *this;
}
MotionTransforms::Type_fr_base0_X_link4_end::Type_fr_base0_X_link4_end()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,2) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_link4_end& MotionTransforms::Type_fr_base0_X_link4_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(0,1) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,2) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,0) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(1,1) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,2) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(2,0) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(2,1) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,2) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(3,0) = (((- tz_link4_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_link4_end * sin_q_j1)) * cos_q_j4);
    (*this)(3,1) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_link4_end * sin_q_j1)) * sin_q_j4)+(((- tz_link4_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(3,2) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(3,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(3,5) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,0) = (((- tz_link4_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_link4_end * cos_q_j1)) * cos_q_j4);
    (*this)(4,1) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_link4_end * cos_q_j1)) * sin_q_j4)+(((- tz_link4_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(4,2) = ((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,3) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(4,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(4,5) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(5,0) = (( tz_link4_end * cos_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4;
    (*this)(5,1) = (( tz_link4_end * cos_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(5,3) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(5,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(5,5) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    return *this;
}
MotionTransforms::Type_fr_base0_X_link5_end::Type_fr_base0_X_link5_end()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_link5_end& MotionTransforms::Type_fr_base0_X_link5_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(0,1) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(0,2) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(1,0) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,1) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(1,2) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(2,0) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,1) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,2) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(3,0) = ((( tz_link5_end * sin_q_j1 * sin_q_j4)+(((- tz_link5_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)-( tz_link5_end * cos_q_j1 * cos_q_j2 * sin_q_j3)+( tz_link5_end * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,1) = ((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)+( tz_link5_end * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_link5_end * sin_q_j1 * sin_q_j4)+(((- tz_link5_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,2) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(3,4) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(3,5) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(4,0) = (((- tz_link5_end * cos_q_j1 * sin_q_j4)+(((- tz_link5_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)-( tz_link5_end * sin_q_j1 * cos_q_j2 * sin_q_j3)+( tz_link5_end * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,1) = ((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)+( tz_link5_end * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((- tz_link5_end * cos_q_j1 * sin_q_j4)+(((- tz_link5_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,2) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(4,3) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(4,4) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(4,5) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(5,0) = ((( tz_link5_end * cos_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+((((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4)-( tz_link5_end * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,1) = (((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4)+( tz_link5_end * sin_q_j2 * sin_q_j3)+( tz_link5_end * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_link5_end * cos_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(5,2) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(5,3) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(5,4) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,5) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_j1::Type_fr_base0_X_fr_j1()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base0_X_fr_j1& MotionTransforms::Type_fr_base0_X_fr_j1::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_j2::Type_fr_base0_X_fr_j2()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_j2& MotionTransforms::Type_fr_base0_X_fr_j2::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,2) = sin_q_j1;
    (*this)(1,0) = sin_q_j1;
    (*this)(1,2) = -cos_q_j1;
    (*this)(3,0) = - tz_j2 * sin_q_j1;
    (*this)(3,2) =  tz_j2 * cos_q_j1;
    (*this)(3,3) = cos_q_j1;
    (*this)(3,5) = sin_q_j1;
    (*this)(4,0) =  tz_j2 * cos_q_j1;
    (*this)(4,2) =  tz_j2 * sin_q_j1;
    (*this)(4,3) = sin_q_j1;
    (*this)(4,5) = -cos_q_j1;
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_j3::Type_fr_base0_X_fr_j3()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_j3& MotionTransforms::Type_fr_base0_X_fr_j3::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = -cos_q_j1 * sin_q_j2;
    (*this)(0,1) = cos_q_j1 * cos_q_j2;
    (*this)(0,2) = -sin_q_j1;
    (*this)(1,0) = -sin_q_j1 * sin_q_j2;
    (*this)(1,1) = sin_q_j1 * cos_q_j2;
    (*this)(1,2) = cos_q_j1;
    (*this)(2,0) = cos_q_j2;
    (*this)(2,1) = sin_q_j2;
    (*this)(3,0) =  tz_j2 * sin_q_j1 * sin_q_j2;
    (*this)(3,1) = (- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1);
    (*this)(3,2) = (- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1);
    (*this)(3,3) = -cos_q_j1 * sin_q_j2;
    (*this)(3,4) = cos_q_j1 * cos_q_j2;
    (*this)(3,5) = -sin_q_j1;
    (*this)(4,0) = - tz_j2 * cos_q_j1 * sin_q_j2;
    (*this)(4,1) = ( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1);
    (*this)(4,2) = (- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1);
    (*this)(4,3) = -sin_q_j1 * sin_q_j2;
    (*this)(4,4) = sin_q_j1 * cos_q_j2;
    (*this)(4,5) = cos_q_j1;
    (*this)(5,2) = - ty_j3 * sin_q_j2;
    (*this)(5,3) = cos_q_j2;
    (*this)(5,4) = sin_q_j2;
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_j4::Type_fr_base0_X_fr_j4()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_j4& MotionTransforms::Type_fr_base0_X_fr_j4::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(0,1) = -sin_q_j1;
    (*this)(0,2) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,0) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(1,1) = cos_q_j1;
    (*this)(1,2) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(2,0) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(2,2) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(3,0) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3);
    (*this)(3,1) = (- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1);
    (*this)(3,2) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(3,3) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,4) = -sin_q_j1;
    (*this)(3,5) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,0) = ( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3);
    (*this)(4,1) = (- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1);
    (*this)(4,2) = ((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,3) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,4) = cos_q_j1;
    (*this)(4,5) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(5,1) = - ty_j3 * sin_q_j2;
    (*this)(5,3) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(5,5) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_j5::Type_fr_base0_X_fr_j5()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,1) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_j5& MotionTransforms::Type_fr_base0_X_fr_j5::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(0,1) = (cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(0,2) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(1,0) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(1,1) = (sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(1,2) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(2,0) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(2,1) = (-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3);
    (*this)(2,2) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(3,0) = (((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4);
    (*this)(3,1) = ((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(3,2) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(3,4) = (cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(3,5) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(4,0) = (((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4);
    (*this)(4,1) = (((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,2) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(4,3) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(4,4) = (sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(4,5) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(5,0) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4;
    (*this)(5,2) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(5,3) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(5,4) = (-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3);
    (*this)(5,5) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_j6::Type_fr_base0_X_fr_j6()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_j6& MotionTransforms::Type_fr_base0_X_fr_j6::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(0,1) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,1) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,1) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,0) = ((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5);
    (*this)(3,1) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(3,2) = (((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(3,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,0) = (((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5);
    (*this)(4,1) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(4,2) = (((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(4,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,0) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5;
    (*this)(5,1) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(5,2) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5;
    (*this)(5,3) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(5,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_j7::Type_fr_base0_X_fr_j7()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_j7& MotionTransforms::Type_fr_base0_X_fr_j7::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(2,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,0) = ((((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j4)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_j7 * sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(3,1) = (((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+((( tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j7 * sin_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j4)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6);
    (*this)(3,2) = (((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,3) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(3,4) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,0) = ((((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * cos_q_j1 * sin_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j7 * cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(4,1) = ((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_j7 * cos_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * cos_q_j1 * sin_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6);
    (*this)(4,2) = (((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(4,4) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,0) = ((((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * sin_q_j6)+((((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * cos_q_j6);
    (*this)(5,1) = (((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((( tz_j7 * sin_q_j2 * cos_q_j3)-( tz_j7 * cos_q_j2 * sin_q_j3)) * sin_q_j4)) * sin_q_j6)+((((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * cos_q_j6);
    (*this)(5,2) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5;
    (*this)(5,3) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(5,4) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_j8::Type_fr_base0_X_fr_j8()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_j8& MotionTransforms::Type_fr_base0_X_fr_j8::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(2,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,0) = (((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+((((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j4)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_j7 * sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(3,1) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+((( tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j7 * sin_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j4)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,2) = ((((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tx_j8 * sin_q_j1 * cos_q_j4)) * cos_q_j6)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,3) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(3,4) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,0) = (((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+((((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * cos_q_j1 * sin_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j7 * cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(4,1) = (((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_j7 * cos_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * cos_q_j1 * sin_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,2) = ((((( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * cos_q_j1 * cos_q_j4)) * cos_q_j6)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(4,4) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,0) = (((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) *  q(J7))+((((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * sin_q_j6)+((((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * cos_q_j6);
    (*this)(5,1) = (((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((( tz_j7 * sin_q_j2 * cos_q_j3)-( tz_j7 * cos_q_j2 * sin_q_j3)) * sin_q_j4)) * sin_q_j6)+((((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+((( tx_j8 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,2) = (((((- tx_j8 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * sin_q_j4 * cos_q_j6)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5);
    (*this)(5,3) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(5,4) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_link6::Type_fr_base0_X_fr_link6()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_link6& MotionTransforms::Type_fr_base0_X_fr_link6::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(2,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,0) = ((((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(3,1) = (((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6);
    (*this)(3,2) = (((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,3) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(3,4) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,0) = (((((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(4,1) = ((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6);
    (*this)(4,2) = (((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(4,4) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,0) = ((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4 * sin_q_j6)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5 * cos_q_j6);
    (*this)(5,1) = (((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5 * sin_q_j6)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4 * cos_q_j6);
    (*this)(5,2) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5;
    (*this)(5,3) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(5,4) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_link1::Type_fr_base0_X_fr_link1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base0_X_fr_link1& MotionTransforms::Type_fr_base0_X_fr_link1::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,1) = -sin_q_j1;
    (*this)(1,0) = sin_q_j1;
    (*this)(1,1) = cos_q_j1;
    (*this)(3,3) = cos_q_j1;
    (*this)(3,4) = -sin_q_j1;
    (*this)(4,3) = sin_q_j1;
    (*this)(4,4) = cos_q_j1;
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_link2::Type_fr_base0_X_fr_link2()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_link2& MotionTransforms::Type_fr_base0_X_fr_link2::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j1 * cos_q_j2;
    (*this)(0,1) = -cos_q_j1 * sin_q_j2;
    (*this)(0,2) = sin_q_j1;
    (*this)(1,0) = sin_q_j1 * cos_q_j2;
    (*this)(1,1) = -sin_q_j1 * sin_q_j2;
    (*this)(1,2) = -cos_q_j1;
    (*this)(2,0) = sin_q_j2;
    (*this)(2,1) = cos_q_j2;
    (*this)(3,0) = - tz_j2 * sin_q_j1 * cos_q_j2;
    (*this)(3,1) =  tz_j2 * sin_q_j1 * sin_q_j2;
    (*this)(3,2) =  tz_j2 * cos_q_j1;
    (*this)(3,3) = cos_q_j1 * cos_q_j2;
    (*this)(3,4) = -cos_q_j1 * sin_q_j2;
    (*this)(3,5) = sin_q_j1;
    (*this)(4,0) =  tz_j2 * cos_q_j1 * cos_q_j2;
    (*this)(4,1) = - tz_j2 * cos_q_j1 * sin_q_j2;
    (*this)(4,2) =  tz_j2 * sin_q_j1;
    (*this)(4,3) = sin_q_j1 * cos_q_j2;
    (*this)(4,4) = -sin_q_j1 * sin_q_j2;
    (*this)(4,5) = -cos_q_j1;
    (*this)(5,3) = sin_q_j2;
    (*this)(5,4) = cos_q_j2;
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_link3::Type_fr_base0_X_fr_link3()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_link3& MotionTransforms::Type_fr_base0_X_fr_link3::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,1) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(0,2) = -sin_q_j1;
    (*this)(1,0) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,1) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(1,2) = cos_q_j1;
    (*this)(2,0) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,1) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(3,0) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(3,1) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3);
    (*this)(3,2) = (- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1);
    (*this)(3,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(3,4) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,5) = -sin_q_j1;
    (*this)(4,0) = ((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,1) = ( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3);
    (*this)(4,2) = (- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1);
    (*this)(4,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,4) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,5) = cos_q_j1;
    (*this)(5,2) = - ty_j3 * sin_q_j2;
    (*this)(5,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(5,4) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_link4::Type_fr_base0_X_fr_link4()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,2) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_link4& MotionTransforms::Type_fr_base0_X_fr_link4::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(0,1) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,2) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,0) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(1,1) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,2) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(2,0) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(2,1) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,2) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(3,0) = (((- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)) * cos_q_j4);
    (*this)(3,1) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)) * sin_q_j4)+(((- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(3,2) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(3,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(3,5) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,0) = (((- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)) * cos_q_j4);
    (*this)(4,1) = (((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)) * sin_q_j4)+(((- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(4,2) = ((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,3) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(4,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(4,5) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(5,0) = - ty_j3 * sin_q_j2 * sin_q_j4;
    (*this)(5,1) = - ty_j3 * sin_q_j2 * cos_q_j4;
    (*this)(5,3) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(5,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(5,5) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    return *this;
}
MotionTransforms::Type_fr_base0_X_fr_link5::Type_fr_base0_X_fr_link5()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base0_X_fr_link5& MotionTransforms::Type_fr_base0_X_fr_link5::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(0,1) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(0,2) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(1,0) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,1) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(1,2) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(2,0) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,1) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,2) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(3,0) = ((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5);
    (*this)(3,1) = ((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,2) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(3,4) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(3,5) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(4,0) = (((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5);
    (*this)(4,1) = ((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,2) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(4,3) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(4,4) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(4,5) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(5,0) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5;
    (*this)(5,1) = ((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5;
    (*this)(5,2) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(5,3) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(5,4) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,5) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    return *this;
}
MotionTransforms::Type_fr_link1_X_fr_base0::Type_fr_link1_X_fr_base0()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_link1_X_fr_base0& MotionTransforms::Type_fr_link1_X_fr_base0::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,1) = sin_q_j1;
    (*this)(1,0) = -sin_q_j1;
    (*this)(1,1) = cos_q_j1;
    (*this)(3,3) = cos_q_j1;
    (*this)(3,4) = sin_q_j1;
    (*this)(4,3) = -sin_q_j1;
    (*this)(4,4) = cos_q_j1;
    return *this;
}
MotionTransforms::Type_fr_link2_X_fr_link1::Type_fr_link2_X_fr_link1()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) =  tz_j2;    // Maxima DSL: _k__tz_j2
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_link2_X_fr_link1& MotionTransforms::Type_fr_link2_X_fr_link1::update(const state_t& q)
{
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j2;
    (*this)(0,2) = sin_q_j2;
    (*this)(1,0) = -sin_q_j2;
    (*this)(1,2) = cos_q_j2;
    (*this)(3,1) =  tz_j2 * cos_q_j2;
    (*this)(3,3) = cos_q_j2;
    (*this)(3,5) = sin_q_j2;
    (*this)(4,1) = - tz_j2 * sin_q_j2;
    (*this)(4,3) = -sin_q_j2;
    (*this)(4,5) = cos_q_j2;
    return *this;
}
MotionTransforms::Type_fr_link1_X_fr_link2::Type_fr_link1_X_fr_link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) =  tz_j2;    // Maxima DSL: _k__tz_j2
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_link1_X_fr_link2& MotionTransforms::Type_fr_link1_X_fr_link2::update(const state_t& q)
{
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j2;
    (*this)(0,1) = -sin_q_j2;
    (*this)(2,0) = sin_q_j2;
    (*this)(2,1) = cos_q_j2;
    (*this)(3,3) = cos_q_j2;
    (*this)(3,4) = -sin_q_j2;
    (*this)(4,0) =  tz_j2 * cos_q_j2;
    (*this)(4,1) = - tz_j2 * sin_q_j2;
    (*this)(5,3) = sin_q_j2;
    (*this)(5,4) = cos_q_j2;
    return *this;
}
MotionTransforms::Type_fr_link3_X_fr_link2::Type_fr_link3_X_fr_link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = - ty_j3;    // Maxima DSL: -_k__ty_j3
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_link3_X_fr_link2& MotionTransforms::Type_fr_link3_X_fr_link2::update(const state_t& q)
{
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = sin_q_j3;
    (*this)(0,1) = cos_q_j3;
    (*this)(1,0) = cos_q_j3;
    (*this)(1,1) = -sin_q_j3;
    (*this)(3,2) = - ty_j3 * sin_q_j3;
    (*this)(3,3) = sin_q_j3;
    (*this)(3,4) = cos_q_j3;
    (*this)(4,2) = - ty_j3 * cos_q_j3;
    (*this)(4,3) = cos_q_j3;
    (*this)(4,4) = -sin_q_j3;
    return *this;
}
MotionTransforms::Type_fr_link2_X_fr_link3::Type_fr_link2_X_fr_link3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = - ty_j3;    // Maxima DSL: -_k__ty_j3
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const MotionTransforms::Type_fr_link2_X_fr_link3& MotionTransforms::Type_fr_link2_X_fr_link3::update(const state_t& q)
{
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = sin_q_j3;
    (*this)(0,1) = cos_q_j3;
    (*this)(1,0) = cos_q_j3;
    (*this)(1,1) = -sin_q_j3;
    (*this)(3,3) = sin_q_j3;
    (*this)(3,4) = cos_q_j3;
    (*this)(4,3) = cos_q_j3;
    (*this)(4,4) = -sin_q_j3;
    (*this)(5,0) = - ty_j3 * sin_q_j3;
    (*this)(5,1) = - ty_j3 * cos_q_j3;
    return *this;
}
MotionTransforms::Type_fr_link4_X_fr_link3::Type_fr_link4_X_fr_link3()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_link4_X_fr_link3& MotionTransforms::Type_fr_link4_X_fr_link3::update(const state_t& q)
{
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,1) = cos_q_j4;
    (*this)(0,2) = sin_q_j4;
    (*this)(1,1) = -sin_q_j4;
    (*this)(1,2) = cos_q_j4;
    (*this)(3,4) = cos_q_j4;
    (*this)(3,5) = sin_q_j4;
    (*this)(4,4) = -sin_q_j4;
    (*this)(4,5) = cos_q_j4;
    return *this;
}
MotionTransforms::Type_fr_link3_X_fr_link4::Type_fr_link3_X_fr_link4()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_link3_X_fr_link4& MotionTransforms::Type_fr_link3_X_fr_link4::update(const state_t& q)
{
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(1,0) = cos_q_j4;
    (*this)(1,1) = -sin_q_j4;
    (*this)(2,0) = sin_q_j4;
    (*this)(2,1) = cos_q_j4;
    (*this)(4,3) = cos_q_j4;
    (*this)(4,4) = -sin_q_j4;
    (*this)(5,3) = sin_q_j4;
    (*this)(5,4) = cos_q_j4;
    return *this;
}
MotionTransforms::Type_fr_link5_X_fr_link4::Type_fr_link5_X_fr_link4()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = - tz_j5;    // Maxima DSL: -_k__tz_j5
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_link5_X_fr_link4& MotionTransforms::Type_fr_link5_X_fr_link4::update(const state_t& q)
{
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = cos_q_j5;
    (*this)(0,2) = -sin_q_j5;
    (*this)(1,0) = -sin_q_j5;
    (*this)(1,2) = -cos_q_j5;
    (*this)(3,1) =  tz_j5 * cos_q_j5;
    (*this)(3,3) = cos_q_j5;
    (*this)(3,5) = -sin_q_j5;
    (*this)(4,1) = - tz_j5 * sin_q_j5;
    (*this)(4,3) = -sin_q_j5;
    (*this)(4,5) = -cos_q_j5;
    return *this;
}
MotionTransforms::Type_fr_link4_X_fr_link5::Type_fr_link4_X_fr_link5()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = - tz_j5;    // Maxima DSL: -_k__tz_j5
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_link4_X_fr_link5& MotionTransforms::Type_fr_link4_X_fr_link5::update(const state_t& q)
{
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = cos_q_j5;
    (*this)(0,1) = -sin_q_j5;
    (*this)(2,0) = -sin_q_j5;
    (*this)(2,1) = -cos_q_j5;
    (*this)(3,3) = cos_q_j5;
    (*this)(3,4) = -sin_q_j5;
    (*this)(4,0) =  tz_j5 * cos_q_j5;
    (*this)(4,1) = - tz_j5 * sin_q_j5;
    (*this)(5,3) = -sin_q_j5;
    (*this)(5,4) = -cos_q_j5;
    return *this;
}
MotionTransforms::Type_fr_link6_X_fr_link5::Type_fr_link6_X_fr_link5()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_link6_X_fr_link5& MotionTransforms::Type_fr_link6_X_fr_link5::update(const state_t& q)
{
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = cos_q_j6;
    (*this)(0,2) = sin_q_j6;
    (*this)(1,0) = -sin_q_j6;
    (*this)(1,2) = cos_q_j6;
    (*this)(3,3) = cos_q_j6;
    (*this)(3,5) = sin_q_j6;
    (*this)(4,3) = -sin_q_j6;
    (*this)(4,5) = cos_q_j6;
    return *this;
}
MotionTransforms::Type_fr_link5_X_fr_link6::Type_fr_link5_X_fr_link6()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_link5_X_fr_link6& MotionTransforms::Type_fr_link5_X_fr_link6::update(const state_t& q)
{
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = cos_q_j6;
    (*this)(0,1) = -sin_q_j6;
    (*this)(2,0) = sin_q_j6;
    (*this)(2,1) = cos_q_j6;
    (*this)(3,3) = cos_q_j6;
    (*this)(3,4) = -sin_q_j6;
    (*this)(5,3) = sin_q_j6;
    (*this)(5,4) = cos_q_j6;
    return *this;
}
MotionTransforms::Type_fr_link7_X_fr_link6::Type_fr_link7_X_fr_link6()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_link7_X_fr_link6& MotionTransforms::Type_fr_link7_X_fr_link6::update(const state_t& q)
{
    (*this)(3,1) =  q(J7)+ tz_j7;
    (*this)(4,0) = - q(J7)- tz_j7;
    return *this;
}
MotionTransforms::Type_fr_link6_X_fr_link7::Type_fr_link6_X_fr_link7()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_link6_X_fr_link7& MotionTransforms::Type_fr_link6_X_fr_link7::update(const state_t& q)
{
    (*this)(3,1) = - q(J7)- tz_j7;
    (*this)(4,0) =  q(J7)+ tz_j7;
    return *this;
}
MotionTransforms::Type_fr_link8_X_fr_link7::Type_fr_link8_X_fr_link7()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_j8;    // Maxima DSL: -_k__tx_j8
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_link8_X_fr_link7& MotionTransforms::Type_fr_link8_X_fr_link7::update(const state_t& q)
{
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = cos_q_j8;
    (*this)(0,1) = sin_q_j8;
    (*this)(1,0) = -sin_q_j8;
    (*this)(1,1) = cos_q_j8;
    (*this)(3,2) =  tx_j8 * sin_q_j8;
    (*this)(3,3) = cos_q_j8;
    (*this)(3,4) = sin_q_j8;
    (*this)(4,2) =  tx_j8 * cos_q_j8;
    (*this)(4,3) = -sin_q_j8;
    (*this)(4,4) = cos_q_j8;
    return *this;
}
MotionTransforms::Type_fr_link7_X_fr_link8::Type_fr_link7_X_fr_link8()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_j8;    // Maxima DSL: -_k__tx_j8
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_link7_X_fr_link8& MotionTransforms::Type_fr_link7_X_fr_link8::update(const state_t& q)
{
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = cos_q_j8;
    (*this)(0,1) = -sin_q_j8;
    (*this)(1,0) = sin_q_j8;
    (*this)(1,1) = cos_q_j8;
    (*this)(3,3) = cos_q_j8;
    (*this)(3,4) = -sin_q_j8;
    (*this)(4,3) = sin_q_j8;
    (*this)(4,4) = cos_q_j8;
    (*this)(5,0) =  tx_j8 * sin_q_j8;
    (*this)(5,1) =  tx_j8 * cos_q_j8;
    return *this;
}

ForceTransforms::Type_fr_base0_X_ee::Type_fr_base0_X_ee()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_ee& ForceTransforms::Type_fr_base0_X_ee::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = (((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(0,1) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = (((((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) * sin_q_j8)+(((((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+(((((( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(0,4) = (((((((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+((((((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j4)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * sin_q_j4)+((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) * cos_q_j8);
    (*this)(0,5) = ((((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tx_j8 * sin_q_j1 * cos_q_j4)) * cos_q_j6)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(1,1) = (((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = (((((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) * sin_q_j8)+(((((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+(((((( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(1,4) = (((((((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+((((((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j4)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * sin_q_j4)+((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) * cos_q_j8);
    (*this)(1,5) = ((((( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * cos_q_j1 * cos_q_j4)) * cos_q_j6)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * sin_q_j8)+(((((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(2,1) = (((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * cos_q_j8);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = (((((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+((( tx_j8 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j2 * cos_q_j3)) * cos_q_j5)) * sin_q_j8)+(((((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) *  q(J7))+(((((( tz_j7+ tz_ee) * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * sin_q_j6)+((((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(2,4) = (((((((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * cos_q_j6)) *  q(J7))+((((((- tz_j7- tz_ee) * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+(((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * cos_q_j4)) * sin_q_j6)+(((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * sin_q_j6)+(((((( tz_j7+ tz_ee) * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+((( tx_j8 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j2 * cos_q_j3)) * cos_q_j5)) * cos_q_j8);
    (*this)(2,5) = (((((- tx_j8 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * sin_q_j4 * cos_q_j6)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5);
    (*this)(3,3) = (((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(3,4) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(4,4) = (((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,3) = (((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * sin_q_j8)+(((((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(5,4) = (((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * cos_q_j8);
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
ForceTransforms::Type_fr_j6_X_ee::Type_fr_j6_X_ee()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_j6_X_ee& ForceTransforms::Type_fr_j6_X_ee::update(const state_t& q)
{
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    (*this)(0,1) = (-cos_q_j6 * sin_q_j8)-(sin_q_j6 * cos_q_j8);
    (*this)(0,3) = ((((- tz_j7- tz_ee) * cos_q_j6)-(cos_q_j6 *  q(J7))) * sin_q_j8)+((((- tz_j7- tz_ee) * sin_q_j6)-(sin_q_j6 *  q(J7))) * cos_q_j8);
    (*this)(0,4) = (((sin_q_j6 *  q(J7))+(( tz_j7+ tz_ee) * sin_q_j6)) * sin_q_j8)+((((- tz_j7- tz_ee) * cos_q_j6)-(cos_q_j6 *  q(J7))) * cos_q_j8);
    (*this)(0,5) =  tx_j8 * sin_q_j6;
    (*this)(1,0) = (cos_q_j6 * sin_q_j8)+(sin_q_j6 * cos_q_j8);
    (*this)(1,1) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    (*this)(1,3) = ((((- tz_j7- tz_ee) * sin_q_j6)-(sin_q_j6 *  q(J7))) * sin_q_j8)+(((cos_q_j6 *  q(J7))+(( tz_j7+ tz_ee) * cos_q_j6)) * cos_q_j8);
    (*this)(1,4) = ((((- tz_j7- tz_ee) * cos_q_j6)-(cos_q_j6 *  q(J7))) * sin_q_j8)+((((- tz_j7- tz_ee) * sin_q_j6)-(sin_q_j6 *  q(J7))) * cos_q_j8);
    (*this)(1,5) = - tx_j8 * cos_q_j6;
    (*this)(2,3) =  tx_j8 * sin_q_j8;
    (*this)(2,4) =  tx_j8 * cos_q_j8;
    (*this)(3,3) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    (*this)(3,4) = (-cos_q_j6 * sin_q_j8)-(sin_q_j6 * cos_q_j8);
    (*this)(4,3) = (cos_q_j6 * sin_q_j8)+(sin_q_j6 * cos_q_j8);
    (*this)(4,4) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    return *this;
}
ForceTransforms::Type_fr_base0_X_link1_end::Type_fr_base0_X_link1_end()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base0_X_link1_end& ForceTransforms::Type_fr_base0_X_link1_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,1) = -sin_q_j1;
    (*this)(0,3) = - tz_link1_end * sin_q_j1;
    (*this)(0,4) = - tz_link1_end * cos_q_j1;
    (*this)(1,0) = sin_q_j1;
    (*this)(1,1) = cos_q_j1;
    (*this)(1,3) =  tz_link1_end * cos_q_j1;
    (*this)(1,4) = - tz_link1_end * sin_q_j1;
    (*this)(3,3) = cos_q_j1;
    (*this)(3,4) = -sin_q_j1;
    (*this)(4,3) = sin_q_j1;
    (*this)(4,4) = cos_q_j1;
    return *this;
}
ForceTransforms::Type_fr_base0_X_link2_end::Type_fr_base0_X_link2_end()
{
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_link2_end& ForceTransforms::Type_fr_base0_X_link2_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j1 * cos_q_j2;
    (*this)(0,1) = -cos_q_j1 * sin_q_j2;
    (*this)(0,2) = sin_q_j1;
    (*this)(0,3) = (- tz_link2_end * cos_q_j1 * sin_q_j2)-( tz_j2 * sin_q_j1 * cos_q_j2);
    (*this)(0,4) = ( tz_j2 * sin_q_j1 * sin_q_j2)-( tz_link2_end * cos_q_j1 * cos_q_j2);
    (*this)(0,5) =  tz_j2 * cos_q_j1;
    (*this)(1,0) = sin_q_j1 * cos_q_j2;
    (*this)(1,1) = -sin_q_j1 * sin_q_j2;
    (*this)(1,2) = -cos_q_j1;
    (*this)(1,3) = ( tz_j2 * cos_q_j1 * cos_q_j2)-( tz_link2_end * sin_q_j1 * sin_q_j2);
    (*this)(1,4) = (- tz_j2 * cos_q_j1 * sin_q_j2)-( tz_link2_end * sin_q_j1 * cos_q_j2);
    (*this)(1,5) =  tz_j2 * sin_q_j1;
    (*this)(2,0) = sin_q_j2;
    (*this)(2,1) = cos_q_j2;
    (*this)(2,3) =  tz_link2_end * cos_q_j2;
    (*this)(2,4) = - tz_link2_end * sin_q_j2;
    (*this)(3,3) = cos_q_j1 * cos_q_j2;
    (*this)(3,4) = -cos_q_j1 * sin_q_j2;
    (*this)(3,5) = sin_q_j1;
    (*this)(4,3) = sin_q_j1 * cos_q_j2;
    (*this)(4,4) = -sin_q_j1 * sin_q_j2;
    (*this)(4,5) = -cos_q_j1;
    (*this)(5,3) = sin_q_j2;
    (*this)(5,4) = cos_q_j2;
    return *this;
}
ForceTransforms::Type_fr_base0_X_link3_end::Type_fr_base0_X_link3_end()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_link3_end& ForceTransforms::Type_fr_base0_X_link3_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,1) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = ((( tz_link3_end * cos_q_j1 * sin_q_j2)-( tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+((( tz_j2 * sin_q_j1 * sin_q_j2)+( tz_link3_end * cos_q_j1 * cos_q_j2)) * cos_q_j3);
    (*this)(0,4) = (((- tz_j2 * sin_q_j1 * sin_q_j2)-( tz_link3_end * cos_q_j1 * cos_q_j2)) * sin_q_j3)+((( tz_link3_end * cos_q_j1 * sin_q_j2)-( tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3);
    (*this)(0,5) = (- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1);
    (*this)(1,0) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,1) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = ((( tz_link3_end * sin_q_j1 * sin_q_j2)+( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)+((( tz_link3_end * sin_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1 * sin_q_j2)) * cos_q_j3);
    (*this)(1,4) = ((( tz_j2 * cos_q_j1 * sin_q_j2)-( tz_link3_end * sin_q_j1 * cos_q_j2)) * sin_q_j3)+((( tz_link3_end * sin_q_j1 * sin_q_j2)+( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3);
    (*this)(1,5) = (- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1);
    (*this)(2,0) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,1) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(2,3) = ( tz_link3_end * sin_q_j2 * cos_q_j3)-( tz_link3_end * cos_q_j2 * sin_q_j3);
    (*this)(2,4) = (- tz_link3_end * sin_q_j2 * sin_q_j3)-( tz_link3_end * cos_q_j2 * cos_q_j3);
    (*this)(2,5) = - ty_j3 * sin_q_j2;
    (*this)(3,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(3,4) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,5) = -sin_q_j1;
    (*this)(4,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,4) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,5) = cos_q_j1;
    (*this)(5,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(5,4) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    return *this;
}
ForceTransforms::Type_fr_base0_X_link4_end::Type_fr_base0_X_link4_end()
{
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_link4_end& ForceTransforms::Type_fr_base0_X_link4_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(0,1) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,2) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,3) = (((- tz_link4_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_link4_end * sin_q_j1)) * cos_q_j4);
    (*this)(0,4) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_link4_end * sin_q_j1)) * sin_q_j4)+(((- tz_link4_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(0,5) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,0) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(1,1) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,2) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,3) = (((- tz_link4_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_link4_end * cos_q_j1)) * cos_q_j4);
    (*this)(1,4) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_link4_end * cos_q_j1)) * sin_q_j4)+(((- tz_link4_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(1,5) = ((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(2,0) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(2,1) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,2) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,3) = (( tz_link4_end * cos_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4;
    (*this)(2,4) = (( tz_link4_end * cos_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(3,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(3,5) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,3) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(4,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(4,5) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(5,3) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(5,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(5,5) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    return *this;
}
ForceTransforms::Type_fr_base0_X_link5_end::Type_fr_base0_X_link5_end()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_link5_end& ForceTransforms::Type_fr_base0_X_link5_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(0,1) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(0,2) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,3) = ((( tz_link5_end * sin_q_j1 * sin_q_j4)+(((- tz_link5_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)-( tz_link5_end * cos_q_j1 * cos_q_j2 * sin_q_j3)+( tz_link5_end * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,4) = ((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)+( tz_link5_end * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_link5_end * sin_q_j1 * sin_q_j4)+(((- tz_link5_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,5) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(1,0) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,1) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(1,2) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,3) = (((- tz_link5_end * cos_q_j1 * sin_q_j4)+(((- tz_link5_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)-( tz_link5_end * sin_q_j1 * cos_q_j2 * sin_q_j3)+( tz_link5_end * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,4) = ((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)+( tz_link5_end * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((- tz_link5_end * cos_q_j1 * sin_q_j4)+(((- tz_link5_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,5) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(2,0) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,1) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,2) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,3) = ((( tz_link5_end * cos_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+((((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4)-( tz_link5_end * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,4) = (((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4)+( tz_link5_end * sin_q_j2 * sin_q_j3)+( tz_link5_end * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_link5_end * cos_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,5) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(3,4) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(3,5) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(4,3) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(4,4) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(4,5) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(5,3) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(5,4) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,5) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_j1::Type_fr_base0_X_fr_j1()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base0_X_fr_j1& ForceTransforms::Type_fr_base0_X_fr_j1::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_j2::Type_fr_base0_X_fr_j2()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_j2& ForceTransforms::Type_fr_base0_X_fr_j2::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,2) = sin_q_j1;
    (*this)(0,3) = - tz_j2 * sin_q_j1;
    (*this)(0,5) =  tz_j2 * cos_q_j1;
    (*this)(1,0) = sin_q_j1;
    (*this)(1,2) = -cos_q_j1;
    (*this)(1,3) =  tz_j2 * cos_q_j1;
    (*this)(1,5) =  tz_j2 * sin_q_j1;
    (*this)(3,3) = cos_q_j1;
    (*this)(3,5) = sin_q_j1;
    (*this)(4,3) = sin_q_j1;
    (*this)(4,5) = -cos_q_j1;
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_j3::Type_fr_base0_X_fr_j3()
{
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_j3& ForceTransforms::Type_fr_base0_X_fr_j3::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = -cos_q_j1 * sin_q_j2;
    (*this)(0,1) = cos_q_j1 * cos_q_j2;
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) =  tz_j2 * sin_q_j1 * sin_q_j2;
    (*this)(0,4) = (- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1);
    (*this)(0,5) = (- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1);
    (*this)(1,0) = -sin_q_j1 * sin_q_j2;
    (*this)(1,1) = sin_q_j1 * cos_q_j2;
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = - tz_j2 * cos_q_j1 * sin_q_j2;
    (*this)(1,4) = ( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1);
    (*this)(1,5) = (- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1);
    (*this)(2,0) = cos_q_j2;
    (*this)(2,1) = sin_q_j2;
    (*this)(2,5) = - ty_j3 * sin_q_j2;
    (*this)(3,3) = -cos_q_j1 * sin_q_j2;
    (*this)(3,4) = cos_q_j1 * cos_q_j2;
    (*this)(3,5) = -sin_q_j1;
    (*this)(4,3) = -sin_q_j1 * sin_q_j2;
    (*this)(4,4) = sin_q_j1 * cos_q_j2;
    (*this)(4,5) = cos_q_j1;
    (*this)(5,3) = cos_q_j2;
    (*this)(5,4) = sin_q_j2;
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_j4::Type_fr_base0_X_fr_j4()
{
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_j4& ForceTransforms::Type_fr_base0_X_fr_j4::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(0,1) = -sin_q_j1;
    (*this)(0,2) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,3) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3);
    (*this)(0,4) = (- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1);
    (*this)(0,5) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,0) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(1,1) = cos_q_j1;
    (*this)(1,2) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,3) = ( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3);
    (*this)(1,4) = (- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1);
    (*this)(1,5) = ((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(2,0) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(2,2) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,4) = - ty_j3 * sin_q_j2;
    (*this)(3,3) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,4) = -sin_q_j1;
    (*this)(3,5) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,3) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,4) = cos_q_j1;
    (*this)(4,5) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(5,3) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(5,5) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_j5::Type_fr_base0_X_fr_j5()
{
    (*this)(2,4) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_j5& ForceTransforms::Type_fr_base0_X_fr_j5::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(0,1) = (cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(0,2) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,3) = (((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4);
    (*this)(0,4) = ((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,5) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(1,0) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(1,1) = (sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(1,2) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,3) = (((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4);
    (*this)(1,4) = (((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,5) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(2,0) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(2,1) = (-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3);
    (*this)(2,2) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,3) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4;
    (*this)(2,5) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(3,4) = (cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(3,5) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(4,3) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(4,4) = (sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(4,5) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(5,3) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(5,4) = (-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3);
    (*this)(5,5) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_j6::Type_fr_base0_X_fr_j6()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_j6& ForceTransforms::Type_fr_base0_X_fr_j6::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(0,1) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = ((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5);
    (*this)(0,4) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(0,5) = (((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,1) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = (((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,4) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(1,5) = (((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,1) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5;
    (*this)(2,4) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(2,5) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5;
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(3,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(4,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,3) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(5,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_j7::Type_fr_base0_X_fr_j7()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_j7& ForceTransforms::Type_fr_base0_X_fr_j7::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = ((((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j4)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_j7 * sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,4) = (((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+((( tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j7 * sin_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j4)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6);
    (*this)(0,5) = (((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = ((((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * cos_q_j1 * sin_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j7 * cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,4) = ((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_j7 * cos_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * cos_q_j1 * sin_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6);
    (*this)(1,5) = (((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(2,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = ((((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * sin_q_j6)+((((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * cos_q_j6);
    (*this)(2,4) = (((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((( tz_j7 * sin_q_j2 * cos_q_j3)-( tz_j7 * cos_q_j2 * sin_q_j3)) * sin_q_j4)) * sin_q_j6)+((((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * cos_q_j6);
    (*this)(2,5) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5;
    (*this)(3,3) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(3,4) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(4,4) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,3) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(5,4) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_j8::Type_fr_base0_X_fr_j8()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_j8& ForceTransforms::Type_fr_base0_X_fr_j8::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = (((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+((((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j4)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_j7 * sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,4) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)+((( tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j7 * sin_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j4)+(((- tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,5) = ((((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tx_j8 * sin_q_j1 * cos_q_j4)) * cos_q_j6)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = (((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) *  q(J7))+((((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * cos_q_j1 * sin_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+(((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j7 * cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,4) = (((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) *  q(J7))+((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)+((( tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_j7 * cos_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * cos_q_j1 * sin_q_j4)) * cos_q_j5)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,5) = ((((( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * cos_q_j1 * cos_q_j4)) * cos_q_j6)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(2,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = (((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) *  q(J7))+((((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * sin_q_j6)+((((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * sin_q_j4)) * cos_q_j6);
    (*this)(2,4) = (((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) *  q(J7))+(((((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5)+((( tz_j7 * sin_q_j2 * cos_q_j3)-( tz_j7 * cos_q_j2 * sin_q_j3)) * sin_q_j4)) * sin_q_j6)+((((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tz_j7 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4)) * cos_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+((( tx_j8 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,5) = (((((- tx_j8 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * sin_q_j4 * cos_q_j6)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5);
    (*this)(3,3) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(3,4) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(4,4) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,3) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(5,4) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_link6::Type_fr_base0_X_fr_link6()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_link6& ForceTransforms::Type_fr_base0_X_fr_link6::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = ((((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * sin_q_j6)+((((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,4) = (((((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+((((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4)) * cos_q_j6);
    (*this)(0,5) = (((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,0) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = (((((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * sin_q_j6)+(((((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,4) = ((((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4)) * cos_q_j6);
    (*this)(1,5) = (((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,0) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(2,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = ((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4 * sin_q_j6)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5 * cos_q_j6);
    (*this)(2,4) = (((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5 * sin_q_j6)+((( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4 * cos_q_j6);
    (*this)(2,5) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5;
    (*this)(3,3) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(3,4) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(3,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,3) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(4,4) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(4,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,3) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(5,4) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(5,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_link1::Type_fr_base0_X_fr_link1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base0_X_fr_link1& ForceTransforms::Type_fr_base0_X_fr_link1::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,1) = -sin_q_j1;
    (*this)(1,0) = sin_q_j1;
    (*this)(1,1) = cos_q_j1;
    (*this)(3,3) = cos_q_j1;
    (*this)(3,4) = -sin_q_j1;
    (*this)(4,3) = sin_q_j1;
    (*this)(4,4) = cos_q_j1;
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_link2::Type_fr_base0_X_fr_link2()
{
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_link2& ForceTransforms::Type_fr_base0_X_fr_link2::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j1 * cos_q_j2;
    (*this)(0,1) = -cos_q_j1 * sin_q_j2;
    (*this)(0,2) = sin_q_j1;
    (*this)(0,3) = - tz_j2 * sin_q_j1 * cos_q_j2;
    (*this)(0,4) =  tz_j2 * sin_q_j1 * sin_q_j2;
    (*this)(0,5) =  tz_j2 * cos_q_j1;
    (*this)(1,0) = sin_q_j1 * cos_q_j2;
    (*this)(1,1) = -sin_q_j1 * sin_q_j2;
    (*this)(1,2) = -cos_q_j1;
    (*this)(1,3) =  tz_j2 * cos_q_j1 * cos_q_j2;
    (*this)(1,4) = - tz_j2 * cos_q_j1 * sin_q_j2;
    (*this)(1,5) =  tz_j2 * sin_q_j1;
    (*this)(2,0) = sin_q_j2;
    (*this)(2,1) = cos_q_j2;
    (*this)(3,3) = cos_q_j1 * cos_q_j2;
    (*this)(3,4) = -cos_q_j1 * sin_q_j2;
    (*this)(3,5) = sin_q_j1;
    (*this)(4,3) = sin_q_j1 * cos_q_j2;
    (*this)(4,4) = -sin_q_j1 * sin_q_j2;
    (*this)(4,5) = -cos_q_j1;
    (*this)(5,3) = sin_q_j2;
    (*this)(5,4) = cos_q_j2;
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_link3::Type_fr_base0_X_fr_link3()
{
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_link3& ForceTransforms::Type_fr_base0_X_fr_link3::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,1) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,4) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3);
    (*this)(0,5) = (- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1);
    (*this)(1,0) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,1) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = ((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,4) = ( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3);
    (*this)(1,5) = (- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1);
    (*this)(2,0) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,1) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(2,5) = - ty_j3 * sin_q_j2;
    (*this)(3,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(3,4) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,5) = -sin_q_j1;
    (*this)(4,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,4) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,5) = cos_q_j1;
    (*this)(5,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(5,4) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_link4::Type_fr_base0_X_fr_link4()
{
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_link4& ForceTransforms::Type_fr_base0_X_fr_link4::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(0,1) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,2) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,3) = (((- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)) * cos_q_j4);
    (*this)(0,4) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)) * sin_q_j4)+(((- ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(0,5) = (((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * sin_q_j3)+( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,0) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(1,1) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,2) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,3) = (((- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)) * cos_q_j4);
    (*this)(1,4) = (((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)) * sin_q_j4)+(((- ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(1,5) = ((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * sin_q_j3)-( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(2,0) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(2,1) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,2) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,3) = - ty_j3 * sin_q_j2 * sin_q_j4;
    (*this)(2,4) = - ty_j3 * sin_q_j2 * cos_q_j4;
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(3,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(3,5) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(4,3) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(4,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(4,5) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(5,3) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(5,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(5,5) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    return *this;
}
ForceTransforms::Type_fr_base0_X_fr_link5::Type_fr_base0_X_fr_link5()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base0_X_fr_link5& ForceTransforms::Type_fr_base0_X_fr_link5::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(0,1) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(0,2) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,3) = ((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * sin_q_j4)+(((- tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * sin_q_j1 * cos_q_j2)-( ty_j3 * sin_q_j1)) * cos_q_j3)-( tz_j5 * sin_q_j1)) * cos_q_j4)) * cos_q_j5);
    (*this)(0,4) = ((((( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j1 * cos_q_j2)+( tz_j2 * cos_q_j1)) * sin_q_j4)+((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * cos_q_j4)) * sin_q_j5)+((((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * sin_q_j3)-( tz_j2 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,5) = ((( tz_j2 * sin_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * sin_q_j1 * cos_q_j2)+( ty_j3 * sin_q_j1)) * cos_q_j3)+( tz_j5 * sin_q_j1)) * sin_q_j4)+(((- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2)-( tz_j2 * cos_q_j1)) * cos_q_j4);
    (*this)(1,0) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,1) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(1,2) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,3) = (((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * sin_q_j4)+((( tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+((( tz_j2 * cos_q_j1 * cos_q_j2)+( ty_j3 * cos_q_j1)) * cos_q_j3)+( tz_j5 * cos_q_j1)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,4) = ((((( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * cos_q_j2)+( tz_j2 * sin_q_j1)) * sin_q_j4)+(((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * cos_q_j4)) * sin_q_j5)+(((((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * sin_q_j3)+( tz_j2 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,5) = (((- tz_j2 * cos_q_j1 * sin_q_j2 * sin_q_j3)+(((- tz_j2 * cos_q_j1 * cos_q_j2)-( ty_j3 * cos_q_j1)) * cos_q_j3)-( tz_j5 * cos_q_j1)) * sin_q_j4)+(((- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2)-( tz_j2 * sin_q_j1)) * cos_q_j4);
    (*this)(2,0) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,1) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,2) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,3) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * sin_q_j4 * cos_q_j5;
    (*this)(2,4) = ((- tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j2)) * sin_q_j4 * sin_q_j5;
    (*this)(2,5) = (( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2)) * cos_q_j4;
    (*this)(3,3) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(3,4) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(3,5) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(4,3) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(4,4) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(4,5) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(5,3) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(5,4) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,5) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    return *this;
}
ForceTransforms::Type_fr_link1_X_fr_base0::Type_fr_link1_X_fr_base0()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_link1_X_fr_base0& ForceTransforms::Type_fr_link1_X_fr_base0::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,1) = sin_q_j1;
    (*this)(1,0) = -sin_q_j1;
    (*this)(1,1) = cos_q_j1;
    (*this)(3,3) = cos_q_j1;
    (*this)(3,4) = sin_q_j1;
    (*this)(4,3) = -sin_q_j1;
    (*this)(4,4) = cos_q_j1;
    return *this;
}
ForceTransforms::Type_fr_link2_X_fr_link1::Type_fr_link2_X_fr_link1()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_j2;    // Maxima DSL: _k__tz_j2
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_link2_X_fr_link1& ForceTransforms::Type_fr_link2_X_fr_link1::update(const state_t& q)
{
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j2;
    (*this)(0,2) = sin_q_j2;
    (*this)(0,4) =  tz_j2 * cos_q_j2;
    (*this)(1,0) = -sin_q_j2;
    (*this)(1,2) = cos_q_j2;
    (*this)(1,4) = - tz_j2 * sin_q_j2;
    (*this)(3,3) = cos_q_j2;
    (*this)(3,5) = sin_q_j2;
    (*this)(4,3) = -sin_q_j2;
    (*this)(4,5) = cos_q_j2;
    return *this;
}
ForceTransforms::Type_fr_link1_X_fr_link2::Type_fr_link1_X_fr_link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) =  tz_j2;    // Maxima DSL: _k__tz_j2
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_link1_X_fr_link2& ForceTransforms::Type_fr_link1_X_fr_link2::update(const state_t& q)
{
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j2;
    (*this)(0,1) = -sin_q_j2;
    (*this)(1,3) =  tz_j2 * cos_q_j2;
    (*this)(1,4) = - tz_j2 * sin_q_j2;
    (*this)(2,0) = sin_q_j2;
    (*this)(2,1) = cos_q_j2;
    (*this)(3,3) = cos_q_j2;
    (*this)(3,4) = -sin_q_j2;
    (*this)(5,3) = sin_q_j2;
    (*this)(5,4) = cos_q_j2;
    return *this;
}
ForceTransforms::Type_fr_link3_X_fr_link2::Type_fr_link3_X_fr_link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = - ty_j3;    // Maxima DSL: -_k__ty_j3
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_link3_X_fr_link2& ForceTransforms::Type_fr_link3_X_fr_link2::update(const state_t& q)
{
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = sin_q_j3;
    (*this)(0,1) = cos_q_j3;
    (*this)(0,5) = - ty_j3 * sin_q_j3;
    (*this)(1,0) = cos_q_j3;
    (*this)(1,1) = -sin_q_j3;
    (*this)(1,5) = - ty_j3 * cos_q_j3;
    (*this)(3,3) = sin_q_j3;
    (*this)(3,4) = cos_q_j3;
    (*this)(4,3) = cos_q_j3;
    (*this)(4,4) = -sin_q_j3;
    return *this;
}
ForceTransforms::Type_fr_link2_X_fr_link3::Type_fr_link2_X_fr_link3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = - ty_j3;    // Maxima DSL: -_k__ty_j3
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = -1.0;
}

const ForceTransforms::Type_fr_link2_X_fr_link3& ForceTransforms::Type_fr_link2_X_fr_link3::update(const state_t& q)
{
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = sin_q_j3;
    (*this)(0,1) = cos_q_j3;
    (*this)(1,0) = cos_q_j3;
    (*this)(1,1) = -sin_q_j3;
    (*this)(2,3) = - ty_j3 * sin_q_j3;
    (*this)(2,4) = - ty_j3 * cos_q_j3;
    (*this)(3,3) = sin_q_j3;
    (*this)(3,4) = cos_q_j3;
    (*this)(4,3) = cos_q_j3;
    (*this)(4,4) = -sin_q_j3;
    return *this;
}
ForceTransforms::Type_fr_link4_X_fr_link3::Type_fr_link4_X_fr_link3()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_link4_X_fr_link3& ForceTransforms::Type_fr_link4_X_fr_link3::update(const state_t& q)
{
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,1) = cos_q_j4;
    (*this)(0,2) = sin_q_j4;
    (*this)(1,1) = -sin_q_j4;
    (*this)(1,2) = cos_q_j4;
    (*this)(3,4) = cos_q_j4;
    (*this)(3,5) = sin_q_j4;
    (*this)(4,4) = -sin_q_j4;
    (*this)(4,5) = cos_q_j4;
    return *this;
}
ForceTransforms::Type_fr_link3_X_fr_link4::Type_fr_link3_X_fr_link4()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_link3_X_fr_link4& ForceTransforms::Type_fr_link3_X_fr_link4::update(const state_t& q)
{
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(1,0) = cos_q_j4;
    (*this)(1,1) = -sin_q_j4;
    (*this)(2,0) = sin_q_j4;
    (*this)(2,1) = cos_q_j4;
    (*this)(4,3) = cos_q_j4;
    (*this)(4,4) = -sin_q_j4;
    (*this)(5,3) = sin_q_j4;
    (*this)(5,4) = cos_q_j4;
    return *this;
}
ForceTransforms::Type_fr_link5_X_fr_link4::Type_fr_link5_X_fr_link4()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tz_j5;    // Maxima DSL: -_k__tz_j5
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_link5_X_fr_link4& ForceTransforms::Type_fr_link5_X_fr_link4::update(const state_t& q)
{
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = cos_q_j5;
    (*this)(0,2) = -sin_q_j5;
    (*this)(0,4) =  tz_j5 * cos_q_j5;
    (*this)(1,0) = -sin_q_j5;
    (*this)(1,2) = -cos_q_j5;
    (*this)(1,4) = - tz_j5 * sin_q_j5;
    (*this)(3,3) = cos_q_j5;
    (*this)(3,5) = -sin_q_j5;
    (*this)(4,3) = -sin_q_j5;
    (*this)(4,5) = -cos_q_j5;
    return *this;
}
ForceTransforms::Type_fr_link4_X_fr_link5::Type_fr_link4_X_fr_link5()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = - tz_j5;    // Maxima DSL: -_k__tz_j5
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_link4_X_fr_link5& ForceTransforms::Type_fr_link4_X_fr_link5::update(const state_t& q)
{
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = cos_q_j5;
    (*this)(0,1) = -sin_q_j5;
    (*this)(1,3) =  tz_j5 * cos_q_j5;
    (*this)(1,4) = - tz_j5 * sin_q_j5;
    (*this)(2,0) = -sin_q_j5;
    (*this)(2,1) = -cos_q_j5;
    (*this)(3,3) = cos_q_j5;
    (*this)(3,4) = -sin_q_j5;
    (*this)(5,3) = -sin_q_j5;
    (*this)(5,4) = -cos_q_j5;
    return *this;
}
ForceTransforms::Type_fr_link6_X_fr_link5::Type_fr_link6_X_fr_link5()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_link6_X_fr_link5& ForceTransforms::Type_fr_link6_X_fr_link5::update(const state_t& q)
{
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = cos_q_j6;
    (*this)(0,2) = sin_q_j6;
    (*this)(1,0) = -sin_q_j6;
    (*this)(1,2) = cos_q_j6;
    (*this)(3,3) = cos_q_j6;
    (*this)(3,5) = sin_q_j6;
    (*this)(4,3) = -sin_q_j6;
    (*this)(4,5) = cos_q_j6;
    return *this;
}
ForceTransforms::Type_fr_link5_X_fr_link6::Type_fr_link5_X_fr_link6()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_link5_X_fr_link6& ForceTransforms::Type_fr_link5_X_fr_link6::update(const state_t& q)
{
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = cos_q_j6;
    (*this)(0,1) = -sin_q_j6;
    (*this)(2,0) = sin_q_j6;
    (*this)(2,1) = cos_q_j6;
    (*this)(3,3) = cos_q_j6;
    (*this)(3,4) = -sin_q_j6;
    (*this)(5,3) = sin_q_j6;
    (*this)(5,4) = cos_q_j6;
    return *this;
}
ForceTransforms::Type_fr_link7_X_fr_link6::Type_fr_link7_X_fr_link6()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_link7_X_fr_link6& ForceTransforms::Type_fr_link7_X_fr_link6::update(const state_t& q)
{
    (*this)(0,4) =  q(J7)+ tz_j7;
    (*this)(1,3) = - q(J7)- tz_j7;
    return *this;
}
ForceTransforms::Type_fr_link6_X_fr_link7::Type_fr_link6_X_fr_link7()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_link6_X_fr_link7& ForceTransforms::Type_fr_link6_X_fr_link7::update(const state_t& q)
{
    (*this)(0,4) = - q(J7)- tz_j7;
    (*this)(1,3) =  q(J7)+ tz_j7;
    return *this;
}
ForceTransforms::Type_fr_link8_X_fr_link7::Type_fr_link8_X_fr_link7()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_j8;    // Maxima DSL: -_k__tx_j8
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_link8_X_fr_link7& ForceTransforms::Type_fr_link8_X_fr_link7::update(const state_t& q)
{
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = cos_q_j8;
    (*this)(0,1) = sin_q_j8;
    (*this)(0,5) =  tx_j8 * sin_q_j8;
    (*this)(1,0) = -sin_q_j8;
    (*this)(1,1) = cos_q_j8;
    (*this)(1,5) =  tx_j8 * cos_q_j8;
    (*this)(3,3) = cos_q_j8;
    (*this)(3,4) = sin_q_j8;
    (*this)(4,3) = -sin_q_j8;
    (*this)(4,4) = cos_q_j8;
    return *this;
}
ForceTransforms::Type_fr_link7_X_fr_link8::Type_fr_link7_X_fr_link8()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_j8;    // Maxima DSL: -_k__tx_j8
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_link7_X_fr_link8& ForceTransforms::Type_fr_link7_X_fr_link8::update(const state_t& q)
{
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = cos_q_j8;
    (*this)(0,1) = -sin_q_j8;
    (*this)(1,0) = sin_q_j8;
    (*this)(1,1) = cos_q_j8;
    (*this)(2,3) =  tx_j8 * sin_q_j8;
    (*this)(2,4) =  tx_j8 * cos_q_j8;
    (*this)(3,3) = cos_q_j8;
    (*this)(3,4) = -sin_q_j8;
    (*this)(4,3) = sin_q_j8;
    (*this)(4,4) = cos_q_j8;
    return *this;
}

HomogeneousTransforms::Type_fr_base0_X_ee::Type_fr_base0_X_ee()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_ee& HomogeneousTransforms::Type_fr_base0_X_ee::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = (((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(0,1) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+(((((- tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * sin_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j4)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(1,0) = (((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * sin_q_j8)+(((((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(1,1) = (((((((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6)) * cos_q_j8);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = (((((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+(((((- tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tx_j8 * cos_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j4)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = (((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * sin_q_j8)+(((((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * cos_q_j8);
    (*this)(2,1) = (((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+(((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)) * sin_q_j8)+(((((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6)) * cos_q_j8);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = (((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+((( tx_j8 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((- tx_j8 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_j6_X_ee::Type_fr_j6_X_ee()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_j6_X_ee& HomogeneousTransforms::Type_fr_j6_X_ee::update(const state_t& q)
{
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    (*this)(0,1) = (-cos_q_j6 * sin_q_j8)-(sin_q_j6 * cos_q_j8);
    (*this)(0,3) =  tx_j8 * cos_q_j6;
    (*this)(1,0) = (cos_q_j6 * sin_q_j8)+(sin_q_j6 * cos_q_j8);
    (*this)(1,1) = (cos_q_j6 * cos_q_j8)-(sin_q_j6 * sin_q_j8);
    (*this)(1,3) =  tx_j8 * sin_q_j6;
    (*this)(2,3) =  q(J7)+ tz_j7+ tz_ee;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_link1_end::Type_fr_base0_X_link1_end()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_link1_end;    // Maxima DSL: _k__tz_link1_end
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_link1_end& HomogeneousTransforms::Type_fr_base0_X_link1_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,1) = -sin_q_j1;
    (*this)(1,0) = sin_q_j1;
    (*this)(1,1) = cos_q_j1;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_link2_end::Type_fr_base0_X_link2_end()
{
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_j2;    // Maxima DSL: _k__tz_j2
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_link2_end& HomogeneousTransforms::Type_fr_base0_X_link2_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j1 * cos_q_j2;
    (*this)(0,1) = -cos_q_j1 * sin_q_j2;
    (*this)(0,2) = sin_q_j1;
    (*this)(0,3) =  tz_link2_end * sin_q_j1;
    (*this)(1,0) = sin_q_j1 * cos_q_j2;
    (*this)(1,1) = -sin_q_j1 * sin_q_j2;
    (*this)(1,2) = -cos_q_j1;
    (*this)(1,3) = - tz_link2_end * cos_q_j1;
    (*this)(2,0) = sin_q_j2;
    (*this)(2,1) = cos_q_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_link3_end::Type_fr_base0_X_link3_end()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_link3_end& HomogeneousTransforms::Type_fr_base0_X_link3_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,1) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = (- ty_j3 * cos_q_j1 * sin_q_j2)-( tz_link3_end * sin_q_j1);
    (*this)(1,0) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,1) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = ( tz_link3_end * cos_q_j1)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,1) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(2,3) = ( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_link4_end::Type_fr_base0_X_link4_end()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_link4_end& HomogeneousTransforms::Type_fr_base0_X_link4_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(0,1) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,2) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,3) = ( tz_link4_end * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_link4_end * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(1,0) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(1,1) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,2) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,3) = ( tz_link4_end * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(2,1) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,2) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,3) = ( tz_link4_end * sin_q_j2 * sin_q_j3)+( tz_link4_end * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_link5_end::Type_fr_base0_X_link5_end()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_link5_end& HomogeneousTransforms::Type_fr_base0_X_link5_end::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(0,1) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(0,2) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,3) = (((- tz_link5_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_link5_end * sin_q_j1 * cos_q_j4)+( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(1,0) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,1) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(1,2) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,3) = (((- tz_link5_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_link5_end * cos_q_j1 * cos_q_j4)+( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,1) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,2) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,3) = ((( tz_link5_end * cos_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j5 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_j1::Type_fr_base0_X_fr_j1()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_j1& HomogeneousTransforms::Type_fr_base0_X_fr_j1::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_j2::Type_fr_base0_X_fr_j2()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_j2;    // Maxima DSL: _k__tz_j2
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_j2& HomogeneousTransforms::Type_fr_base0_X_fr_j2::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,2) = sin_q_j1;
    (*this)(1,0) = sin_q_j1;
    (*this)(1,2) = -cos_q_j1;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_j3::Type_fr_base0_X_fr_j3()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_j3& HomogeneousTransforms::Type_fr_base0_X_fr_j3::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = -cos_q_j1 * sin_q_j2;
    (*this)(0,1) = cos_q_j1 * cos_q_j2;
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = - ty_j3 * cos_q_j1 * sin_q_j2;
    (*this)(1,0) = -sin_q_j1 * sin_q_j2;
    (*this)(1,1) = sin_q_j1 * cos_q_j2;
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = - ty_j3 * sin_q_j1 * sin_q_j2;
    (*this)(2,0) = cos_q_j2;
    (*this)(2,1) = sin_q_j2;
    (*this)(2,3) = ( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_j4::Type_fr_base0_X_fr_j4()
{
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_j4& HomogeneousTransforms::Type_fr_base0_X_fr_j4::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(0,1) = -sin_q_j1;
    (*this)(0,2) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,3) = - ty_j3 * cos_q_j1 * sin_q_j2;
    (*this)(1,0) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(1,1) = cos_q_j1;
    (*this)(1,2) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,3) = - ty_j3 * sin_q_j1 * sin_q_j2;
    (*this)(2,0) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(2,2) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,3) = ( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_j5::Type_fr_base0_X_fr_j5()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_j5& HomogeneousTransforms::Type_fr_base0_X_fr_j5::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(0,1) = (cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(0,2) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,3) = ( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(1,0) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(1,1) = (sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3);
    (*this)(1,2) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,3) = ( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(2,1) = (-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3);
    (*this)(2,2) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,3) = ( tz_j5 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_j6::Type_fr_base0_X_fr_j6()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_j6& HomogeneousTransforms::Type_fr_base0_X_fr_j6::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(0,1) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = ( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(1,0) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,1) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = ( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,1) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = ( tz_j5 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_j7::Type_fr_base0_X_fr_j7()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_j7& HomogeneousTransforms::Type_fr_base0_X_fr_j7::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = ((((( tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * sin_q_j1 * sin_q_j4)) * sin_q_j5)+((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(1,0) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = ((( tz_j7 * cos_q_j1 * sin_q_j4)+((( tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(2,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = ((( tz_j7 * sin_q_j2 * cos_q_j3)-( tz_j7 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_j8::Type_fr_base0_X_fr_j8()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_j8& HomogeneousTransforms::Type_fr_base0_X_fr_j8::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+(((((- tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * sin_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)+((((( tz_j7 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_j7 * sin_q_j1 * sin_q_j4)) * sin_q_j5)+((( tz_j7 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(1,0) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = (((((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+(((((- tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tx_j8 * cos_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * sin_q_j4)+((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6)+((( tz_j7 * cos_q_j1 * sin_q_j4)+((( tz_j7 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j7 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((( tz_j7 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j7 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(2,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = (((((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+((( tx_j8 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((- tx_j8 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)+((( tz_j7 * sin_q_j2 * cos_q_j3)-( tz_j7 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+((( tz_j7 * sin_q_j2 * sin_q_j3)+( tz_j7 * cos_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_link6::Type_fr_base0_X_fr_link6()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_link6& HomogeneousTransforms::Type_fr_base0_X_fr_link6::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j6)+(((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(0,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(0,2) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,3) = ( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(1,0) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j6)+(((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * cos_q_j6);
    (*this)(1,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(1,2) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,3) = ( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6);
    (*this)(2,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(2,2) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = ( tz_j5 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_link1::Type_fr_base0_X_fr_link1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_link1& HomogeneousTransforms::Type_fr_base0_X_fr_link1::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,1) = -sin_q_j1;
    (*this)(1,0) = sin_q_j1;
    (*this)(1,1) = cos_q_j1;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_link2::Type_fr_base0_X_fr_link2()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_j2;    // Maxima DSL: _k__tz_j2
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_link2& HomogeneousTransforms::Type_fr_base0_X_fr_link2::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j1 * cos_q_j2;
    (*this)(0,1) = -cos_q_j1 * sin_q_j2;
    (*this)(0,2) = sin_q_j1;
    (*this)(1,0) = sin_q_j1 * cos_q_j2;
    (*this)(1,1) = -sin_q_j1 * sin_q_j2;
    (*this)(1,2) = -cos_q_j1;
    (*this)(2,0) = sin_q_j2;
    (*this)(2,1) = cos_q_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_link3::Type_fr_base0_X_fr_link3()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_link3& HomogeneousTransforms::Type_fr_base0_X_fr_link3::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,1) = (cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = - ty_j3 * cos_q_j1 * sin_q_j2;
    (*this)(1,0) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,1) = (sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = - ty_j3 * sin_q_j1 * sin_q_j2;
    (*this)(2,0) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,1) = (sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3);
    (*this)(2,3) = ( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_link4::Type_fr_base0_X_fr_link4()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_link4& HomogeneousTransforms::Type_fr_base0_X_fr_link4::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4);
    (*this)(0,1) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,2) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,3) = - ty_j3 * cos_q_j1 * sin_q_j2;
    (*this)(1,0) = (cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(1,1) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,2) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,3) = - ty_j3 * sin_q_j1 * sin_q_j2;
    (*this)(2,0) = ((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4;
    (*this)(2,1) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,2) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,3) = ( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_base0_X_fr_link5::Type_fr_base0_X_fr_link5()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base0_X_fr_link5& HomogeneousTransforms::Type_fr_base0_X_fr_link5::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = (((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5);
    (*this)(0,1) = (((sin_q_j1 * sin_q_j4)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(0,2) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,3) = ( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(1,0) = (((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(1,1) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5);
    (*this)(1,2) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,3) = ( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(2,0) = (((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(2,1) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,2) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,3) = ( tz_j5 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j2 * cos_q_j3)+( ty_j3 * cos_q_j2)+ tz_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_link1_X_fr_base0::Type_fr_link1_X_fr_base0()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link1_X_fr_base0& HomogeneousTransforms::Type_fr_link1_X_fr_base0::update(const state_t& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    (*this)(0,0) = cos_q_j1;
    (*this)(0,1) = sin_q_j1;
    (*this)(1,0) = -sin_q_j1;
    (*this)(1,1) = cos_q_j1;
    return *this;
}
HomogeneousTransforms::Type_fr_link2_X_fr_link1::Type_fr_link2_X_fr_link1()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link2_X_fr_link1& HomogeneousTransforms::Type_fr_link2_X_fr_link1::update(const state_t& q)
{
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j2;
    (*this)(0,2) = sin_q_j2;
    (*this)(0,3) = - tz_j2 * sin_q_j2;
    (*this)(1,0) = -sin_q_j2;
    (*this)(1,2) = cos_q_j2;
    (*this)(1,3) = - tz_j2 * cos_q_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_link1_X_fr_link2::Type_fr_link1_X_fr_link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_j2;    // Maxima DSL: _k__tz_j2
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link1_X_fr_link2& HomogeneousTransforms::Type_fr_link1_X_fr_link2::update(const state_t& q)
{
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,0) = cos_q_j2;
    (*this)(0,1) = -sin_q_j2;
    (*this)(2,0) = sin_q_j2;
    (*this)(2,1) = cos_q_j2;
    return *this;
}
HomogeneousTransforms::Type_fr_link3_X_fr_link2::Type_fr_link3_X_fr_link2()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link3_X_fr_link2& HomogeneousTransforms::Type_fr_link3_X_fr_link2::update(const state_t& q)
{
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = sin_q_j3;
    (*this)(0,1) = cos_q_j3;
    (*this)(0,3) = - ty_j3 * cos_q_j3;
    (*this)(1,0) = cos_q_j3;
    (*this)(1,1) = -sin_q_j3;
    (*this)(1,3) =  ty_j3 * sin_q_j3;
    return *this;
}
HomogeneousTransforms::Type_fr_link2_X_fr_link3::Type_fr_link2_X_fr_link3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_j3;    // Maxima DSL: _k__ty_j3
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = -1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link2_X_fr_link3& HomogeneousTransforms::Type_fr_link2_X_fr_link3::update(const state_t& q)
{
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,0) = sin_q_j3;
    (*this)(0,1) = cos_q_j3;
    (*this)(1,0) = cos_q_j3;
    (*this)(1,1) = -sin_q_j3;
    return *this;
}
HomogeneousTransforms::Type_fr_link4_X_fr_link3::Type_fr_link4_X_fr_link3()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link4_X_fr_link3& HomogeneousTransforms::Type_fr_link4_X_fr_link3::update(const state_t& q)
{
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,1) = cos_q_j4;
    (*this)(0,2) = sin_q_j4;
    (*this)(1,1) = -sin_q_j4;
    (*this)(1,2) = cos_q_j4;
    return *this;
}
HomogeneousTransforms::Type_fr_link3_X_fr_link4::Type_fr_link3_X_fr_link4()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link3_X_fr_link4& HomogeneousTransforms::Type_fr_link3_X_fr_link4::update(const state_t& q)
{
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(1,0) = cos_q_j4;
    (*this)(1,1) = -sin_q_j4;
    (*this)(2,0) = sin_q_j4;
    (*this)(2,1) = cos_q_j4;
    return *this;
}
HomogeneousTransforms::Type_fr_link5_X_fr_link4::Type_fr_link5_X_fr_link4()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link5_X_fr_link4& HomogeneousTransforms::Type_fr_link5_X_fr_link4::update(const state_t& q)
{
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = cos_q_j5;
    (*this)(0,2) = -sin_q_j5;
    (*this)(0,3) =  tz_j5 * sin_q_j5;
    (*this)(1,0) = -sin_q_j5;
    (*this)(1,2) = -cos_q_j5;
    (*this)(1,3) =  tz_j5 * cos_q_j5;
    return *this;
}
HomogeneousTransforms::Type_fr_link4_X_fr_link5::Type_fr_link4_X_fr_link5()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_j5;    // Maxima DSL: _k__tz_j5
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link4_X_fr_link5& HomogeneousTransforms::Type_fr_link4_X_fr_link5::update(const state_t& q)
{
    Scalar sin_q_j5  = ScalarTraits::sin( q(J5) );
    Scalar cos_q_j5  = ScalarTraits::cos( q(J5) );
    (*this)(0,0) = cos_q_j5;
    (*this)(0,1) = -sin_q_j5;
    (*this)(2,0) = -sin_q_j5;
    (*this)(2,1) = -cos_q_j5;
    return *this;
}
HomogeneousTransforms::Type_fr_link6_X_fr_link5::Type_fr_link6_X_fr_link5()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link6_X_fr_link5& HomogeneousTransforms::Type_fr_link6_X_fr_link5::update(const state_t& q)
{
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = cos_q_j6;
    (*this)(0,2) = sin_q_j6;
    (*this)(1,0) = -sin_q_j6;
    (*this)(1,2) = cos_q_j6;
    return *this;
}
HomogeneousTransforms::Type_fr_link5_X_fr_link6::Type_fr_link5_X_fr_link6()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link5_X_fr_link6& HomogeneousTransforms::Type_fr_link5_X_fr_link6::update(const state_t& q)
{
    Scalar sin_q_j6  = ScalarTraits::sin( q(J6) );
    Scalar cos_q_j6  = ScalarTraits::cos( q(J6) );
    (*this)(0,0) = cos_q_j6;
    (*this)(0,1) = -sin_q_j6;
    (*this)(2,0) = sin_q_j6;
    (*this)(2,1) = cos_q_j6;
    return *this;
}
HomogeneousTransforms::Type_fr_link7_X_fr_link6::Type_fr_link7_X_fr_link6()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link7_X_fr_link6& HomogeneousTransforms::Type_fr_link7_X_fr_link6::update(const state_t& q)
{
    (*this)(2,3) = - q(J7)- tz_j7;
    return *this;
}
HomogeneousTransforms::Type_fr_link6_X_fr_link7::Type_fr_link6_X_fr_link7()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link6_X_fr_link7& HomogeneousTransforms::Type_fr_link6_X_fr_link7::update(const state_t& q)
{
    (*this)(2,3) =  q(J7)+ tz_j7;
    return *this;
}
HomogeneousTransforms::Type_fr_link8_X_fr_link7::Type_fr_link8_X_fr_link7()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link8_X_fr_link7& HomogeneousTransforms::Type_fr_link8_X_fr_link7::update(const state_t& q)
{
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = cos_q_j8;
    (*this)(0,1) = sin_q_j8;
    (*this)(0,3) = - tx_j8 * cos_q_j8;
    (*this)(1,0) = -sin_q_j8;
    (*this)(1,1) = cos_q_j8;
    (*this)(1,3) =  tx_j8 * sin_q_j8;
    return *this;
}
HomogeneousTransforms::Type_fr_link7_X_fr_link8::Type_fr_link7_X_fr_link8()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_j8;    // Maxima DSL: _k__tx_j8
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_link7_X_fr_link8& HomogeneousTransforms::Type_fr_link7_X_fr_link8::update(const state_t& q)
{
    Scalar sin_q_j8  = ScalarTraits::sin( q(J8) );
    Scalar cos_q_j8  = ScalarTraits::cos( q(J8) );
    (*this)(0,0) = cos_q_j8;
    (*this)(0,1) = -sin_q_j8;
    (*this)(1,0) = sin_q_j8;
    (*this)(1,1) = cos_q_j8;
    return *this;
}

