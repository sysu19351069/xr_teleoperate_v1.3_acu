#include "jacobians.h"

AcuRobot::rcg::Jacobians::Jacobians()
:    fr_base0_J_ee(), 
    fr_base0_J_fr_link6(), 
    fr_base0_J_fr_link1(), 
    fr_base0_J_link1_end(), 
    fr_base0_J_fr_link2(), 
    fr_base0_J_link2_end(), 
    fr_base0_J_fr_link3(), 
    fr_base0_J_link3_end(), 
    fr_base0_J_fr_link4(), 
    fr_base0_J_link4_end(), 
    fr_base0_J_fr_link5(), 
    fr_base0_J_link5_end()
{}

void AcuRobot::rcg::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles)
{
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_ee::Type_fr_base0_J_ee()
{
    (*this)(0,0) = 0.0;
    (*this)(0,6) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,6) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,6) = 0.0;
    (*this)(5,0) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_ee& AcuRobot::rcg::Jacobians::Type_fr_base0_J_ee::update(const JointState& q)
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
    (*this)(0,1) = sin_q_j1;
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(0,7) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,1) = -cos_q_j1;
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,7) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,7) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,0) = (((((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(cos_q_j1 * sin_q_j4)) * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5)) *  q(J7))+((((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * cos_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * cos_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j4)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)-( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(3,1) = (((((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)+(((( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j5)-( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2);
    (*this)(3,2) = (((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+((( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((- tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,3) = (((((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4)) * sin_q_j5 *  q(J7))+((( tx_j8 * sin_q_j1 * sin_q_j4)+(((- tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j6)+(((((- tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * sin_q_j1 * cos_q_j4)) * cos_q_j5 * cos_q_j6)+((((((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j4)) * sin_q_j5);
    (*this)(3,4) = (((((cos_q_j1 * sin_q_j2 * cos_q_j3)-(cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * cos_q_j5)) *  q(J7))+((((( tx_j8 * sin_q_j1 * sin_q_j4)+(((- tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j4)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(3,5) = ((((( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * sin_q_j4)+(((- tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((- tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * sin_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(3,6) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,7) =  0.0;
    (*this)(4,0) = (((((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+(((((- tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tx_j8 * sin_q_j1 * cos_q_j4)) * sin_q_j6)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((((( tx_j8 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * sin_q_j1 * sin_q_j4)) * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j4)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(4,1) = (((((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+((( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j4 * sin_q_j6)+((((( tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)+(((( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j5)-( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2);
    (*this)(4,2) = (((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+(((((- tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,3) = (((((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4)) * sin_q_j5 *  q(J7))+(((((- tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * cos_q_j1 * sin_q_j4)) * sin_q_j6)+(((((- tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tx_j8 * cos_q_j1 * cos_q_j4)) * cos_q_j5 * cos_q_j6)+((((((- tz_j7- tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(( tz_j7+ tz_ee) * cos_q_j1 * cos_q_j4)) * sin_q_j5);
    (*this)(4,4) = (((((sin_q_j1 * sin_q_j2 * cos_q_j3)-(sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+(((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5)) *  q(J7))+(((((((- tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * cos_q_j1 * sin_q_j4)) * sin_q_j5)+((( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)) * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * sin_q_j1 * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j1 * sin_q_j4)+(((( tz_j7+ tz_ee) * sin_q_j1 * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * cos_q_j5);
    (*this)(4,5) = ((((( tx_j8 * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((((- tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tx_j8 * cos_q_j1 * sin_q_j4)) * cos_q_j5)) * sin_q_j6)+(((((- tx_j8 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tx_j8 * cos_q_j1 * cos_q_j4)) * cos_q_j6);
    (*this)(4,6) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(4,7) =  0.0;
    (*this)(5,1) = (((((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * cos_q_j5)) *  q(J7))+(((- tx_j8 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+((((( tx_j8 * sin_q_j2 * cos_q_j3)-( tx_j8 * cos_q_j2 * sin_q_j3)) * sin_q_j5)+((( tx_j8 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)+(((( tz_j7+ tz_ee) * sin_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * cos_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j5)+( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2);
    (*this)(5,2) = (((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j5)) *  q(J7))+((( tx_j8 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j6)+((((( tx_j8 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j2 * cos_q_j3)) * sin_q_j5)+(((- tx_j8 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j5)-( tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3);
    (*this)(5,3) = (((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j5 *  q(J7))+((( tx_j8 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j6)+((( tx_j8 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j5 * cos_q_j6)+(((( tz_j7+ tz_ee) * cos_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * sin_q_j2 * cos_q_j3)) * sin_q_j4 * sin_q_j5);
    (*this)(5,4) = (((((-sin_q_j2 * sin_q_j3)-(cos_q_j2 * cos_q_j3)) * sin_q_j5)+(((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * cos_q_j5)) *  q(J7))+((((( tx_j8 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * sin_q_j5)+(((- tx_j8 * sin_q_j2 * sin_q_j3)-( tx_j8 * cos_q_j2 * cos_q_j3)) * cos_q_j5)) * cos_q_j6)+((((- tz_j7- tz_ee) * sin_q_j2 * sin_q_j3)+((- tz_j7- tz_ee) * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((((- tz_j7- tz_ee) * cos_q_j2 * sin_q_j3)+(( tz_j7+ tz_ee) * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5);
    (*this)(5,5) = ((((( tx_j8 * sin_q_j2 * sin_q_j3)+( tx_j8 * cos_q_j2 * cos_q_j3)) * sin_q_j5)+((( tx_j8 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j2 * cos_q_j3)) * cos_q_j4 * cos_q_j5)) * sin_q_j6)+((( tx_j8 * cos_q_j2 * sin_q_j3)-( tx_j8 * sin_q_j2 * cos_q_j3)) * sin_q_j4 * cos_q_j6);
    (*this)(5,6) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(5,7) =  0.0;
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link6::Type_fr_base0_J_fr_link6()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link6& AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link6::update(const JointState& q)
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
    (*this)(0,1) = sin_q_j1;
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(0,5) = (((((cos_q_j1 * sin_q_j2 * sin_q_j3)+(cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-(sin_q_j1 * sin_q_j4)) * sin_q_j5)+(((cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(1,1) = -cos_q_j1;
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(1,5) = (((cos_q_j1 * sin_q_j4)+(((sin_q_j1 * sin_q_j2 * sin_q_j3)+(sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)) * sin_q_j5)+(((sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(2,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(2,5) = (((sin_q_j2 * cos_q_j3)-(cos_q_j2 * sin_q_j3)) * cos_q_j4 * sin_q_j5)+(((sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3)) * cos_q_j5);
    (*this)(3,0) = (- tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(3,1) = (- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2);
    (*this)(3,2) = ( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,3) =  0.0;
    (*this)(4,0) = ( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(4,1) = (- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2);
    (*this)(4,2) = ( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,3) =  0.0;
    (*this)(5,1) = ( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2);
    (*this)(5,2) = ( tz_j5 * sin_q_j2 * cos_q_j3)-( tz_j5 * cos_q_j2 * sin_q_j3);
    (*this)(5,3) =  0.0;
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link1::Type_fr_base0_J_fr_link1()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link1& AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link1::update(const JointState& q)
{
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_link1_end::Type_fr_base0_J_link1_end()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_link1_end& AcuRobot::rcg::Jacobians::Type_fr_base0_J_link1_end::update(const JointState& q)
{
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link2::Type_fr_base0_J_fr_link2()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link2& AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link2::update(const JointState& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,1) = sin_q_j1;
    (*this)(1,1) = -cos_q_j1;
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_link2_end::Type_fr_base0_J_link2_end()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_link2_end& AcuRobot::rcg::Jacobians::Type_fr_base0_J_link2_end::update(const JointState& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    (*this)(0,1) = sin_q_j1;
    (*this)(1,1) = -cos_q_j1;
    (*this)(3,0) =  1.0 *  tz_link2_end * cos_q_j1;
    (*this)(4,0) =  1.0 *  tz_link2_end * sin_q_j1;
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link3::Type_fr_base0_J_fr_link3()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,2) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link3& AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link3::update(const JointState& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,1) = sin_q_j1;
    (*this)(0,2) = -sin_q_j1;
    (*this)(1,1) = -cos_q_j1;
    (*this)(1,2) = cos_q_j1;
    (*this)(3,0) =  ty_j3 * sin_q_j1 * sin_q_j2;
    (*this)(3,1) = - ty_j3 * cos_q_j1 * cos_q_j2;
    (*this)(4,0) = - ty_j3 * cos_q_j1 * sin_q_j2;
    (*this)(4,1) = - ty_j3 * sin_q_j1 * cos_q_j2;
    (*this)(5,1) = - ty_j3 * sin_q_j2;
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_link3_end::Type_fr_base0_J_link3_end()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,2) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_link3_end& AcuRobot::rcg::Jacobians::Type_fr_base0_J_link3_end::update(const JointState& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    (*this)(0,1) = sin_q_j1;
    (*this)(0,2) = -sin_q_j1;
    (*this)(1,1) = -cos_q_j1;
    (*this)(1,2) = cos_q_j1;
    (*this)(3,0) = ( ty_j3 * sin_q_j1 * sin_q_j2)-( tz_link3_end * cos_q_j1);
    (*this)(3,1) = - ty_j3 * cos_q_j1 * cos_q_j2;
    (*this)(4,0) = (- ty_j3 * cos_q_j1 * sin_q_j2)-( tz_link3_end * sin_q_j1);
    (*this)(4,1) = - ty_j3 * sin_q_j1 * cos_q_j2;
    (*this)(5,1) = - ty_j3 * sin_q_j2;
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link4::Type_fr_base0_J_fr_link4()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link4& AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link4::update(const JointState& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,1) = sin_q_j1;
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,1) = -cos_q_j1;
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(2,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(3,0) =  ty_j3 * sin_q_j1 * sin_q_j2;
    (*this)(3,1) = - ty_j3 * cos_q_j1 * cos_q_j2;
    (*this)(4,0) = - ty_j3 * cos_q_j1 * sin_q_j2;
    (*this)(4,1) = - ty_j3 * sin_q_j1 * cos_q_j2;
    (*this)(5,1) = - ty_j3 * sin_q_j2;
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_link4_end::Type_fr_base0_J_link4_end()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(5,0) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_link4_end& AcuRobot::rcg::Jacobians::Type_fr_base0_J_link4_end::update(const JointState& q)
{
    Scalar sin_q_j1  = ScalarTraits::sin( q(J1) );
    Scalar cos_q_j1  = ScalarTraits::cos( q(J1) );
    Scalar sin_q_j2  = ScalarTraits::sin( q(J2) );
    Scalar cos_q_j2  = ScalarTraits::cos( q(J2) );
    Scalar sin_q_j3  = ScalarTraits::sin( q(J3) );
    Scalar cos_q_j3  = ScalarTraits::cos( q(J3) );
    Scalar sin_q_j4  = ScalarTraits::sin( q(J4) );
    Scalar cos_q_j4  = ScalarTraits::cos( q(J4) );
    (*this)(0,1) = sin_q_j1;
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,1) = -cos_q_j1;
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(2,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(3,0) = (- tz_link4_end * sin_q_j1 * cos_q_j2 * sin_q_j3)+( tz_link4_end * sin_q_j1 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(3,1) = (- tz_link4_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2);
    (*this)(3,2) = ( tz_link4_end * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_link4_end * cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,3) =  0.0;
    (*this)(4,0) = ( tz_link4_end * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_link4_end * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(4,1) = (- tz_link4_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2);
    (*this)(4,2) = ( tz_link4_end * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_link4_end * sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,3) =  0.0;
    (*this)(5,1) = ( tz_link4_end * cos_q_j2 * sin_q_j3)-( tz_link4_end * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2);
    (*this)(5,2) = ( tz_link4_end * sin_q_j2 * cos_q_j3)-( tz_link4_end * cos_q_j2 * sin_q_j3);
    (*this)(5,3) =  0.0;
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link5::Type_fr_base0_J_fr_link5()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,4) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link5& AcuRobot::rcg::Jacobians::Type_fr_base0_J_fr_link5::update(const JointState& q)
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
    (*this)(0,1) = sin_q_j1;
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(1,1) = -cos_q_j1;
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(2,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(3,0) = (- tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(3,1) = (- tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2);
    (*this)(3,2) = ( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,3) =  0.0;
    (*this)(4,0) = ( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(4,1) = (- tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2);
    (*this)(4,2) = ( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,3) =  0.0;
    (*this)(5,1) = ( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2);
    (*this)(5,2) = ( tz_j5 * sin_q_j2 * cos_q_j3)-( tz_j5 * cos_q_j2 * sin_q_j3);
    (*this)(5,3) =  0.0;
    return *this;
}

AcuRobot::rcg::Jacobians::Type_fr_base0_J_link5_end::Type_fr_base0_J_link5_end()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(5,0) = 0.0;
}

const AcuRobot::rcg::Jacobians::Type_fr_base0_J_link5_end& AcuRobot::rcg::Jacobians::Type_fr_base0_J_link5_end::update(const JointState& q)
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
    (*this)(0,1) = sin_q_j1;
    (*this)(0,2) = -sin_q_j1;
    (*this)(0,3) = (cos_q_j1 * cos_q_j2 * sin_q_j3)-(cos_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(0,4) = (((-cos_q_j1 * sin_q_j2 * sin_q_j3)-(cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-(sin_q_j1 * cos_q_j4);
    (*this)(1,1) = -cos_q_j1;
    (*this)(1,2) = cos_q_j1;
    (*this)(1,3) = (sin_q_j1 * cos_q_j2 * sin_q_j3)-(sin_q_j1 * sin_q_j2 * cos_q_j3);
    (*this)(1,4) = (((-sin_q_j1 * sin_q_j2 * sin_q_j3)-(sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)+(cos_q_j1 * cos_q_j4);
    (*this)(2,3) = (sin_q_j2 * sin_q_j3)+(cos_q_j2 * cos_q_j3);
    (*this)(2,4) = ((cos_q_j2 * sin_q_j3)-(sin_q_j2 * cos_q_j3)) * sin_q_j4;
    (*this)(3,0) = ((( tz_link5_end * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_link5_end * sin_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_link5_end * cos_q_j1 * cos_q_j4)-( tz_j5 * sin_q_j1 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * sin_q_j2 * cos_q_j3)+( ty_j3 * sin_q_j1 * sin_q_j2);
    (*this)(3,1) = ((( tz_link5_end * cos_q_j1 * sin_q_j2 * cos_q_j3)-( tz_link5_end * cos_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j4)-( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * cos_q_j2);
    (*this)(3,2) = ((( tz_link5_end * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j5 * cos_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * cos_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(3,3) = ( tz_link5_end * sin_q_j1 * sin_q_j4)+(((- tz_link5_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4);
    (*this)(3,4) =  0.0;
    (*this)(4,0) = (((- tz_link5_end * cos_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j1 * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_link5_end * sin_q_j1 * cos_q_j4)+( tz_j5 * cos_q_j1 * cos_q_j2 * sin_q_j3)-( tz_j5 * cos_q_j1 * sin_q_j2 * cos_q_j3)-( ty_j3 * cos_q_j1 * sin_q_j2);
    (*this)(4,1) = ((( tz_link5_end * sin_q_j1 * sin_q_j2 * cos_q_j3)-( tz_link5_end * sin_q_j1 * cos_q_j2 * sin_q_j3)) * sin_q_j4)-( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j1 * cos_q_j2);
    (*this)(4,2) = ((( tz_link5_end * sin_q_j1 * cos_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j1 * sin_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j5 * sin_q_j1 * sin_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j1 * cos_q_j2 * cos_q_j3);
    (*this)(4,3) = (((- tz_link5_end * sin_q_j1 * sin_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j1 * cos_q_j2 * cos_q_j3)) * cos_q_j4)-( tz_link5_end * cos_q_j1 * sin_q_j4);
    (*this)(4,4) =  0.0;
    (*this)(5,1) = (((- tz_link5_end * sin_q_j2 * sin_q_j3)-( tz_link5_end * cos_q_j2 * cos_q_j3)) * sin_q_j4)+( tz_j5 * cos_q_j2 * sin_q_j3)-( tz_j5 * sin_q_j2 * cos_q_j3)-( ty_j3 * sin_q_j2);
    (*this)(5,2) = ((( tz_link5_end * sin_q_j2 * sin_q_j3)+( tz_link5_end * cos_q_j2 * cos_q_j3)) * sin_q_j4)-( tz_j5 * cos_q_j2 * sin_q_j3)+( tz_j5 * sin_q_j2 * cos_q_j3);
    (*this)(5,3) = (( tz_link5_end * cos_q_j2 * sin_q_j3)-( tz_link5_end * sin_q_j2 * cos_q_j3)) * cos_q_j4;
    (*this)(5,4) =  0.0;
    return *this;
}

