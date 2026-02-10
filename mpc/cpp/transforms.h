#ifndef ACUROBOT_TRANSFORMS_H_
#define ACUROBOT_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace AcuRobot {
namespace rcg {

struct Parameters
{
    struct AngleFuncValues {
        AngleFuncValues() {
            update();
        }

        void update()
        {
        }
    };

    Params_lengths lengths;
    Params_angles angles;
    AngleFuncValues trig = AngleFuncValues();
};

// The type of the "vector" with the status of the variables
typedef JointState state_t;

template<class M>
using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms
{
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;

    struct Type_fr_base0_X_ee : public TransformMotion<Type_fr_base0_X_ee>
    {
        Type_fr_base0_X_ee();
        const Type_fr_base0_X_ee& update(const state_t&);
    };
    
    struct Type_fr_j6_X_ee : public TransformMotion<Type_fr_j6_X_ee>
    {
        Type_fr_j6_X_ee();
        const Type_fr_j6_X_ee& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link1_end : public TransformMotion<Type_fr_base0_X_link1_end>
    {
        Type_fr_base0_X_link1_end();
        const Type_fr_base0_X_link1_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link2_end : public TransformMotion<Type_fr_base0_X_link2_end>
    {
        Type_fr_base0_X_link2_end();
        const Type_fr_base0_X_link2_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link3_end : public TransformMotion<Type_fr_base0_X_link3_end>
    {
        Type_fr_base0_X_link3_end();
        const Type_fr_base0_X_link3_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link4_end : public TransformMotion<Type_fr_base0_X_link4_end>
    {
        Type_fr_base0_X_link4_end();
        const Type_fr_base0_X_link4_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link5_end : public TransformMotion<Type_fr_base0_X_link5_end>
    {
        Type_fr_base0_X_link5_end();
        const Type_fr_base0_X_link5_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j1 : public TransformMotion<Type_fr_base0_X_fr_j1>
    {
        Type_fr_base0_X_fr_j1();
        const Type_fr_base0_X_fr_j1& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j2 : public TransformMotion<Type_fr_base0_X_fr_j2>
    {
        Type_fr_base0_X_fr_j2();
        const Type_fr_base0_X_fr_j2& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j3 : public TransformMotion<Type_fr_base0_X_fr_j3>
    {
        Type_fr_base0_X_fr_j3();
        const Type_fr_base0_X_fr_j3& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j4 : public TransformMotion<Type_fr_base0_X_fr_j4>
    {
        Type_fr_base0_X_fr_j4();
        const Type_fr_base0_X_fr_j4& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j5 : public TransformMotion<Type_fr_base0_X_fr_j5>
    {
        Type_fr_base0_X_fr_j5();
        const Type_fr_base0_X_fr_j5& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j6 : public TransformMotion<Type_fr_base0_X_fr_j6>
    {
        Type_fr_base0_X_fr_j6();
        const Type_fr_base0_X_fr_j6& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j7 : public TransformMotion<Type_fr_base0_X_fr_j7>
    {
        Type_fr_base0_X_fr_j7();
        const Type_fr_base0_X_fr_j7& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j8 : public TransformMotion<Type_fr_base0_X_fr_j8>
    {
        Type_fr_base0_X_fr_j8();
        const Type_fr_base0_X_fr_j8& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link6 : public TransformMotion<Type_fr_base0_X_fr_link6>
    {
        Type_fr_base0_X_fr_link6();
        const Type_fr_base0_X_fr_link6& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link1 : public TransformMotion<Type_fr_base0_X_fr_link1>
    {
        Type_fr_base0_X_fr_link1();
        const Type_fr_base0_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link2 : public TransformMotion<Type_fr_base0_X_fr_link2>
    {
        Type_fr_base0_X_fr_link2();
        const Type_fr_base0_X_fr_link2& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link3 : public TransformMotion<Type_fr_base0_X_fr_link3>
    {
        Type_fr_base0_X_fr_link3();
        const Type_fr_base0_X_fr_link3& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link4 : public TransformMotion<Type_fr_base0_X_fr_link4>
    {
        Type_fr_base0_X_fr_link4();
        const Type_fr_base0_X_fr_link4& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link5 : public TransformMotion<Type_fr_base0_X_fr_link5>
    {
        Type_fr_base0_X_fr_link5();
        const Type_fr_base0_X_fr_link5& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_base0 : public TransformMotion<Type_fr_link1_X_fr_base0>
    {
        Type_fr_link1_X_fr_base0();
        const Type_fr_link1_X_fr_base0& update(const state_t&);
    };
    
    struct Type_fr_link2_X_fr_link1 : public TransformMotion<Type_fr_link2_X_fr_link1>
    {
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_link2 : public TransformMotion<Type_fr_link1_X_fr_link2>
    {
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const state_t&);
    };
    
    struct Type_fr_link3_X_fr_link2 : public TransformMotion<Type_fr_link3_X_fr_link2>
    {
        Type_fr_link3_X_fr_link2();
        const Type_fr_link3_X_fr_link2& update(const state_t&);
    };
    
    struct Type_fr_link2_X_fr_link3 : public TransformMotion<Type_fr_link2_X_fr_link3>
    {
        Type_fr_link2_X_fr_link3();
        const Type_fr_link2_X_fr_link3& update(const state_t&);
    };
    
    struct Type_fr_link4_X_fr_link3 : public TransformMotion<Type_fr_link4_X_fr_link3>
    {
        Type_fr_link4_X_fr_link3();
        const Type_fr_link4_X_fr_link3& update(const state_t&);
    };
    
    struct Type_fr_link3_X_fr_link4 : public TransformMotion<Type_fr_link3_X_fr_link4>
    {
        Type_fr_link3_X_fr_link4();
        const Type_fr_link3_X_fr_link4& update(const state_t&);
    };
    
    struct Type_fr_link5_X_fr_link4 : public TransformMotion<Type_fr_link5_X_fr_link4>
    {
        Type_fr_link5_X_fr_link4();
        const Type_fr_link5_X_fr_link4& update(const state_t&);
    };
    
    struct Type_fr_link4_X_fr_link5 : public TransformMotion<Type_fr_link4_X_fr_link5>
    {
        Type_fr_link4_X_fr_link5();
        const Type_fr_link4_X_fr_link5& update(const state_t&);
    };
    
    struct Type_fr_link6_X_fr_link5 : public TransformMotion<Type_fr_link6_X_fr_link5>
    {
        Type_fr_link6_X_fr_link5();
        const Type_fr_link6_X_fr_link5& update(const state_t&);
    };
    
    struct Type_fr_link5_X_fr_link6 : public TransformMotion<Type_fr_link5_X_fr_link6>
    {
        Type_fr_link5_X_fr_link6();
        const Type_fr_link5_X_fr_link6& update(const state_t&);
    };
    
    struct Type_fr_link7_X_fr_link6 : public TransformMotion<Type_fr_link7_X_fr_link6>
    {
        Type_fr_link7_X_fr_link6();
        const Type_fr_link7_X_fr_link6& update(const state_t&);
    };
    
    struct Type_fr_link6_X_fr_link7 : public TransformMotion<Type_fr_link6_X_fr_link7>
    {
        Type_fr_link6_X_fr_link7();
        const Type_fr_link6_X_fr_link7& update(const state_t&);
    };
    
    struct Type_fr_link8_X_fr_link7 : public TransformMotion<Type_fr_link8_X_fr_link7>
    {
        Type_fr_link8_X_fr_link7();
        const Type_fr_link8_X_fr_link7& update(const state_t&);
    };
    
    struct Type_fr_link7_X_fr_link8 : public TransformMotion<Type_fr_link7_X_fr_link8>
    {
        Type_fr_link7_X_fr_link8();
        const Type_fr_link7_X_fr_link8& update(const state_t&);
    };
    
public:
    MotionTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base0_X_ee fr_base0_X_ee;
    Type_fr_j6_X_ee fr_j6_X_ee;
    Type_fr_base0_X_link1_end fr_base0_X_link1_end;
    Type_fr_base0_X_link2_end fr_base0_X_link2_end;
    Type_fr_base0_X_link3_end fr_base0_X_link3_end;
    Type_fr_base0_X_link4_end fr_base0_X_link4_end;
    Type_fr_base0_X_link5_end fr_base0_X_link5_end;
    Type_fr_base0_X_fr_j1 fr_base0_X_fr_j1;
    Type_fr_base0_X_fr_j2 fr_base0_X_fr_j2;
    Type_fr_base0_X_fr_j3 fr_base0_X_fr_j3;
    Type_fr_base0_X_fr_j4 fr_base0_X_fr_j4;
    Type_fr_base0_X_fr_j5 fr_base0_X_fr_j5;
    Type_fr_base0_X_fr_j6 fr_base0_X_fr_j6;
    Type_fr_base0_X_fr_j7 fr_base0_X_fr_j7;
    Type_fr_base0_X_fr_j8 fr_base0_X_fr_j8;
    Type_fr_base0_X_fr_link6 fr_base0_X_fr_link6;
    Type_fr_base0_X_fr_link1 fr_base0_X_fr_link1;
    Type_fr_base0_X_fr_link2 fr_base0_X_fr_link2;
    Type_fr_base0_X_fr_link3 fr_base0_X_fr_link3;
    Type_fr_base0_X_fr_link4 fr_base0_X_fr_link4;
    Type_fr_base0_X_fr_link5 fr_base0_X_fr_link5;
    Type_fr_link1_X_fr_base0 fr_link1_X_fr_base0;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;
    Type_fr_link3_X_fr_link2 fr_link3_X_fr_link2;
    Type_fr_link2_X_fr_link3 fr_link2_X_fr_link3;
    Type_fr_link4_X_fr_link3 fr_link4_X_fr_link3;
    Type_fr_link3_X_fr_link4 fr_link3_X_fr_link4;
    Type_fr_link5_X_fr_link4 fr_link5_X_fr_link4;
    Type_fr_link4_X_fr_link5 fr_link4_X_fr_link5;
    Type_fr_link6_X_fr_link5 fr_link6_X_fr_link5;
    Type_fr_link5_X_fr_link6 fr_link5_X_fr_link6;
    Type_fr_link7_X_fr_link6 fr_link7_X_fr_link6;
    Type_fr_link6_X_fr_link7 fr_link6_X_fr_link7;
    Type_fr_link8_X_fr_link7 fr_link8_X_fr_link7;
    Type_fr_link7_X_fr_link8 fr_link7_X_fr_link8;

protected:
    Parameters params;

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms
{
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;

    struct Type_fr_base0_X_ee : public TransformForce<Type_fr_base0_X_ee>
    {
        Type_fr_base0_X_ee();
        const Type_fr_base0_X_ee& update(const state_t&);
    };
    
    struct Type_fr_j6_X_ee : public TransformForce<Type_fr_j6_X_ee>
    {
        Type_fr_j6_X_ee();
        const Type_fr_j6_X_ee& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link1_end : public TransformForce<Type_fr_base0_X_link1_end>
    {
        Type_fr_base0_X_link1_end();
        const Type_fr_base0_X_link1_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link2_end : public TransformForce<Type_fr_base0_X_link2_end>
    {
        Type_fr_base0_X_link2_end();
        const Type_fr_base0_X_link2_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link3_end : public TransformForce<Type_fr_base0_X_link3_end>
    {
        Type_fr_base0_X_link3_end();
        const Type_fr_base0_X_link3_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link4_end : public TransformForce<Type_fr_base0_X_link4_end>
    {
        Type_fr_base0_X_link4_end();
        const Type_fr_base0_X_link4_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link5_end : public TransformForce<Type_fr_base0_X_link5_end>
    {
        Type_fr_base0_X_link5_end();
        const Type_fr_base0_X_link5_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j1 : public TransformForce<Type_fr_base0_X_fr_j1>
    {
        Type_fr_base0_X_fr_j1();
        const Type_fr_base0_X_fr_j1& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j2 : public TransformForce<Type_fr_base0_X_fr_j2>
    {
        Type_fr_base0_X_fr_j2();
        const Type_fr_base0_X_fr_j2& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j3 : public TransformForce<Type_fr_base0_X_fr_j3>
    {
        Type_fr_base0_X_fr_j3();
        const Type_fr_base0_X_fr_j3& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j4 : public TransformForce<Type_fr_base0_X_fr_j4>
    {
        Type_fr_base0_X_fr_j4();
        const Type_fr_base0_X_fr_j4& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j5 : public TransformForce<Type_fr_base0_X_fr_j5>
    {
        Type_fr_base0_X_fr_j5();
        const Type_fr_base0_X_fr_j5& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j6 : public TransformForce<Type_fr_base0_X_fr_j6>
    {
        Type_fr_base0_X_fr_j6();
        const Type_fr_base0_X_fr_j6& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j7 : public TransformForce<Type_fr_base0_X_fr_j7>
    {
        Type_fr_base0_X_fr_j7();
        const Type_fr_base0_X_fr_j7& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j8 : public TransformForce<Type_fr_base0_X_fr_j8>
    {
        Type_fr_base0_X_fr_j8();
        const Type_fr_base0_X_fr_j8& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link6 : public TransformForce<Type_fr_base0_X_fr_link6>
    {
        Type_fr_base0_X_fr_link6();
        const Type_fr_base0_X_fr_link6& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link1 : public TransformForce<Type_fr_base0_X_fr_link1>
    {
        Type_fr_base0_X_fr_link1();
        const Type_fr_base0_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link2 : public TransformForce<Type_fr_base0_X_fr_link2>
    {
        Type_fr_base0_X_fr_link2();
        const Type_fr_base0_X_fr_link2& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link3 : public TransformForce<Type_fr_base0_X_fr_link3>
    {
        Type_fr_base0_X_fr_link3();
        const Type_fr_base0_X_fr_link3& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link4 : public TransformForce<Type_fr_base0_X_fr_link4>
    {
        Type_fr_base0_X_fr_link4();
        const Type_fr_base0_X_fr_link4& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link5 : public TransformForce<Type_fr_base0_X_fr_link5>
    {
        Type_fr_base0_X_fr_link5();
        const Type_fr_base0_X_fr_link5& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_base0 : public TransformForce<Type_fr_link1_X_fr_base0>
    {
        Type_fr_link1_X_fr_base0();
        const Type_fr_link1_X_fr_base0& update(const state_t&);
    };
    
    struct Type_fr_link2_X_fr_link1 : public TransformForce<Type_fr_link2_X_fr_link1>
    {
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_link2 : public TransformForce<Type_fr_link1_X_fr_link2>
    {
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const state_t&);
    };
    
    struct Type_fr_link3_X_fr_link2 : public TransformForce<Type_fr_link3_X_fr_link2>
    {
        Type_fr_link3_X_fr_link2();
        const Type_fr_link3_X_fr_link2& update(const state_t&);
    };
    
    struct Type_fr_link2_X_fr_link3 : public TransformForce<Type_fr_link2_X_fr_link3>
    {
        Type_fr_link2_X_fr_link3();
        const Type_fr_link2_X_fr_link3& update(const state_t&);
    };
    
    struct Type_fr_link4_X_fr_link3 : public TransformForce<Type_fr_link4_X_fr_link3>
    {
        Type_fr_link4_X_fr_link3();
        const Type_fr_link4_X_fr_link3& update(const state_t&);
    };
    
    struct Type_fr_link3_X_fr_link4 : public TransformForce<Type_fr_link3_X_fr_link4>
    {
        Type_fr_link3_X_fr_link4();
        const Type_fr_link3_X_fr_link4& update(const state_t&);
    };
    
    struct Type_fr_link5_X_fr_link4 : public TransformForce<Type_fr_link5_X_fr_link4>
    {
        Type_fr_link5_X_fr_link4();
        const Type_fr_link5_X_fr_link4& update(const state_t&);
    };
    
    struct Type_fr_link4_X_fr_link5 : public TransformForce<Type_fr_link4_X_fr_link5>
    {
        Type_fr_link4_X_fr_link5();
        const Type_fr_link4_X_fr_link5& update(const state_t&);
    };
    
    struct Type_fr_link6_X_fr_link5 : public TransformForce<Type_fr_link6_X_fr_link5>
    {
        Type_fr_link6_X_fr_link5();
        const Type_fr_link6_X_fr_link5& update(const state_t&);
    };
    
    struct Type_fr_link5_X_fr_link6 : public TransformForce<Type_fr_link5_X_fr_link6>
    {
        Type_fr_link5_X_fr_link6();
        const Type_fr_link5_X_fr_link6& update(const state_t&);
    };
    
    struct Type_fr_link7_X_fr_link6 : public TransformForce<Type_fr_link7_X_fr_link6>
    {
        Type_fr_link7_X_fr_link6();
        const Type_fr_link7_X_fr_link6& update(const state_t&);
    };
    
    struct Type_fr_link6_X_fr_link7 : public TransformForce<Type_fr_link6_X_fr_link7>
    {
        Type_fr_link6_X_fr_link7();
        const Type_fr_link6_X_fr_link7& update(const state_t&);
    };
    
    struct Type_fr_link8_X_fr_link7 : public TransformForce<Type_fr_link8_X_fr_link7>
    {
        Type_fr_link8_X_fr_link7();
        const Type_fr_link8_X_fr_link7& update(const state_t&);
    };
    
    struct Type_fr_link7_X_fr_link8 : public TransformForce<Type_fr_link7_X_fr_link8>
    {
        Type_fr_link7_X_fr_link8();
        const Type_fr_link7_X_fr_link8& update(const state_t&);
    };
    
public:
    ForceTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base0_X_ee fr_base0_X_ee;
    Type_fr_j6_X_ee fr_j6_X_ee;
    Type_fr_base0_X_link1_end fr_base0_X_link1_end;
    Type_fr_base0_X_link2_end fr_base0_X_link2_end;
    Type_fr_base0_X_link3_end fr_base0_X_link3_end;
    Type_fr_base0_X_link4_end fr_base0_X_link4_end;
    Type_fr_base0_X_link5_end fr_base0_X_link5_end;
    Type_fr_base0_X_fr_j1 fr_base0_X_fr_j1;
    Type_fr_base0_X_fr_j2 fr_base0_X_fr_j2;
    Type_fr_base0_X_fr_j3 fr_base0_X_fr_j3;
    Type_fr_base0_X_fr_j4 fr_base0_X_fr_j4;
    Type_fr_base0_X_fr_j5 fr_base0_X_fr_j5;
    Type_fr_base0_X_fr_j6 fr_base0_X_fr_j6;
    Type_fr_base0_X_fr_j7 fr_base0_X_fr_j7;
    Type_fr_base0_X_fr_j8 fr_base0_X_fr_j8;
    Type_fr_base0_X_fr_link6 fr_base0_X_fr_link6;
    Type_fr_base0_X_fr_link1 fr_base0_X_fr_link1;
    Type_fr_base0_X_fr_link2 fr_base0_X_fr_link2;
    Type_fr_base0_X_fr_link3 fr_base0_X_fr_link3;
    Type_fr_base0_X_fr_link4 fr_base0_X_fr_link4;
    Type_fr_base0_X_fr_link5 fr_base0_X_fr_link5;
    Type_fr_link1_X_fr_base0 fr_link1_X_fr_base0;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;
    Type_fr_link3_X_fr_link2 fr_link3_X_fr_link2;
    Type_fr_link2_X_fr_link3 fr_link2_X_fr_link3;
    Type_fr_link4_X_fr_link3 fr_link4_X_fr_link3;
    Type_fr_link3_X_fr_link4 fr_link3_X_fr_link4;
    Type_fr_link5_X_fr_link4 fr_link5_X_fr_link4;
    Type_fr_link4_X_fr_link5 fr_link4_X_fr_link5;
    Type_fr_link6_X_fr_link5 fr_link6_X_fr_link5;
    Type_fr_link5_X_fr_link6 fr_link5_X_fr_link6;
    Type_fr_link7_X_fr_link6 fr_link7_X_fr_link6;
    Type_fr_link6_X_fr_link7 fr_link6_X_fr_link7;
    Type_fr_link8_X_fr_link7 fr_link8_X_fr_link7;
    Type_fr_link7_X_fr_link8 fr_link7_X_fr_link8;

protected:
    Parameters params;

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms
{
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

    struct Type_fr_base0_X_ee : public TransformHomogeneous<Type_fr_base0_X_ee>
    {
        Type_fr_base0_X_ee();
        const Type_fr_base0_X_ee& update(const state_t&);
    };
    
    struct Type_fr_j6_X_ee : public TransformHomogeneous<Type_fr_j6_X_ee>
    {
        Type_fr_j6_X_ee();
        const Type_fr_j6_X_ee& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link1_end : public TransformHomogeneous<Type_fr_base0_X_link1_end>
    {
        Type_fr_base0_X_link1_end();
        const Type_fr_base0_X_link1_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link2_end : public TransformHomogeneous<Type_fr_base0_X_link2_end>
    {
        Type_fr_base0_X_link2_end();
        const Type_fr_base0_X_link2_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link3_end : public TransformHomogeneous<Type_fr_base0_X_link3_end>
    {
        Type_fr_base0_X_link3_end();
        const Type_fr_base0_X_link3_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link4_end : public TransformHomogeneous<Type_fr_base0_X_link4_end>
    {
        Type_fr_base0_X_link4_end();
        const Type_fr_base0_X_link4_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_link5_end : public TransformHomogeneous<Type_fr_base0_X_link5_end>
    {
        Type_fr_base0_X_link5_end();
        const Type_fr_base0_X_link5_end& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j1 : public TransformHomogeneous<Type_fr_base0_X_fr_j1>
    {
        Type_fr_base0_X_fr_j1();
        const Type_fr_base0_X_fr_j1& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j2 : public TransformHomogeneous<Type_fr_base0_X_fr_j2>
    {
        Type_fr_base0_X_fr_j2();
        const Type_fr_base0_X_fr_j2& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j3 : public TransformHomogeneous<Type_fr_base0_X_fr_j3>
    {
        Type_fr_base0_X_fr_j3();
        const Type_fr_base0_X_fr_j3& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j4 : public TransformHomogeneous<Type_fr_base0_X_fr_j4>
    {
        Type_fr_base0_X_fr_j4();
        const Type_fr_base0_X_fr_j4& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j5 : public TransformHomogeneous<Type_fr_base0_X_fr_j5>
    {
        Type_fr_base0_X_fr_j5();
        const Type_fr_base0_X_fr_j5& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j6 : public TransformHomogeneous<Type_fr_base0_X_fr_j6>
    {
        Type_fr_base0_X_fr_j6();
        const Type_fr_base0_X_fr_j6& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j7 : public TransformHomogeneous<Type_fr_base0_X_fr_j7>
    {
        Type_fr_base0_X_fr_j7();
        const Type_fr_base0_X_fr_j7& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_j8 : public TransformHomogeneous<Type_fr_base0_X_fr_j8>
    {
        Type_fr_base0_X_fr_j8();
        const Type_fr_base0_X_fr_j8& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link6 : public TransformHomogeneous<Type_fr_base0_X_fr_link6>
    {
        Type_fr_base0_X_fr_link6();
        const Type_fr_base0_X_fr_link6& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link1 : public TransformHomogeneous<Type_fr_base0_X_fr_link1>
    {
        Type_fr_base0_X_fr_link1();
        const Type_fr_base0_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link2 : public TransformHomogeneous<Type_fr_base0_X_fr_link2>
    {
        Type_fr_base0_X_fr_link2();
        const Type_fr_base0_X_fr_link2& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link3 : public TransformHomogeneous<Type_fr_base0_X_fr_link3>
    {
        Type_fr_base0_X_fr_link3();
        const Type_fr_base0_X_fr_link3& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link4 : public TransformHomogeneous<Type_fr_base0_X_fr_link4>
    {
        Type_fr_base0_X_fr_link4();
        const Type_fr_base0_X_fr_link4& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link5 : public TransformHomogeneous<Type_fr_base0_X_fr_link5>
    {
        Type_fr_base0_X_fr_link5();
        const Type_fr_base0_X_fr_link5& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_base0 : public TransformHomogeneous<Type_fr_link1_X_fr_base0>
    {
        Type_fr_link1_X_fr_base0();
        const Type_fr_link1_X_fr_base0& update(const state_t&);
    };
    
    struct Type_fr_link2_X_fr_link1 : public TransformHomogeneous<Type_fr_link2_X_fr_link1>
    {
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_link2 : public TransformHomogeneous<Type_fr_link1_X_fr_link2>
    {
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const state_t&);
    };
    
    struct Type_fr_link3_X_fr_link2 : public TransformHomogeneous<Type_fr_link3_X_fr_link2>
    {
        Type_fr_link3_X_fr_link2();
        const Type_fr_link3_X_fr_link2& update(const state_t&);
    };
    
    struct Type_fr_link2_X_fr_link3 : public TransformHomogeneous<Type_fr_link2_X_fr_link3>
    {
        Type_fr_link2_X_fr_link3();
        const Type_fr_link2_X_fr_link3& update(const state_t&);
    };
    
    struct Type_fr_link4_X_fr_link3 : public TransformHomogeneous<Type_fr_link4_X_fr_link3>
    {
        Type_fr_link4_X_fr_link3();
        const Type_fr_link4_X_fr_link3& update(const state_t&);
    };
    
    struct Type_fr_link3_X_fr_link4 : public TransformHomogeneous<Type_fr_link3_X_fr_link4>
    {
        Type_fr_link3_X_fr_link4();
        const Type_fr_link3_X_fr_link4& update(const state_t&);
    };
    
    struct Type_fr_link5_X_fr_link4 : public TransformHomogeneous<Type_fr_link5_X_fr_link4>
    {
        Type_fr_link5_X_fr_link4();
        const Type_fr_link5_X_fr_link4& update(const state_t&);
    };
    
    struct Type_fr_link4_X_fr_link5 : public TransformHomogeneous<Type_fr_link4_X_fr_link5>
    {
        Type_fr_link4_X_fr_link5();
        const Type_fr_link4_X_fr_link5& update(const state_t&);
    };
    
    struct Type_fr_link6_X_fr_link5 : public TransformHomogeneous<Type_fr_link6_X_fr_link5>
    {
        Type_fr_link6_X_fr_link5();
        const Type_fr_link6_X_fr_link5& update(const state_t&);
    };
    
    struct Type_fr_link5_X_fr_link6 : public TransformHomogeneous<Type_fr_link5_X_fr_link6>
    {
        Type_fr_link5_X_fr_link6();
        const Type_fr_link5_X_fr_link6& update(const state_t&);
    };
    
    struct Type_fr_link7_X_fr_link6 : public TransformHomogeneous<Type_fr_link7_X_fr_link6>
    {
        Type_fr_link7_X_fr_link6();
        const Type_fr_link7_X_fr_link6& update(const state_t&);
    };
    
    struct Type_fr_link6_X_fr_link7 : public TransformHomogeneous<Type_fr_link6_X_fr_link7>
    {
        Type_fr_link6_X_fr_link7();
        const Type_fr_link6_X_fr_link7& update(const state_t&);
    };
    
    struct Type_fr_link8_X_fr_link7 : public TransformHomogeneous<Type_fr_link8_X_fr_link7>
    {
        Type_fr_link8_X_fr_link7();
        const Type_fr_link8_X_fr_link7& update(const state_t&);
    };
    
    struct Type_fr_link7_X_fr_link8 : public TransformHomogeneous<Type_fr_link7_X_fr_link8>
    {
        Type_fr_link7_X_fr_link8();
        const Type_fr_link7_X_fr_link8& update(const state_t&);
    };
    
public:
    HomogeneousTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base0_X_ee fr_base0_X_ee;
    Type_fr_j6_X_ee fr_j6_X_ee;
    Type_fr_base0_X_link1_end fr_base0_X_link1_end;
    Type_fr_base0_X_link2_end fr_base0_X_link2_end;
    Type_fr_base0_X_link3_end fr_base0_X_link3_end;
    Type_fr_base0_X_link4_end fr_base0_X_link4_end;
    Type_fr_base0_X_link5_end fr_base0_X_link5_end;
    Type_fr_base0_X_fr_j1 fr_base0_X_fr_j1;
    Type_fr_base0_X_fr_j2 fr_base0_X_fr_j2;
    Type_fr_base0_X_fr_j3 fr_base0_X_fr_j3;
    Type_fr_base0_X_fr_j4 fr_base0_X_fr_j4;
    Type_fr_base0_X_fr_j5 fr_base0_X_fr_j5;
    Type_fr_base0_X_fr_j6 fr_base0_X_fr_j6;
    Type_fr_base0_X_fr_j7 fr_base0_X_fr_j7;
    Type_fr_base0_X_fr_j8 fr_base0_X_fr_j8;
    Type_fr_base0_X_fr_link6 fr_base0_X_fr_link6;
    Type_fr_base0_X_fr_link1 fr_base0_X_fr_link1;
    Type_fr_base0_X_fr_link2 fr_base0_X_fr_link2;
    Type_fr_base0_X_fr_link3 fr_base0_X_fr_link3;
    Type_fr_base0_X_fr_link4 fr_base0_X_fr_link4;
    Type_fr_base0_X_fr_link5 fr_base0_X_fr_link5;
    Type_fr_link1_X_fr_base0 fr_link1_X_fr_base0;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;
    Type_fr_link3_X_fr_link2 fr_link3_X_fr_link2;
    Type_fr_link2_X_fr_link3 fr_link2_X_fr_link3;
    Type_fr_link4_X_fr_link3 fr_link4_X_fr_link3;
    Type_fr_link3_X_fr_link4 fr_link3_X_fr_link4;
    Type_fr_link5_X_fr_link4 fr_link5_X_fr_link4;
    Type_fr_link4_X_fr_link5 fr_link4_X_fr_link5;
    Type_fr_link6_X_fr_link5 fr_link6_X_fr_link5;
    Type_fr_link5_X_fr_link6 fr_link5_X_fr_link6;
    Type_fr_link7_X_fr_link6 fr_link7_X_fr_link6;
    Type_fr_link6_X_fr_link7 fr_link6_X_fr_link7;
    Type_fr_link8_X_fr_link7 fr_link8_X_fr_link7;
    Type_fr_link7_X_fr_link8 fr_link7_X_fr_link8;

protected:
    Parameters params;

}; //class 'HomogeneousTransforms'

}
}

#endif
