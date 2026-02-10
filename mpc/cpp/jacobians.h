#ifndef ACUROBOT_JACOBIANS_H_
#define ACUROBOT_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "transforms.h" // to use the same 'Parameters' struct defined there
#include "model_constants.h"

namespace AcuRobot {
namespace rcg {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians
{
    public:
        
        struct Type_fr_base0_J_ee : public JacobianT<8, Type_fr_base0_J_ee>
        {
            Type_fr_base0_J_ee();
            const Type_fr_base0_J_ee& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_fr_link6 : public JacobianT<6, Type_fr_base0_J_fr_link6>
        {
            Type_fr_base0_J_fr_link6();
            const Type_fr_base0_J_fr_link6& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_fr_link1 : public JacobianT<1, Type_fr_base0_J_fr_link1>
        {
            Type_fr_base0_J_fr_link1();
            const Type_fr_base0_J_fr_link1& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_link1_end : public JacobianT<1, Type_fr_base0_J_link1_end>
        {
            Type_fr_base0_J_link1_end();
            const Type_fr_base0_J_link1_end& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_fr_link2 : public JacobianT<2, Type_fr_base0_J_fr_link2>
        {
            Type_fr_base0_J_fr_link2();
            const Type_fr_base0_J_fr_link2& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_link2_end : public JacobianT<2, Type_fr_base0_J_link2_end>
        {
            Type_fr_base0_J_link2_end();
            const Type_fr_base0_J_link2_end& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_fr_link3 : public JacobianT<3, Type_fr_base0_J_fr_link3>
        {
            Type_fr_base0_J_fr_link3();
            const Type_fr_base0_J_fr_link3& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_link3_end : public JacobianT<3, Type_fr_base0_J_link3_end>
        {
            Type_fr_base0_J_link3_end();
            const Type_fr_base0_J_link3_end& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_fr_link4 : public JacobianT<4, Type_fr_base0_J_fr_link4>
        {
            Type_fr_base0_J_fr_link4();
            const Type_fr_base0_J_fr_link4& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_link4_end : public JacobianT<4, Type_fr_base0_J_link4_end>
        {
            Type_fr_base0_J_link4_end();
            const Type_fr_base0_J_link4_end& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_fr_link5 : public JacobianT<5, Type_fr_base0_J_fr_link5>
        {
            Type_fr_base0_J_fr_link5();
            const Type_fr_base0_J_fr_link5& update(const JointState&);
        };
        
        
        struct Type_fr_base0_J_link5_end : public JacobianT<5, Type_fr_base0_J_link5_end>
        {
            Type_fr_base0_J_link5_end();
            const Type_fr_base0_J_link5_end& update(const JointState&);
        };
        
    public:
        Jacobians();
        void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
    public:
        Type_fr_base0_J_ee fr_base0_J_ee;
        Type_fr_base0_J_fr_link6 fr_base0_J_fr_link6;
        Type_fr_base0_J_fr_link1 fr_base0_J_fr_link1;
        Type_fr_base0_J_link1_end fr_base0_J_link1_end;
        Type_fr_base0_J_fr_link2 fr_base0_J_fr_link2;
        Type_fr_base0_J_link2_end fr_base0_J_link2_end;
        Type_fr_base0_J_fr_link3 fr_base0_J_fr_link3;
        Type_fr_base0_J_link3_end fr_base0_J_link3_end;
        Type_fr_base0_J_fr_link4 fr_base0_J_fr_link4;
        Type_fr_base0_J_link4_end fr_base0_J_link4_end;
        Type_fr_base0_J_fr_link5 fr_base0_J_fr_link5;
        Type_fr_base0_J_link5_end fr_base0_J_link5_end;

    protected:
        Parameters params;

};


}
}

#endif
