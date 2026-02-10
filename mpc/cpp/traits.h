#ifndef RCG__ACUROBOT_TRAITS_H_
#define RCG__ACUROBOT_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace AcuRobot {
namespace rcg {
struct Traits {
    typedef typename AcuRobot::rcg::ScalarTraits ScalarTraits;

    typedef typename AcuRobot::rcg::JointState JointState;

    typedef typename AcuRobot::rcg::JointIdentifiers JointID;
    typedef typename AcuRobot::rcg::LinkIdentifiers  LinkID;

    typedef typename AcuRobot::rcg::HomogeneousTransforms HomogeneousTransforms;
    typedef typename AcuRobot::rcg::MotionTransforms MotionTransforms;
    typedef typename AcuRobot::rcg::ForceTransforms ForceTransforms;

    typedef typename AcuRobot::rcg::InertiaProperties InertiaProperties;
    typedef typename AcuRobot::rcg::ForwardDynamics FwdDynEngine;
    typedef typename AcuRobot::rcg::InverseDynamics InvDynEngine;
    typedef typename AcuRobot::rcg::JSIM JSIM;

    static const int joints_count = AcuRobot::rcg::jointsCount;
    static const int links_count  = AcuRobot::rcg::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return AcuRobot::rcg::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return AcuRobot::rcg::orderedLinkIDs;
}

}
}

#endif
