#ifndef RCG_ACUROBOT_DECLARATIONS_H_
#define RCG_ACUROBOT_DECLARATIONS_H_

#include "rbd_types.h"

namespace AcuRobot {
namespace rcg {

static constexpr int JointSpaceDimension = 8;
static constexpr int jointsCount = 8;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 9;

typedef Matrix<8, 1> Column8d;
typedef Column8d JointState;

enum JointIdentifiers {
    J1 = 0
    , J2
    , J3
    , J4
    , J5
    , J6
    , J7
    , J8
};

enum LinkIdentifiers {
    BASE0 = 0
    , LINK1
    , LINK2
    , LINK3
    , LINK4
    , LINK5
    , LINK6
    , LINK7
    , LINK8
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {J1,J2,J3,J4,J5,J6,J7,J8};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE0,LINK1,LINK2,LINK3,LINK4,LINK5,LINK6,LINK7,LINK8};

}
}
#endif
