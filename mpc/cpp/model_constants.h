#ifndef RCG_ACUROBOT_MODEL_CONSTANTS_H_
#define RCG_ACUROBOT_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace AcuRobot {
namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types
/*const Scalar tz_j2 = 0.19249999523162842;*/
/*const Scalar ty_j3 = 0.26600000262260437;*/
/*const Scalar tz_j5 = 0.3240000009536743;*/
/*const Scalar tz_j7 = 0.20000000298023224;*/
/*const Scalar tx_j8 = 0.04699999839067459;*/
/*const Scalar tz_link1_end = -0.10000000149011612;*/
/*const Scalar tz_link2_end = 0.10000000149011612;*/
/*const Scalar tz_link3_end = 0.10000000149011612;*/
/*const Scalar tz_link4_end = 0.10000000149011612;*/
/*const Scalar tz_link5_end = 0.1550000011920929;*/
/*const Scalar tz_ee = 0.2299999;*/


const Scalar tz_j2 = 192.49999523162842;
const Scalar ty_j3 = 266.00000262260437;
const Scalar tz_j5 = 324.0000009536743;
const Scalar tz_j7 = 200.00000298023224;
const Scalar tx_j8 = 46.99999839067459;
const Scalar tz_link1_end = -100.00000149011612;
const Scalar tz_link2_end = 100.00000149011612;
const Scalar tz_link3_end = 100.00000149011612;
const Scalar tz_link4_end = 100.00000149011612;
const Scalar tz_link5_end = 155.0000011920929;
const Scalar tz_ee = 229.9999;


const Scalar m_base0 = 1.0;
const Scalar m_link1 = 1.0;
const Scalar comx_link1 = 0.5;
const Scalar ix_link1 = 0.0024999999441206455;
const Scalar iy_link1 = 0.3345800042152405;
const Scalar iz_link1 = 0.3345800042152405;
const Scalar m_link2 = 1.0;
const Scalar comz_link2 = 0.5;
const Scalar ix_link2 = 0.3345800042152405;
const Scalar iy_link2 = 0.3345800042152405;
const Scalar iz_link2 = 0.0024999999441206455;
const Scalar m_link3 = 1.0;
const Scalar comx_link3 = 0.5;
const Scalar ix_link3 = 0.0024999999441206455;
const Scalar iy_link3 = 0.3345800042152405;
const Scalar iz_link3 = 0.3345800042152405;
const Scalar m_link4 = 1.0;
const Scalar comz_link4 = 0.5;
const Scalar ix_link4 = 0.3345800042152405;
const Scalar iy_link4 = 0.3345800042152405;
const Scalar iz_link4 = 0.0024999999441206455;
const Scalar m_link5 = 1.0;
const Scalar comx_link5 = 0.5;
const Scalar ix_link5 = 0.0024999999441206455;
const Scalar iy_link5 = 0.3345800042152405;
const Scalar iz_link5 = 0.3345800042152405;
const Scalar m_link6 = 1.0;
const Scalar comx_link6 = 0.5;
const Scalar ix_link6 = 0.0024999999441206455;
const Scalar iy_link6 = 0.3345800042152405;
const Scalar iz_link6 = 0.3345800042152405;
const Scalar m_link7 = 1.0;
const Scalar comx_link7 = 0.5;
const Scalar ix_link7 = 0.0024999999441206455;
const Scalar iy_link7 = 0.3345800042152405;
const Scalar iz_link7 = 0.3345800042152405;
const Scalar m_link8 = 1.0;
const Scalar comx_link8 = 0.5;
const Scalar ix_link8 = 0.0024999999441206455;
const Scalar iy_link8 = 0.3345800042152405;
const Scalar iz_link8 = 0.3345800042152405;

}
}
#endif
