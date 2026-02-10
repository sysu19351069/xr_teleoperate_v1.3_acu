#ifndef RCG_ACUROBOT_JOINT_DATA_MAP_H_
#define RCG_ACUROBOT_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace AcuRobot {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[J1] = rhs[J1];
    data[J2] = rhs[J2];
    data[J3] = rhs[J3];
    data[J4] = rhs[J4];
    data[J5] = rhs[J5];
    data[J6] = rhs[J6];
    data[J7] = rhs[J7];
    data[J8] = rhs[J8];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[J1] = value;
    data[J2] = value;
    data[J3] = value;
    data[J4] = value;
    data[J5] = value;
    data[J6] = value;
    data[J7] = value;
    data[J8] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   j1 = "
    << map[J1]
    << "   j2 = "
    << map[J2]
    << "   j3 = "
    << map[J3]
    << "   j4 = "
    << map[J4]
    << "   j5 = "
    << map[J5]
    << "   j6 = "
    << map[J6]
    << "   j7 = "
    << map[J7]
    << "   j8 = "
    << map[J8]
    ;
    return out;
}

}
}
#endif
