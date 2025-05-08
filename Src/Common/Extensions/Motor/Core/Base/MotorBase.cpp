#include "./MotorBase.hpp"

using namespace PINYMOTOR;
// binding motorMap_
template <typename Derived>
std::vector<std::pair<uint32_t *,
                      std::unordered_map<uint16_t, QuadMotorGroup_s *> > >
        QuadMotorBase<Derived>::motorMap_ = {};

template <typename Derived>
std::vector<std::pair<uint32_t *,
                      std::unordered_map<uint16_t, TripMotorGroup_s *> > >
        TripMotorBase<Derived>::motorMap_ = {};
