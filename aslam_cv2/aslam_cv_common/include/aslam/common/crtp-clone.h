#ifndef ASLAM_CV_CRTP_CLONE_H_
#define ASLAM_CV_CRTP_CLONE_H_

namespace aslam {

/// \class Cloneable
/// \brief Implements the cloneable concept using the CRTP and constructor inheritance.
/// Example:
/// @code
///  class Vehicle {
///    Vehicle() = delete;
///    Vehicle(int speed) : speed_(speed) {};
///    virtual Vehicle* clone() const = 0;  // Make sure to add the pure virtual function clone
///    const int speed_;
///  };
///
///  class Car : public aslam::Cloneable<Vehicle, Car> {
///    Car(int speed) : Base(speed) {};     // Use the "Base" typedef defined by the Cloneable class
///  };
/// @endcode
template<typename BaseType, typename DerivedType>
class Cloneable : public BaseType {
 public:
  typedef Cloneable Base;
  using BaseType::BaseType;

  /// Method to clone this instance
  virtual BaseType* clone() const {
    return new DerivedType(static_cast<const DerivedType&>(*this));
  };

  virtual ~Cloneable() {};
};

}  // namespace aslam

#endif  // ASLAM_CV_CRTP_CLONE_H_
