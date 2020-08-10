#ifndef MINKINDR_CONVERSIONS_KINDR_XML_H
#define MINKINDR_CONVERSIONS_KINDR_XML_H

#include <XmlRpcValue.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glog/logging.h>

namespace kindr {
namespace minimal {

// Convert from an xml RPC (from ROS Param server) to a
// kindr::minimal::QuatTransformation.
template <typename Scalar>
void xmlRpcToKindr(XmlRpc::XmlRpcValue& xml_rpc,
                   kindr::minimal::QuatTransformationTemplate<Scalar>* kindr) {
  typename kindr::minimal::QuatTransformationTemplate<Scalar>::RotationMatrix
      temp_rot_matrix;
  typename kindr::minimal::QuatTransformationTemplate<Scalar>::Position
      temp_translation;

  if (kindr == nullptr) {
    LOG(ERROR) << "Null pointer given";
    return;
  }
  if (xml_rpc.size() != 4) {
    LOG(ERROR) << "XmlRpc matrix has " << xml_rpc.size() << " rows";
    return;
  }
  // read raw inputs
  for (size_t i = 0; i < 3; ++i) {
    if (xml_rpc[i].size() != 4) {
      LOG(ERROR) << "XmlRpc matrix has " << xml_rpc[i].size()
                 << " columns in its " << i << " row";
      return;
    }
    for (size_t j = 0; j < 3; ++j) {
      temp_rot_matrix(i, j) = static_cast<double>(xml_rpc[i][j]);
    }
    temp_translation(i) = static_cast<double>(xml_rpc[i][3]);
  }

  // renormalize rotation to correct for rounding error when yaml was written
  kindr::minimal::RotationQuaternionTemplate<Scalar> temp_rot_quat =
      kindr::minimal::RotationQuaternionTemplate<
          Scalar>::constructAndRenormalize(temp_rot_matrix);

  // recombine
  *kindr = kindr::minimal::QuatTransformationTemplate<Scalar>(temp_rot_quat,
                                                              temp_translation);
}

}  // namespace minimal
}  // namespace kindr

#endif  // MINKINDR_CONVERSIONS_KINDR_TF_H
