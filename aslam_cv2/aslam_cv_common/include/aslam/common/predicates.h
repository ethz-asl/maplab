#ifndef ASLAM_PREDICATES_H
#define ASLAM_PREDICATES_H

namespace aslam {

template<typename Value>
bool checkSharedEqual(const std::shared_ptr<Value> & lhs,
                      const std::shared_ptr<Value> & rhs) {
  if(lhs && rhs) {
    // if they are both nonnull, check for equality
    return (*lhs) == (*rhs);
  } else {
    // otherwise, check if they are both null
    return (!lhs) && (!rhs); 
  }
}

} // namespace aslam


#endif /* ASLAM_PREDICATES_H */
