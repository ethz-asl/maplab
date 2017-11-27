#ifndef VOCABULARY_TREE_HAMMING_H_
#define VOCABULARY_TREE_HAMMING_H_
namespace loop_closure {
inline unsigned int HammingDistance32(unsigned int a, unsigned int b);
inline unsigned int HammingDistance128(
    const unsigned char d1[16], const unsigned char d2[16]);
inline unsigned int HammingDistance256(
    const unsigned char d1[32], const unsigned char d2[32]);
inline unsigned int HammingDistance512(
    const unsigned char d1[64], const unsigned char d2[64]);
inline unsigned int HammingDistance(
    const unsigned char* d1, const unsigned char* d2, unsigned int numBits);
}  // namespace loop_closure

#include "vocabulary-tree/impl/hamming-inl.h"

#endif  // VOCABULARY_TREE_HAMMING_H_
