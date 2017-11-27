inverted_multi_index
==========================

Contains code for an efficient index to search for nearest neighbors for loop-closure:
Implements the inverted multi-index datastructure proposed in
A. Babenko, V. Lempitsky. The Inverted Multi-Index. CVPR'12
for fast (approximate) nearest neighbor search.
The method splits a feature descriptor into two subvectors of equal length
and quantizes each part individually. This yields a fine quantization of the
descriptor space into k^2 visual words but only requires us to store k
descriptors.