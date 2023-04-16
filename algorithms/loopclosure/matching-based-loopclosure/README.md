# Matching Based Loop Closure

Implements an index of descriptors from images which can be queried by
matching the descriptors from the current image. Uses two main approaches.

## Inverted Multi-Index With Product Quantization
Only used for binary descriptors (BRISK and FREAK).

### References
```bibtex
@ARTICLE{6915715,
  title={The Inverted Multi-Index}, 
  author={Babenko, Artem and Lempitsky, Victor},
  journal={IEEE Transactions on Pattern Analysis and Machine Intelligence}, 
  volume={37},
  number={6},
  pages={1247-1260},
  year={2015}
}
```

## Hnswlib - fast approximate nearest neighbor search
Header-only C++ HNSW implementation with python bindings, insertions and updates. Hard copied from https://github.com/nmslib/hnswlib

### HNSW algorithm parameters

#### Search parameters:
* ```ef``` - the size of the dynamic list for the nearest neighbors (used during the search). Higher ```ef```
leads to more accurate but slower search. ```ef``` cannot be set lower than the number of queried nearest neighbors
```k```. The value ```ef``` of can be anything between ```k``` and the size of the dataset.
* ```k``` number of nearest neighbors to be returned as the result.
The ```knn_query``` function returns two numpy arrays, containing labels and distances to the k found nearest 
elements for the queries. Note that in case the algorithm is not be able to find ```k``` neighbors to all of the queries,
(this can be due to problems with graph or ```k```>size of the dataset) an exception is thrown.

#### Construction parameters:
* ```M``` - the number of bi-directional links created for every new element during construction. Reasonable range for ```M``` 
is 2-100. Higher ```M``` work better on datasets with high intrinsic dimensionality and/or high recall, while low ```M``` work 
better for datasets with low intrinsic dimensionality and/or low recalls. The parameter also determines the algorithm's memory 
consumption, which is roughly ```M * 8-10``` bytes per stored element.  
As an example for ```dim```=4 random vectors optimal ```M``` for search is somewhere around 6, while for high dimensional datasets 
(word embeddings, good face descriptors), higher ```M``` are required (e.g. ```M```=48-64) for optimal performance at high recall. 
The range ```M```=12-48 is ok for the most of the use cases. When ```M``` is changed one has to update the other parameters. 
Nonetheless, ef and ef_construction parameters can be roughly estimated by assuming that ```M```*```ef_{construction}``` is 
a constant.

* ```ef_construction``` - the parameter has the same meaning as ```ef```, but controls the index_time/index_accuracy. Bigger 
ef_construction leads to longer construction, but better index quality. At some point, increasing ef_construction does
not improve the quality of the index. One way to check if the selection of ef_construction was ok is to measure a recall 
for M nearest neighbor search when ```ef``` =```ef_construction```: if the recall is lower than 0.9, than there is room 
for improvement.
* ```num_elements``` - defines the maximum number of elements in the index. The index can be extended by saving/loading (load_index
function has a parameter which defines the new maximum number of elements).

### References
```bibtex
@article{malkov2018efficient,
  title={Efficient and robust approximate nearest neighbor search using hierarchical navigable small world graphs},
  author={Malkov, Yu A and Yashunin, Dmitry A},
  journal={IEEE transactions on pattern analysis and machine intelligence},
  volume={42},
  number={4},
  pages={824--836},
  year={2018}
}
```

The update algorithm supported in this repository is to be published in "Dynamic Updates For HNSW, Hierarchical Navigable Small World Graphs" US Patent 15/929,802 by Apoorv Sharma, Abhishek Tayal and Yury Malkov.
