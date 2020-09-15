# Header Include Guide

In `.cc` files, put:

1. The corresponding header
2. A block of system includes
3. A block of includes from non-system packages (3rd party, viwls-graph, Eigen, etc)
4. A block of includes from this package

Separate the blocks by vspace, and alphabetize within each block. For example, for a file `foo.cc` in `my-package`:

```cpp
#include "my-package/foo.h" 

#include <memory>
#include <pthread.h>
#include <string>

#include <Eigen/Core>
#include <epipolar-matching/matcher.h>
#include <glog/logging.h>
#include <vi-map/map.h>

#include "my-package/bar.h"
#include "my-package/baz.h"
```

Header file includes should follow the same structure. 
