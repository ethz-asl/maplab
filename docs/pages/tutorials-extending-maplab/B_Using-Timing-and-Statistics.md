## Using Timing and Statistics

To enable for a certain package, compile with
```
catkin build --no-deps --force-cmake <package_name> -DENABLE_TIMING=1 -DENABLE_STATISTICS=1
```

To disable it again, run
```
catkin build --no-deps --force-cmake <package_name> -DENABLE_TIMING=0 -DENABLE_STATISTICS=0
```

To time part of your code, you can use
```cpp
#include <aslam/common/timer.h>

timing::Timer my_timer("My Operation");
// Some expensive operation that should be timed.
my_timer.Stop();

// Print all values.
timing::Timing::Print();
```

For statistics collection, you can do
```cpp
#include <aslam/common/statistics/statistics.h>

statistics::StatsCollector my_statistics("My Statistics");
// Add a given value to the collection of statistical samples.
my_statistics.AddValue(5.4);
// Add the value 1 to the statistics (mostly useful to count certain events and their frequency).
my_statistics.IncrementOne();

// Printing similar to timing.
statistics::Statistics::Print();
```

You can also use the `timing` or `statistics` command in the console to see the data.
