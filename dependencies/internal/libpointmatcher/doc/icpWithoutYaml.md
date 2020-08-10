# Example of an ICP solution without yaml

See [examples/icp_customized.cpp](../examples/icp_customized.cpp) for a working example.

Here are the important part of the example. First, generate an empty ICP object with some generic variables:
```c++
// Create the default ICP algorithm
PM::ICP icp;
PointMatcherSupport::Parametrizable::Parameters params;
std::string name;
```
Prepare the objects for the DataFilters:
```c++
// Prepare reading filters
name = "MinDistDataPointsFilter";
params["minDist"] = "1.0";
PM::DataPointsFilter* minDist_read = 
	PM::get().DataPointsFilterRegistrar.create(name, params);
params.clear();

name = "RandomSamplingDataPointsFilter";
params["prob"] = "0.05";
PM::DataPointsFilter* rand_read = 
	PM::get().DataPointsFilterRegistrar.create(name, params);
params.clear();

// Prepare reference filters
name = "MinDistDataPointsFilter";
params["minDist"] = "1.0";
PM::DataPointsFilter* minDist_ref = 
	PM::get().DataPointsFilterRegistrar.create(name, params);
params.clear();

name = "RandomSamplingDataPointsFilter";
params["prob"] = "0.05";
PM::DataPointsFilter* rand_ref = 
	PM::get().DataPointsFilterRegistrar.create(name, params);
params.clear();
```

Prepare the objects for the Matchers:
```c++
// Prepare matching function
name = "KDTreeMatcher";
params["knn"] = "1";
params["epsilon"] = "3.16";
PM::Matcher* kdtree = 
	PM::get().MatcherRegistrar.create(name, params);
params.clear();
```

Prepare the objects for the OutlierFilters:
```c++
// Prepare outlier filters
name = "TrimmedDistOutlierFilter";
params["ratio"] = "0.75";
PM::OutlierFilter* trim = 
	PM::get().OutlierFilterRegistrar.create(name, params);
params.clear();
```

Prepare the object for the ErrorMinimizer:
```c++
// Prepare error minimization
name = "PointToPointErrorMinimizer";
PM::ErrorMinimizer* pointToPoint =   
	PM::get().ErrorMinimizerRegistrar.create(name);
```

Prepare the objects for the TransformationCheckers:
```c++
// Prepare outlier filters
name = "CounterTransformationChecker";
params["maxIterationCount"] = "150";
PM::TransformationChecker* maxIter = 
	PM::get().TransformationCheckerRegistrar.create(name, params);
params.clear();

name = "DifferentialTransformationChecker";
params["minDiffRotErr"] = "0.001";
params["minDiffTransErr"] = "0.01";
params["smoothLength"] = "4";
PM::TransformationChecker* diff = 
	PM::get().TransformationCheckerRegistrar.create(name, params);
params.clear();

```

Prepare the objects for the Inspector:
```c++
// Prepare inspector
PM::Inspector* nullInspect =
	PM::get().InspectorRegistrar.create("NullInspector");

```

Prepare the objects for the Transformation:
```c++	
// Prepare transformation
PM::Transformation* rigidTrans =
	PM::get().TransformationRegistrar.create("RigidTransformation");
```
Finally, build the complet solution:
```c++
// Build ICP solution
icp.readingDataPointsFilters.push_back(minDist_read);
icp.readingDataPointsFilters.push_back(rand_read);

icp.referenceDataPointsFilters.push_back(minDist_ref);
icp.referenceDataPointsFilters.push_back(rand_ref);

icp.matcher.reset(kdtree);
	
icp.outlierFilters.push_back(trim);
	
icp.errorMinimizer.reset(pointToPoint);

icp.transformationCheckers.push_back(maxIter);
icp.transformationCheckers.push_back(diff);
	
icp.inspector.reset(nullInspect);

icp.transformations.push_back(rigidTrans);
```
