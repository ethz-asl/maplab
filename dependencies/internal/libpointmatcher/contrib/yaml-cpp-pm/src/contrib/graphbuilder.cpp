#include "yaml-cpp-pm/parser.h"
#include "yaml-cpp-pm/contrib/graphbuilder.h"
#include "graphbuilderadapter.h"

namespace YAML_PM
{
  void *BuildGraphOfNextDocument(Parser& parser, GraphBuilderInterface& graphBuilder)
  {
    GraphBuilderAdapter eventHandler(graphBuilder);
    if (parser.HandleNextDocument(eventHandler)) {
      return eventHandler.RootNode();
    } else {
      return NULL;
    }
  }
}
