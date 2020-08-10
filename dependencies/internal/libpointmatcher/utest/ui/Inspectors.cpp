#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// Inspectors
//---------------------------
TEST(Inspectors, PerformanceInspector)
{
	std::shared_ptr<PM::Inspector> performances =
		PM::get().REG(Inspector).create(
			"PerformanceInspector", {
				{"baseFileName", "/tmp/utest_performances"},
				{"dumpPerfOnExit", "1"}
			}
		)
	;

  performances->init();

	//TODO: we only test constructor here, check other things...
}

TEST(Inspectors, VTKFileInspector)
{
	std::shared_ptr<PM::Inspector> vtkFile =
		PM::get().REG(Inspector).create(
			"VTKFileInspector", {
				{"baseFileName", "/tmp/utest_vtk"},
				{"dumpPerfOnExit", "1"}
			}
		);
	//TODO: we only test constructor here, check other things...
}
