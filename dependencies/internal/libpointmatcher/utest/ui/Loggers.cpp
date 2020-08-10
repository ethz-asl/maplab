#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// Loggers
//---------------------------

TEST(Loggers, FileLogger)
{
	string infoFileName = "utest_info";
	string warningFileName = "utest_warn";

	std::shared_ptr<Logger> fileLog =
		PM::get().REG(Logger).create(
			"FileLogger", {
				{"infoFileName", infoFileName},
				{ "warningFileName", warningFileName },
				{ "displayLocation", "1" }
			}
		);

	fileLog.reset(); // The logger needs to release the files to allow them to be removed

	EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(infoFileName)));
	EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(warningFileName)));
}

TEST(Loggers, FileLoggerInfoToConsole)
{
	std::shared_ptr<Logger> fileLog =
		PM::get().REG(Logger).create(
			"FileLogger", {
				{ "displayLocation", "1" }
			}
		);
	EXPECT_NO_THROW((*fileLog->infoStream()) << "TEST\n");
}

TEST(Loggers, FileLoggerWarningToConsole)
{
	std::shared_ptr<Logger> fileLog =
		PM::get().REG(Logger).create(
			"FileLogger", {
				{ "displayLocation", "1" }
			}
		);
	EXPECT_NO_THROW((*fileLog->warningStream()) << "TEST\n");
}

TEST(Loggers, FileLoggerInfoToFile)
{
	string infoFileName = "utest_info";

	std::shared_ptr<Logger> fileLog =
		PM::get().REG(Logger).create(
			"FileLogger", {
				{"infoFileName", infoFileName},
				{ "displayLocation", "1" }
			}
		);

	EXPECT_NO_THROW((*fileLog->infoStream()) << "TEST");

	fileLog.reset(); // The logger needs to release the file to allow it to be removed

	EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(infoFileName)));
}

TEST(Loggers, FileLoggerWarningToFile)
{
	string warningFileName = "utest_warn";

	std::shared_ptr<Logger> fileLog =
		PM::get().REG(Logger).create(
			"FileLogger", {
				{ "warningFileName", warningFileName },
				{ "displayLocation", "1" }
			}
		);

	EXPECT_NO_THROW((*fileLog->warningStream()) << "TEST");

	fileLog.reset(); // The logger needs to release the file to allow it to be removed

	EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(warningFileName)));
}
