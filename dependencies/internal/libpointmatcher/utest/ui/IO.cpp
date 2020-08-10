#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// Tests for IO
//---------------------------

TEST(IOTest, loadYaml)
{

	// Test loading configuration files for data filters
	std::ifstream ifs0((dataPath + "default-convert.yaml").c_str());
	EXPECT_NO_THROW(PM::DataPointsFilters filters(ifs0));

	// Test loading configuration files for ICP
	PM::ICP icp;

	std::ifstream ifs1((dataPath + "default.yaml").c_str());
	EXPECT_NO_THROW(icp.loadFromYaml(ifs1));

	std::ifstream ifs2((dataPath + "unit_tests/badIcpConfig_InvalidParameter.yaml").c_str());
	EXPECT_THROW(icp.loadFromYaml(ifs2), PointMatcherSupport::Parametrizable::InvalidParameter);
	
	std::ifstream ifs3((dataPath + "unit_tests/badIcpConfig_InvalidModuleType.yaml").c_str());
	EXPECT_THROW(icp.loadFromYaml(ifs3), PointMatcherSupport::InvalidModuleType);
}

TEST(IOTest, loadCSV)
{
  typedef PointMatcherIO<float> IO;
	std::istringstream is;
  std::ostringstream os;
  DP pts;

  // too many elements on a line
  os.clear();
  os.str("");
  os <<
  "x, y\n"
  "1, 2\n"
  "1, 2, 3\n"
  "1, 2\n"
  ;

  is.clear();
  is.str(os.str());
  EXPECT_THROW(IO::loadCSV(is), runtime_error);

  // not enough elements on a line
  os.clear();
  os.str("");
  os <<
  "x, y\n"
  "1, 2\n"
  "1, \n"
  "1, 2\n"
  ;

  is.clear();
  is.str(os.str());
  EXPECT_THROW(IO::loadCSV(is), runtime_error);
  
  // 2D data 
  os.clear();
  os.str("");
  os <<
  "x, y\n"
  "1, 2\n"
  "1, 2\n"
  "8, 2\n"
  "1, 2\n"
  ;

  is.clear();
  is.str(os.str());
  pts = IO::loadCSV(is);
  EXPECT_EQ(4u, pts.getNbPoints());
  EXPECT_EQ(2u, pts.getEuclideanDim());
  EXPECT_EQ(0u, pts.getDescriptorDim());
  EXPECT_EQ(0u, pts.getTimeDim());
  EXPECT_EQ(8.0, pts.features(0,2));
  
  // 3D data 
  os.clear();
  os.str("");
  os <<
  "x, y, z\n"
  "1, 2, 3\n"
  "1, 2, 3\n"
  "8, 2, 3\n"
  "1, 2, 3\n"
  ;

  is.clear();
  is.str(os.str());
  pts = IO::loadCSV(is);
  EXPECT_EQ(4u, pts.getNbPoints());
  EXPECT_EQ(3u, pts.getEuclideanDim());
  EXPECT_EQ(0u, pts.getDescriptorDim());
  EXPECT_EQ(0u, pts.getTimeDim());
  EXPECT_EQ(8.0, pts.features(0,2));

  // 3D data with unknown descriptor
  os.clear();
  os.str("");
  os <<
  "x, y, z, dummy\n"
  "1, 2, 3, 4\n"
  "1, 2, 3, 4\n"
  "8, 2, 3, 4\n"
  "1, 2, 3, 4\n"
  ;

  is.clear();
  is.str(os.str());
  pts = IO::loadCSV(is);
  EXPECT_EQ(4u, pts.getNbPoints());
  EXPECT_EQ(3u, pts.getEuclideanDim());
  EXPECT_EQ(1u, pts.getDescriptorDim());
  EXPECT_EQ(0u, pts.getTimeDim());
  EXPECT_TRUE(pts.descriptorExists("dummy"));

  // 3D data with known descriptor
  os.clear();
  os.str("");
  os <<
  "x, y, z, nx, ny, nz\n"
  "1, 2, 3, 4, 5, 6\n"
  "1, 2, 3, 4, 5, 6\n"
  "8, 2, 3, 4, 5, 6\n"
  "1, 2, 3, 4, 5, 6\n"
  ;

  is.clear();
  is.str(os.str());
  pts = IO::loadCSV(is);
  EXPECT_EQ(4u, pts.getNbPoints());
  EXPECT_EQ(3u, pts.getEuclideanDim());
  EXPECT_EQ(3u, pts.getDescriptorDim());
  EXPECT_EQ(1u, pts.getNbGroupedDescriptors());
  EXPECT_EQ(0u, pts.getTimeDim());
  EXPECT_TRUE(pts.descriptorExists("normals"));
  
  // csv with time
  int64_t time0 = 1410264593275569438;
  int64_t time1 = 1410264593325569391;
  int64_t time2 = 1410264593425569295;
  int64_t time3 = 1410264593522569417;

  os.clear();
  os.str("");
	os <<
	"x, y, z, time\n"
	"1, 1, 1, " << time0 << "\n"
	"2, 1, 1, " << time1 << "\n"
	"3, 1, 1, " << time2 << "\n"
	"4, 1, 1, " << time3 << "\n"
	;

  is.clear();
  is.str(os.str());

  pts = IO::loadCSV(is);
  EXPECT_EQ(4u, pts.getNbPoints());
  EXPECT_EQ(3u, pts.getEuclideanDim());
  EXPECT_EQ(0u, pts.getDescriptorDim());
  EXPECT_EQ(1u, pts.getTimeDim());
  EXPECT_EQ(time3, pts.times(0,3));

  //cout << "dim: " << pts.getEuclideanDim() << endl;
  //cout << "nb pts: " << pts.getNbPoints() << endl;
  //cout << "desc dim: " << pts.getDescriptorDim() << endl;
  //cout << "time dim: " << pts.getTimeDim() << endl;



}

TEST(IOTest, loadPLY)
{
	typedef PointMatcherIO<float> IO;
	std::istringstream is;
	
	is.str(
	""
	);

	EXPECT_THROW(IO::loadPLY(is), runtime_error);

	is.clear();
	is.str(
	"ply\n"
	"format binary_big_endian 1.0\n"
	);

	EXPECT_THROW(IO::loadPLY(is), runtime_error);
	
	is.clear();
	is.str(
	"ply\n"
	"format ascii 2.0\n"
	);
	
	EXPECT_THROW(IO::loadPLY(is), runtime_error);

	is.clear();
	is.str(
	"ply\n"
	"format ascii 1.0\n"
	);
	
	EXPECT_THROW(IO::loadPLY(is), runtime_error);

	is.clear();
	is.str(
	"ply\n"
	"format ascii 1.0\n"
	"element vertex 5\n"
	"\n" //empty line
	"property float z\n" // wrong order
	"property float y\n"
	"property float x\n"
	"property float grrrr\n" //unknown property
	"property float nz\n" // wrong order
	"property float ny\n"
	"property float nx\n"
	"end_header\n"
	"3 2 1 99 33 22 11\n"
	"3 2 1 99 33 22 11\n"
	"\n" //empty line
	"3 2 1 99 33 22 11 3 2 1 99 33 22 11\n" // no line break
	"3 2 1 99 33 22 11\n"

	);
	
	DP pointCloud = IO::loadPLY(is);
	
	// Confirm sizes and dimensions
	EXPECT_TRUE(pointCloud.features.cols() == 5);
	EXPECT_TRUE(pointCloud.features.rows() == 4); //x, y, z, pad
	EXPECT_TRUE(pointCloud.descriptors.cols() == 5);
	EXPECT_TRUE(pointCloud.descriptors.rows() == 4);//nx, ny, nz, grrrr
	
	// Random value check
	EXPECT_TRUE(pointCloud.features(0, 0) == 1);
	EXPECT_TRUE(pointCloud.features(2, 2) == 3);
	EXPECT_TRUE(pointCloud.descriptors(1, 1) == 22);
	EXPECT_TRUE(pointCloud.descriptors(2, 4) == 33);

}


TEST(IOTest, loadPCD)
{
	typedef PointMatcherIO<float> IO;
	std::istringstream is;

	// Empty file
	is.str(
	""
	);

	EXPECT_THROW(IO::loadPCD(is), runtime_error);

	// Partial header
	is.clear();
	is.str(
	"# .PCD v.7 - Point Cloud Data file format\n"
	"VERSION .7\n"
	);

	EXPECT_THROW(IO::loadPCD(is), runtime_error);
	
	// Missing data
	is.clear();
	is.str(
	"# .PCD v.7 - Point Cloud Data file format\n"
	"VERSION .7\n"
	"FIELDS x y z\n"
	"SIZE 4 4 4\n"
	"TYPE F F F\n"
	"COUNT 1 1 1\n"
	"WIDTH 18\n"
	"HEIGHT 1\n"
	"VIEWPOINT 0 0 0 1 0 0 0\n"
	"POINTS 18\n"
	"DATA ascii\n"
	);

	EXPECT_THROW(IO::loadPCD(is), runtime_error);


	is.clear();
	is.str(
	"# .PCD v.7 - Point Cloud Data file format\n"
	"VERSION .7\n"
	"FIELDS x y z grrr\n"
	"SIZE 4 4 4 4\n"
	"TYPE F F F F\n"
	"COUNT 1 1 1 1\n"
	"WIDTH 18\n"
	"HEIGHT 1\n"
	"VIEWPOINT 0 0 0 1 0 0 0\n"
	"POINTS 18\n"
	"DATA ascii\n"
	"0.44912094 0.49084857 1.153 22\n"
	"0.34907714 0.48914573 1.149 22\n"
	"0.33813429 0.48914573 1.149 22\n"
	"0.32833049 0.49084857 1.153 22\n"
	"0.24395333 0.42856666 0.98900002 22\n"
	"0.20816095 0.42856666 0.98900002 22\n"
	"0.1987419 0.42291525 0.98900002 22\n"
	"0.18178761 0.42291525 0.98900002 22\n"
	"0.17990381 0.42291525 0.98900002 22\n"
	"-0.035590474 0.42997143 1.01 22\n"
	"-0.035907622 0.43962574 1.0190001 22\n"
	"-0.043542858 0.43639618 1.016 22\n"
	"-0.15246001 0.36058003 0.84700006 22\n"
	"0.21956001 0.44007048 0.99800003 22\n"
	"-0.16635144 0.3699457 0.86900002 22\n"
	"-0.33879143 0.36143145 0.84900004 22\n"
	"-0.35055432 0.36853144 0.85800004 22\n"
	"-0.39932001 0.38058859 0.89400005 22\n"
	);
	
	DP pointCloud = IO::loadPCD(is);
	
	// Confirm sizes and dimensions
	EXPECT_TRUE(pointCloud.features.cols() == 18);
	EXPECT_TRUE(pointCloud.features.rows() == 4);
	EXPECT_TRUE(pointCloud.descriptors.cols() == 18);
	EXPECT_TRUE(pointCloud.descriptors.rows() == 1);
	
	// Random value check
	EXPECT_NEAR(pointCloud.features(0, 0), 0.449121, 0.0000001);
	EXPECT_NEAR(pointCloud.features(2, 2), 1.149, 0.0000001);
	EXPECT_NEAR(pointCloud.descriptors(0, 10), 22, 0.0000001);

}

class IOLoadSaveTest : public testing::Test
{

public:
	virtual void SetUp()
	{
		nbPts = 10;
		addRandomFeature("x", 1);
		addRandomFeature("y", 1);
		addRandomFeature("z", 1);
		ptCloud.addFeature("pad", PM::Matrix::Ones(1, nbPts));

		addRandomDescriptor("normals",3);
		addRandomDescriptor("eigValues",3);
		addRandomDescriptor("eigVectors",9);
		addRandomDescriptor("color",4);
	}

	void addRandomFeature(const string& featureName, const int rows)
	{
		ptCloud.addFeature(featureName,PM::Matrix::Random(rows, nbPts));
	}

	void addRandomDescriptor(const string& descriptorName, const int rows)
	{
		ptCloud.addDescriptor(descriptorName, PM::Matrix::Random(rows, nbPts));
	}

	virtual void loadSaveTest(const string& testFileName, bool plyFormat = false, const int nbPts = 10, bool binary = false)
	{
		this->testFileName = testFileName;

		if (plyFormat || binary) {
			// make sure random values generated for colors are within ply format range
			int pointCount(ptCloud.features.cols());
			int descRows(ptCloud.descriptors.rows());
			bool datawithColor = ptCloud.descriptorExists("color");
			int colorStartingRow = ptCloud.getDescriptorStartingRow("color");
			int colorEndRow = colorStartingRow + ptCloud.getDescriptorDimension("color");
			for (int p = 0; p < pointCount; ++p)
			{
				for (int d = 0; d < descRows; ++d)
				{
					if (datawithColor && d >= colorStartingRow && d < colorEndRow) {
						if (ptCloud.descriptors(d, p) < 0) { ptCloud.descriptors.coeffRef(d, p) = -(ptCloud.descriptors(d, p)); }
						ptCloud.descriptors.coeffRef(d, p) = (static_cast<unsigned>(ptCloud.descriptors(d, p) * 255.0)) / 255.0;
					}
				}
			}
		}

		ptCloud.save(testFileName, binary);

		ptCloudFromFile = DP::load(testFileName);

		EXPECT_TRUE(ptCloudFromFile.features.cols() == ptCloud.features.cols());
		EXPECT_TRUE(ptCloudFromFile.descriptorExists("normals",3));
		EXPECT_TRUE(ptCloudFromFile.getDescriptorViewByName("normals").isApprox(ptCloud.getDescriptorViewByName("normals")));
		EXPECT_TRUE(ptCloudFromFile.descriptorExists("eigValues",3));
		EXPECT_TRUE(ptCloudFromFile.getDescriptorViewByName("eigValues").isApprox(ptCloud.getDescriptorViewByName("eigValues")));
		EXPECT_TRUE(ptCloudFromFile.descriptorExists("eigVectors",9));
		EXPECT_TRUE(ptCloudFromFile.getDescriptorViewByName("eigVectors").isApprox(ptCloud.getDescriptorViewByName("eigVectors")));
		EXPECT_TRUE(ptCloudFromFile.descriptorExists("color",4));
		if (plyFormat || binary) {
			EXPECT_TRUE(((ptCloudFromFile.getDescriptorViewByName("color") * 255.0)).isApprox((ptCloud.getDescriptorViewByName("color") * 255.0), 1.0));
		} else {
			EXPECT_TRUE(ptCloudFromFile.getDescriptorViewByName("color").isApprox(ptCloud.getDescriptorViewByName("color")));
		}

		EXPECT_TRUE(ptCloudFromFile.features.isApprox(ptCloud.features));

	}

	virtual void TearDown()
	{
		EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(testFileName)));
	}


protected:
	int nbPts;
	DP::Labels featureLabels;
	DP ptCloud;
	DP ptCloudFromFile;
	string testFileName;

};

TEST_F(IOLoadSaveTest, VTK)
{
	ptCloud.addDescriptor("genericScalar", PM::Matrix::Random(1, nbPts));
	ptCloud.addDescriptor("genericVector", PM::Matrix::Random(3, nbPts));
	ptCloud.addTime("genericTime", PM::Int64Matrix::Random(1, nbPts));

	loadSaveTest(dataPath + "unit_test.vtk");

	EXPECT_TRUE(ptCloudFromFile.descriptorExists("genericScalar",1));
	EXPECT_TRUE(ptCloudFromFile.descriptorExists("genericVector",3));
	EXPECT_TRUE(ptCloudFromFile.timeExists("genericTime",1));

}

TEST_F(IOLoadSaveTest, VTKBinary)
{
	ptCloud.addDescriptor("genericScalar", PM::Matrix::Random(1, nbPts));
	ptCloud.addDescriptor("genericVector", PM::Matrix::Random(3, nbPts));
	ptCloud.addTime("genericTime", PM::Int64Matrix::Random(1, nbPts));

	loadSaveTest(dataPath + "unit_test.bin.vtk", false, 10, true);

	EXPECT_TRUE(ptCloudFromFile.descriptorExists("genericScalar",1));
	EXPECT_TRUE(ptCloudFromFile.descriptorExists("genericVector",3));
	EXPECT_TRUE(ptCloudFromFile.timeExists("genericTime",1));
}

TEST_F(IOLoadSaveTest, PLY)
{
	loadSaveTest(dataPath + "unit_test.ply", true);
}

TEST_F(IOLoadSaveTest, PCD)
{
	loadSaveTest(dataPath + "unit_test.pcd");
}

TEST_F(IOLoadSaveTest, CSV)
{
	loadSaveTest(dataPath + "unit_test.csv");
}
