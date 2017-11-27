#include <algorithm>

#include <maplab-common/file-system-tools.h>
#include <maplab-common/test/testing-entrypoint.h>

namespace common {

TEST(MaplabCommon, TestTokenizeString) {
  static const std::string kValidDelimiter = ",";
  static const std::string kValidTokenString = "a,b";
  static const std::string kSpecialTokenString1 = ",a";
  static const std::string kSpecialTokenString2 = "a,";
  static const std::string kSpecialTokenString3 = ",";
  static const std::string kSpecialTokenString4 = ",,";
  static const std::string kSpecialTokenString5 = ",b,";
  static const std::string kSpecialTokenString6 = ",,b";
  static const std::string kSpecialTokenString7 = "b,,";
  static const std::string kSpecialTokenString8 = "a,,b";
  static const std::string kSpecialTokenString9 = "a,,,,b";
  static const std::string kEmptyString = "";

  static const std::string kExampleTokenString = "Bla,foo-bar. O K";
  static const std::string kExampleDelimitersString = ",-. ";

  std::vector<std::string> tokens;

  EXPECT_DEATH(tokenizeString(kEmptyString, kValidDelimiter, &tokens), "empty");
  tokens.clear();
  EXPECT_DEATH(
      tokenizeString(kValidTokenString, kEmptyString, &tokens), "empty");
  tokens.clear();
  EXPECT_DEATH(tokenizeString(kEmptyString, kEmptyString, &tokens), "empty");
  tokens.clear();

  tokenizeString(kSpecialTokenString1, kValidDelimiter, &tokens);
  ASSERT_EQ(1u, tokens.size());
  EXPECT_EQ("a", tokens[0]);
  tokens.clear();

  tokenizeString(kSpecialTokenString2, kValidDelimiter, &tokens);
  ASSERT_EQ(1u, tokens.size());
  EXPECT_EQ("a", tokens[0]);
  tokens.clear();

  tokenizeString(kSpecialTokenString3, kValidDelimiter, &tokens);
  EXPECT_EQ(tokens.size(), 0u);
  tokens.clear();

  tokenizeString(kSpecialTokenString4, kValidDelimiter, &tokens);
  EXPECT_EQ(tokens.size(), 0u);
  tokens.clear();

  tokenizeString(kSpecialTokenString5, kValidDelimiter, &tokens);
  ASSERT_EQ(1u, tokens.size());
  EXPECT_EQ("b", tokens[0]);
  tokens.clear();

  tokenizeString(kSpecialTokenString6, kValidDelimiter, &tokens);
  ASSERT_EQ(1u, tokens.size());
  EXPECT_EQ("b", tokens[0]);
  tokens.clear();

  tokenizeString(kSpecialTokenString7, kValidDelimiter, &tokens);
  ASSERT_EQ(1u, tokens.size());
  EXPECT_EQ("b", tokens[0]);
  tokens.clear();

  tokenizeString(kSpecialTokenString8, kValidDelimiter, &tokens);
  ASSERT_EQ(2u, tokens.size());
  EXPECT_EQ("a", tokens[0]);
  EXPECT_EQ("b", tokens[1]);
  tokens.clear();

  tokenizeString(kSpecialTokenString9, kValidDelimiter, &tokens);
  ASSERT_EQ(2u, tokens.size());
  EXPECT_EQ("a", tokens[0]);
  EXPECT_EQ("b", tokens[1]);
  tokens.clear();

  tokenizeString(kExampleTokenString, kExampleDelimitersString, &tokens);
  ASSERT_EQ(5u, tokens.size());
  EXPECT_EQ("Bla", tokens[0]);
  EXPECT_EQ("foo", tokens[1]);
  EXPECT_EQ("bar", tokens[2]);
  EXPECT_EQ("O", tokens[3]);
  EXPECT_EQ("K", tokens[4]);
  tokens.clear();
}

TEST(MaplabCommon, pathExistsTest) {
  EXPECT_TRUE(pathExists("maplab_test_data/testfolder"));
  EXPECT_FALSE(pathExists("maplab_test_data/inexistentfolder"));
}

TEST(MaplabCommon, fileExistsTest) {
  EXPECT_TRUE(fileExists("maplab_test_data/testfile.txt"));
  EXPECT_FALSE(fileExists("maplab_test_data/inexistentfile.foo"));
}

TEST(MaplabCommon, copyFileTest) {
  constexpr int kMode = 0777;
  constexpr bool kOverwrite = true;
  constexpr bool kDontOverwrite = false;

  const std::string kSourceFile = "maplab_test_data/testfile.txt";
  const std::string kTargetFile = "maplab_test_data/testfile.txt-copy";

  EXPECT_TRUE(fileExists(kSourceFile));
  EXPECT_TRUE(copyFile(kSourceFile, kTargetFile, kMode, kDontOverwrite));
  EXPECT_FALSE(copyFile(kSourceFile, kTargetFile, kMode, kDontOverwrite));
  EXPECT_TRUE(copyFile(kSourceFile, kTargetFile, kMode, kOverwrite));

  EXPECT_TRUE(deleteFile(kTargetFile));
}

TEST(MaplabCommon, getCurrentWorkingDirectoryTest) {
  const std::string current_working_directory = getCurrentWorkingDirectory();
  EXPECT_FALSE(current_working_directory.empty());
}

TEST(MaplabCommon, getAllFilesAndFoldersInFolderTest) {
  const std::string current_working_directory = getCurrentWorkingDirectory();

  const std::string root_folder =
      current_working_directory + "/maplab_test_data/subfolder_test/";
  const std::string subfolder_path_1 =
      current_working_directory + "/maplab_test_data/subfolder_test/test_1";
  const std::string subfolder_path_2 =
      current_working_directory + "/maplab_test_data/subfolder_test/test_2";
  const std::string subfolder_path_3 =
      current_working_directory +
      "/maplab_test_data/subfolder_test/test_2/test_2_1";
  const std::string subfolder_path_4 =
      current_working_directory +
      "/maplab_test_data/subfolder_test/test_2/test_2_2";

  // Create paths
  EXPECT_TRUE(common::createPath(root_folder));
  EXPECT_TRUE(common::createPath(subfolder_path_1));
  EXPECT_TRUE(common::createPath(subfolder_path_2));
  EXPECT_TRUE(common::createPath(subfolder_path_3));
  EXPECT_TRUE(common::createPath(subfolder_path_4));

  const std::string subfolder_file_path_1 =
      subfolder_path_1 + "/test_file_1.txt";
  const std::string subfolder_file_path_2 =
      subfolder_path_2 + "/test_file_2.txt";
  const std::string subfolder_file_path_3 =
      subfolder_path_3 + "/test_file_3.txt";
  const std::string subfolder_file_path_4 =
      subfolder_path_4 + "/test_file_4.txt";

  // Create files
  constexpr int kMode = 0777;
  constexpr bool kOverwrite = true;
  EXPECT_TRUE(
      copyFile(
          "maplab_test_data/testfile.txt", subfolder_file_path_1, kMode,
          kOverwrite));
  EXPECT_TRUE(
      copyFile(
          "maplab_test_data/testfile.txt", subfolder_file_path_2, kMode,
          kOverwrite));
  EXPECT_TRUE(
      copyFile(
          "maplab_test_data/testfile.txt", subfolder_file_path_3, kMode,
          kOverwrite));
  EXPECT_TRUE(
      copyFile(
          "maplab_test_data/testfile.txt", subfolder_file_path_4, kMode,
          kOverwrite));

  std::vector<std::string> expected_file_paths{
      subfolder_file_path_1, subfolder_file_path_2, subfolder_file_path_3,
      subfolder_file_path_4};
  std::vector<std::string> expected_file_names{
      "test_file_1.txt", "test_file_2.txt", "test_file_3.txt",
      "test_file_4.txt"};
  std::vector<std::string> expected_folder_paths{
      subfolder_path_1, subfolder_path_2, subfolder_path_3, subfolder_path_4};

  std::vector<std::string> file_paths;
  std::vector<std::string> file_names;
  std::vector<std::string> folder_paths;
  const int return_value = getAllFilesAndFoldersInFolder(
      root_folder, &file_paths, &file_names, &folder_paths);

  EXPECT_TRUE(
      std::is_permutation(
          expected_file_paths.begin(), expected_file_paths.end(),
          file_paths.begin()));
  EXPECT_TRUE(
      std::is_permutation(
          expected_file_names.begin(), expected_file_names.end(),
          file_names.begin()));
  EXPECT_TRUE(
      std::is_permutation(
          expected_folder_paths.begin(), expected_folder_paths.begin(),
          folder_paths.begin()));

  EXPECT_EQ(return_value, 0);
}

TEST(MaplabCommon, isSameRealPathTest) {
  const std::string current_working_directory = getCurrentWorkingDirectory();
  const std::string full_path =
      current_working_directory + "/maplab_test_data/";
  const std::string full_path_2 =
      current_working_directory + "/maplab_test_data/../maplab_test_data/";
  const std::string full_path_3 =
      current_working_directory + "/maplab_test_data/../";
  const std::string relative_path = "maplab_test_data/";
  const std::string relative_path_2 = "maplab_test_data/../";

  const std::string file_that_does_not_exist =
      "maplab_test_data/some_folder_that_doesnt_exist";

  ASSERT_TRUE(pathExists(full_path));
  ASSERT_TRUE(pathExists(full_path_2));
  ASSERT_TRUE(pathExists(full_path_3));
  ASSERT_TRUE(pathExists(relative_path));
  ASSERT_TRUE(pathExists(relative_path_2));

  EXPECT_TRUE(isSameRealPath(full_path, full_path));
  EXPECT_TRUE(isSameRealPath(full_path_2, full_path));
  EXPECT_TRUE(isSameRealPath(relative_path, full_path));

  EXPECT_FALSE(isSameRealPath(full_path_3, full_path));
  EXPECT_FALSE(isSameRealPath(relative_path_2, full_path));

  EXPECT_DEATH(
      isSameRealPath(file_that_does_not_exist, full_path), "pathExists");
}

TEST(MaplabCommon, isSameRealFilePathTest) {
  const std::string current_working_directory = getCurrentWorkingDirectory();
  const std::string full_file_path =
      current_working_directory + "/maplab_test_data/testfile.txt";
  const std::string full_file_path_2 =
      current_working_directory +
      "/maplab_test_data/../maplab_test_data/testfile.txt";
  const std::string full_file_path_3 =
      current_working_directory + "/maplab_test_data/testfile.txt-copy";
  const std::string relative_file_path = "maplab_test_data/testfile.txt";
  const std::string relative_file_path_2 = "maplab_test_data/testfile.txt-copy";

  const std::string file_that_does_not_exist =
      "maplab_test_data/some_file_that_doesnt_exist.txt";

  constexpr int kMode = 0777;
  constexpr bool kOverwrite = true;
  EXPECT_TRUE(
      copyFile(
          "maplab_test_data/testfile.txt", "maplab_test_data/testfile.txt-copy",
          kMode, kOverwrite));

  ASSERT_TRUE(fileExists(full_file_path));
  ASSERT_TRUE(fileExists(full_file_path_2));
  ASSERT_TRUE(fileExists(full_file_path_3));
  ASSERT_TRUE(fileExists(relative_file_path));
  ASSERT_TRUE(fileExists(relative_file_path_2));

  EXPECT_TRUE(isSameRealFilePath(full_file_path, full_file_path));
  EXPECT_TRUE(isSameRealFilePath(full_file_path_2, full_file_path));
  EXPECT_TRUE(isSameRealFilePath(relative_file_path, full_file_path));

  EXPECT_FALSE(isSameRealFilePath(full_file_path_3, full_file_path));
  EXPECT_FALSE(isSameRealFilePath(relative_file_path_2, full_file_path));

  EXPECT_DEATH(
      isSameRealFilePath(file_that_does_not_exist, full_file_path),
      "fileExists");
}

TEST(MaplabCommon, getFileListTest) {
  EXPECT_TRUE(fileExists("maplab_test_data/testfile.txt"));
  std::vector<std::string> initial_paths(1, "maplab_test_data");
  constexpr bool kSortLexical = true;
  const std::string extension_filter = "txt";
  std::vector<std::string> file_paths;
  EXPECT_EQ(
      0,
      getFileLists(initial_paths, kSortLexical, extension_filter, &file_paths));
  ASSERT_EQ(1u, file_paths.size());
  EXPECT_EQ("maplab_test_data/testfile.txt", file_paths[0]);
}

TEST(MaplabCommon, splitPathAndFilename) {
  std::string path;
  std::string filename;

  common::splitPathAndFilename("/usr/bin/bash.bin", &path, &filename);
  EXPECT_EQ(path, "/usr/bin");
  EXPECT_EQ(filename, "bash.bin");

  common::splitPathAndFilename("/usr/bin/", &path, &filename);
  EXPECT_EQ(path, "/usr/bin");
  EXPECT_EQ(filename, "");

  common::splitPathAndFilename("bash.bin", &path, &filename);
  EXPECT_EQ(path, "");
  EXPECT_EQ(filename, "bash.bin");
}

TEST(MaplabCommon, splitFilePathAndExtension) {
  std::string filename;
  std::string extension;

  common::splitFilePathAndExtension("bash.bin", &filename, &extension);
  EXPECT_EQ(filename, "bash");
  EXPECT_EQ(extension, "bin");

  common::splitFilePathAndExtension("bash", &filename, &extension);
  EXPECT_EQ(filename, "bash");
  EXPECT_EQ(extension, "");

  common::splitFilePathAndExtension(".bin", &filename, &extension);
  EXPECT_EQ(filename, "");
  EXPECT_EQ(extension, "bin");
}

TEST(MaplabCommon, isSamePath) {
  std::string string_a = "/foo/bar";
  std::string string_b = "/foo/bar";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "./foo/bar/";
  string_b = "./foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "foo/bar/";
  string_b = "foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));

  // Trailing slashes.
  string_a = "/foo/bar/";
  string_b = "/foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "/foo/bar";
  string_b = "/foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));

  // Different paths.
  string_a = "/foo/bar";
  string_b = "/bar/foo";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "/foo/bar";
  string_b = "foo/bar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "/foo/bar/foo/bar";
  string_b = "/foo/bar/foo/";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "/foo/bar/foo/bar";
  string_b = "/foobarfoobar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "foo/bar/foo/bar";
  string_b = "foobarfoobar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));

  // Duplicate slashes
  string_a = "/foo/bar//";
  string_b = "/foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "/foo/bar/";
  string_b = "/bar//foo/";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "foo//bar";
  string_b = "foobar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "//foobar";
  string_b = "foobar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "foobar//";
  string_b = "foobar";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "foobar//";
  string_b = "foobar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "/foo/bar///";
  string_b = "/foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "/foo/bar////";
  string_b = "/foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "/foo//bar/";
  string_b = "/foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "/foo///bar/";
  string_b = "/foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "//foo/bar/";
  string_b = "/foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "///foo/bar/";
  string_b = "/foo/bar/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "//";
  string_b = "/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "///";
  string_b = "/";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = ".//";
  string_b = "./";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = ".///";
  string_b = "./";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));

  // /./, ./ and /.
  string_a = "./foo";
  string_b = "foo";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "./foo/.";
  string_b = "foo";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "./foo/./";
  string_b = "foo";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "./foo/./.";
  string_b = "foo";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "././foo/./.";
  string_b = "foo";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "/././foo/././";
  string_b = "/foo";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "././foo/./bar/.";
  string_b = "./foo/bar";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = ".///./foo//.//bar//.";
  string_b = "./foo/bar";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "//.///./foo//.//bar//.//";
  string_b = "/foo/bar";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));

  // Handle hidden folders and filenames with periods.
  string_a = "foo/.bar";
  string_b = "foo/bar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "foo/bar.foo";
  string_b = "foo/barfoo";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "foo./bar";
  string_b = "foo/bar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "//.///./.foo//.//.bar//.//";
  string_b = "/.foo/.bar";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "//.///./.foo//.//.bar//.//foo.bar//";
  string_b = "/.foo/.bar/foo.bar";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "//.///./.foo//.//.bar//.//foo.bar//";
  string_b = "/foo/bar/foo.bar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));

  // Unsupported ".." cases. Will log to error.
  // Some simple cases actually work.
  string_a = ".///foo/../bar/.";
  string_b = "foo/../bar";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = ".///foo//bar/..";
  string_b = "foo/bar/..";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "..///foo//bar";
  string_b = "../foo/bar";
  EXPECT_TRUE(common::isSamePath(string_a, string_b));
  string_a = "foo/../bar/foo";
  string_b = "foo/bar/../foo";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "../foo/bar";
  string_b = "foo/bar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "foo/bar/..";
  string_b = "foo/bar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  // More difficult cases are producing wrong results.
  // TODO(mfehr): Adapt this if we ever fix these cases.
  string_a = "foo/../foo/bar";
  string_b = "foo/bar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "foo/bar/..";
  string_b = "foo";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "foo/bar/../bar";
  string_b = "foo/bar/foo/../../bar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));
  string_a = "foo/bar/../bar";
  string_b = "foo/foo/../bar";
  EXPECT_FALSE(common::isSamePath(string_a, string_b));

  // Invalid paths.
  string_a = "";
  string_b = "./";
  EXPECT_DEATH(common::isSamePath(string_a, string_b), "^");
}

TEST(MaplabCommon, createPathTest) {
  const std::string test_path = "maplab_test_data/createPathTest/A/B";
  EXPECT_FALSE(common::pathExists(test_path));
  EXPECT_TRUE(common::createPath(test_path));
  EXPECT_TRUE(common::pathExists(test_path));
}

TEST(MaplabCommon, createPathToFileTest) {
  const std::string test_path = "maplab_test_data/createPathToFileTest/A/";
  const std::string test_file = test_path + "B.txt";
  EXPECT_FALSE(common::pathExists(test_path));
  EXPECT_TRUE(common::createPathToFile(test_file));
  EXPECT_TRUE(common::pathExists(test_path));
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
