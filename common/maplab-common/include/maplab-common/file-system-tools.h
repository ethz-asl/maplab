#ifndef MAPLAB_COMMON_FILE_SYSTEM_TOOLS_H_
#define MAPLAB_COMMON_FILE_SYSTEM_TOOLS_H_

#include <ctime>
#include <string>
#include <vector>
#include <sys/stat.h>

namespace common {

// Remove the given path and all it's contents. Returns false on failure
// (unable to delete files or folders in the given path). Returns true on
// success or if path does not exist.
bool removePath(const std::string& path);

bool createPath(const std::string& path_to_create_input);

// Make the given path. Remove it first if it exists. Returns false if
// removing fails. Otherwise, returns true if createPath() returns no error,
// false otherwise.
bool removeIfExistsAndCreatePath(const std::string& path);

bool pathExists(const std::string& path);

bool deleteFile(const std::string& file_path);

bool fileExists(const std::string& file);

bool createPathToFile(const std::string& path_to_file);

std::string getCurrentWorkingDirectory();

// Copies file source to destination with the given mode.
// If the destination file already exists and overwrite is false,
// nothing is done and false is returned.
// If the file is successfully copied, true is returned, and false otherwise.
// Source:
// http://stackoverflow.com/questions/10195343/copy-a-file-in-a-sane-safe-and-efficient-way
bool copyFile(
    const std::string& source, const std::string& destination, mode_t mode,
    bool overwrite);

// Traverse folder and subfolder and list all files and directories.
// Sources:
// https://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c
// https://stackoverflow.com/questions/146924/how-can-i-tell-if-a-given-path-is-a-directory-or-a-file-c-c
int getAllFilesAndFoldersInFolder(
    const std::string& folder, std::vector<std::string>* file_paths,
    std::vector<std::string>* file_names,
    std::vector<std::string>* folder_paths);

int getAllFilesAndFoldersInFolder(
    const std::string& folder, std::vector<std::string>* file_paths,
    std::vector<std::string>* folder_paths);

int getAllFoldersInFolder(
    const std::string& folder, std::vector<std::string>* folder_paths);

int getAllFilesInFolder(
    const std::string& folder, std::vector<std::string>* file_paths);

// Imported from ethzasl_brisk - image-io.h
bool compareNumericPartsOfStrings(const std::string& s1, const std::string& s2);
// Imported from ethzasl_brisk - image-io.h
int getFileLists(
    const std::vector<std::string>& initial_paths, bool sort_lexical,
    const std::string& extension_filter, std::vector<std::string>* file_paths);

// Generate a date-time string from a given input time.
// Source:
// https://stackoverflow.com/questions/16357999/current-date-and-time-as-string
std::string generateDateString(time_t* input_time);

std::string generateDateStringFromCurrentTime();

// Gets the real path of a (relative) path. Can handle ".", ".." and links.
// Doesn't work properly if the path in rel_path doesn't exist.
std::string getRealPath(const std::string& rel_path);

void concatenateFolderAndFileName(
    const std::string& folder, const std::string& file_name, std::string* path);

std::string concatenateFolderAndFileName(
    const std::string& folder, const std::string& file_name);

// Split the string into tokens based on the delimiters specified.
// E.g. setting the delimiters to ',-. ' will split the string
// "Bla,foo-bar. O K" into [Bla, foo, bar, O, K].
// Empty delimiters and input strings are not allowed.
void tokenizeString(
    const std::string& input_string, const std::string& delimiters,
    std::vector<std::string>* tokens);

// Removes duplicate slashes and unnecessary periods.
// IMPORTANT: Cannot handle links, and "..".
void simplifyPath(std::string* path);

// Checks if two paths are the same, can handle leading/trailing and
// duplicate/unnecessary slashes and periods.
// IMPORTANT: This function is purely string based and does not involve any
// checks on the file system. Therefore it cannot handle links, absolute vs
// relative paths and "..".
bool isSamePath(
    const std::string& path_a_input, const std::string& path_b_input);

// Split string by last occurence of delimiter. If there is no delimiter left
// is contains to the complete string, unless left_first = false, then right
// will contain the complete string.
void splitPathByLastOccurenceOf(
    const std::string& path, const std::string& delimiter,
    const bool left_first, std::string* left, std::string* right);

void splitPathAndFilename(
    const std::string& path_with_file, std::string* path,
    std::string* filename);

void splitFilePathAndExtension(
    const std::string& path_with_file_and_extension,
    std::string* path_with_file, std::string* extension);

// Compares two REAL folder paths. If they don't exist on the file system or are
// empty this function will check-fail.
// realpath manual: http://man7.org/linux/man-pages/man3/realpath.3.html
bool isSameRealPath(
    const std::string& real_path_A, const std::string& real_path_B);

// Compares two REAL file paths. If they don't exist on the file system or are
// empty this function will check-fail.
bool isSameRealFilePath(
    const std::string& real_file_A, const std::string& real_file_B);

}  // namespace common

#endif  // MAPLAB_COMMON_FILE_SYSTEM_TOOLS_H_
