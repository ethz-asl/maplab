#include "maplab-common/file-system-tools.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <iostream>
#include <regex>  // NOLINT

#include <glog/logging.h>

namespace common {

std::string generateDateString(time_t* input_time) {
  time(input_time);
  tm* timeinfo = localtime(input_time);  // NOLINT

  char buffer[512];
  const size_t string_length =
      strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", timeinfo);
  CHECK_GT(string_length, 0u) << "Failed to create date time string!";

  std::string date_time_string(buffer);
  return date_time_string;
}

std::string generateDateStringFromCurrentTime() {
  time_t epoch_time = time(NULL);
  return generateDateString(&epoch_time);
}

// Gets the real path of a (relative) path. Can handle ".", ".." and links.
// Doesn't work properly if the path in relative_path doesn't exist.
std::string getRealPath(const std::string& relative_path) {
  CHECK(pathExists(relative_path) || fileExists(relative_path))
      << "Path must exist in order to call getRealPath()! path: "
      << relative_path;
  char resolved_path[PATH_MAX];
  char* result = realpath(relative_path.c_str(), resolved_path);
  if (result != nullptr) {
    CHECK_EQ(result, resolved_path);
  }
  return std::string(resolved_path);
}

// Source: https://stackoverflow.com/a/145309
std::string getCurrentWorkingDirectory() {
  char current_working_dir_array[PATH_MAX];

  if (!getcwd(current_working_dir_array, PATH_MAX)) {
    LOG(FATAL) << "Could not get current working directory! Error: " << errno
               << ": " << strerror(errno);
  }
  return std::string(current_working_dir_array);
}

void concatenateFolderAndFileName(
    const std::string& folder, const std::string& file_name,
    std::string* path) {
  CHECK_NOTNULL(path);
  CHECK(!file_name.empty());
  *path = folder;
  if (path->back() != '/') {
    *path += '/';
  }
  *path = *path + file_name;
}

std::string concatenateFolderAndFileName(
    const std::string& folder, const std::string& file_name) {
  std::string path;
  concatenateFolderAndFileName(folder, file_name, &path);
  return path;
}

bool deleteFile(const std::string& file_path) {
  if (remove(file_path.c_str()) != 0) {
    LOG(ERROR) << "Could not delete file: " << file_path;
    return false;
  }
  return true;
}

bool pathExists(const std::string& path) {
  struct stat file_status;
  if (stat(path.c_str(), &file_status) == 0 &&
      (file_status.st_mode & S_IFDIR)) {
    return true;
  }
  return false;
}

bool fileExists(const std::string& file) {
  struct stat file_status;
  if (stat(file.c_str(), &file_status) == 0 &&
      (file_status.st_mode & S_IFREG)) {
    return true;
  }
  return false;
}

int getAllFilesAndFoldersInFolder(
    const std::string& folder, std::vector<std::string>* file_paths,
    std::vector<std::string>* file_names,
    std::vector<std::string>* folder_paths) {
  DIR* directory;
  struct dirent* ent;
  if ((directory = opendir(folder.c_str())) != nullptr) {
    while ((ent = readdir(directory)) != nullptr) {
      const std::string file_or_folder_name = ent->d_name;

      // Discard current and parent folder reference.
      if (file_or_folder_name == "." || file_or_folder_name == "..") {
        continue;
      }

      // Get full path.
      std::string full_path;
      concatenateFolderAndFileName(folder, file_or_folder_name, &full_path);

      // Get file status.
      struct stat file_status;
      const int stat_result = stat(full_path.c_str(), &file_status);
      if (stat_result == 0) {
        if (file_status.st_mode & S_IFDIR) {
          // The path points to a directory.

          // Add the new subfolder.
          if (folder_paths != nullptr) {
            folder_paths->push_back(full_path);
          }

          // Recursive call.
          getAllFilesAndFoldersInFolder(
              full_path, file_paths, file_names, folder_paths);
        } else if (file_status.st_mode & S_IFREG) {
          // The path points to a file. Append if a vector has been provided.
          if (file_paths != nullptr) {
            file_paths->push_back(full_path);
          }
          if (file_names != nullptr) {
            file_names->push_back(file_or_folder_name);
          }
        } else {
          // The path leads to something else.
          continue;
        }
      } else {
        // Check if it's a dead link.
        struct stat file_status_lstat;
        const int lstat_result = lstat(full_path.c_str(), &file_status_lstat);
        if (lstat_result == 0 && file_status_lstat.st_mode & S_IFLNK) {
          VLOG(1) << "The link " << full_path << " does not seem to be valid.";
          continue;
        }
        closedir(directory);
        LOG(FATAL) << "Unable to retrieve file status for path: " << full_path;
      }
    }
    closedir(directory);
  } else {
    LOG(ERROR) << "Unable to open directory. Error: " << strerror(errno);
    return errno;
  }
  return 0;
}

int getAllFilesAndFoldersInFolder(
    const std::string& folder, std::vector<std::string>* file_paths,
    std::vector<std::string>* folder_paths) {
  return getAllFilesAndFoldersInFolder(
      folder, file_paths, nullptr, folder_paths);
}

int getAllFoldersInFolder(
    const std::string& folder, std::vector<std::string>* folder_paths) {
  return getAllFilesAndFoldersInFolder(folder, nullptr, nullptr, folder_paths);
}

int getAllFilesInFolder(
    const std::string& folder, std::vector<std::string>* file_paths) {
  return getAllFilesAndFoldersInFolder(folder, file_paths, nullptr, nullptr);
}

bool compareNumericPartsOfStrings(
    const std::string& s1, const std::string& s2) {
  std::string::const_iterator it1 = s1.begin(), it2 = s2.begin();
  std::string::const_iterator rit1 = s1.end(), rit2 = s2.end();
  // find beginning of number
  --rit1;
  --rit2;
  while (!std::isdigit(*rit1)) {
    --rit1;
  }
  it1 = rit1;
  while (std::isdigit(*it1)) {
    --it1;
  }
  ++it1;
  while (!std::isdigit(*rit2)) {
    --rit2;
  }
  it2 = rit2;
  while (std::isdigit(*it2)) {
    --it2;
  }
  ++it2;

  if (rit1 - it1 == rit2 - it2) {
    while (it1 != rit1 && it2 != rit2) {
      if (*it1 > *it2) {
        return true;
      } else if (*it1 < *it2) {
        return false;
      }
      ++it1;
      ++it2;
    }
    if (*it1 > *it2) {
      return true;
    } else {
      return false;
    }
  } else if (rit1 - it1 > rit2 - it2) {
    return true;
  }
  return false;
}

int getFileLists(
    const std::vector<std::string>& initial_paths, bool sort_lexical,
    const std::string& extension_filter, std::vector<std::string>* file_paths) {
  CHECK_NOTNULL(file_paths);

  std::string initial_path;
  DIR* d;
  struct dirent* dir;
  for (size_t directory_idx = 0; directory_idx < initial_paths.size();
       ++directory_idx) {
    initial_path = initial_paths.at(directory_idx);
    if (initial_path.back() != '/' && !initial_path.empty()) {
      initial_path += '/';
    }

    d = opendir(initial_path.c_str());
    if (d == NULL) {
      throw std::logic_error(initial_path + " results in d == NULL");
      return 1;
    }
    int i = 0;
    while ((dir = readdir(d))) {
      if (strcmp(dir->d_name, ".") == 0 || strcmp(dir->d_name, "..") == 0) {
        continue;
      }
      if (!std::regex_match(
              dir->d_name, std::regex("(.*)(" + extension_filter + ")"))) {
        std::cout << "Skipped file: " << dir->d_name << std::endl;
        continue;
      }
      if (dir == NULL)
        break;

      i++;
      file_paths->push_back(initial_path + dir->d_name);
    }
  }
  if (sort_lexical) {
    std::sort(file_paths->begin(), file_paths->end());  // normal lexical sort
  } else {
    std::sort(
        file_paths->begin(), file_paths->end(),
        std::bind(
            &common::compareNumericPartsOfStrings, std::placeholders::_2,
            std::placeholders::_1));  // sorts strictly by the number in the
                                      // file name
  }
  return 0;
}

bool removePath(const std::string& path) {
  CHECK(!path.empty()) << "The given path is empty.";
  if (pathExists(path)) {
    std::vector<std::string> all_files_in_folder;
    std::vector<std::string> all_folders_in_folder;
    getAllFilesAndFoldersInFolder(
        path, &all_files_in_folder, &all_folders_in_folder);
    for (const std::string& file : all_files_in_folder) {
      deleteFile(file);
    }

    for (std::vector<std::string>::reverse_iterator rit =
             all_folders_in_folder.rbegin();
         rit != all_folders_in_folder.rend(); ++rit) {
      if (remove(rit->c_str()) != 0) {
        LOG(ERROR) << "Unable to delete the given path: " << *rit;
        return false;
      }
    }

    if (remove(path.c_str()) != 0) {
      LOG(ERROR) << "Unable to delete the given path: " << path;
      return false;
    }
  }
  return true;
}

bool hasOnlyAsciiCharacters(const std::string& string_to_test) {
  constexpr char kUpperAsciiBound = '~';
  constexpr char kLowerAsciiBound = ' ';
  for (const char& character : string_to_test) {
    if (character > kUpperAsciiBound || character < kLowerAsciiBound) {
      return false;
    }
  }
  return true;
}

bool createPath(const std::string& path_to_create_input) {
  constexpr mode_t kMode = 0777;

  CHECK(!path_to_create_input.empty()) << "Cannot create empty path!";

  // Append slash if necessary to make sure that stepping through the folders
  // works.
  std::string path_to_create = path_to_create_input;
  if (path_to_create.back() != '/') {
    path_to_create += '/';
  }

  // Loop over the path and create one folder after another.
  size_t current_position = 0u;
  size_t previous_position = 0u;
  std::string current_directory;
  while ((current_position = path_to_create.find_first_of(
              '/', previous_position)) != std::string::npos) {
    current_directory = path_to_create.substr(0, current_position++);
    previous_position = current_position;

    if (current_directory == "." || current_directory.empty()) {
      continue;
    }

    if (!hasOnlyAsciiCharacters(current_directory)) {
      return false;
    }

    int make_dir_status = 0;
    if ((make_dir_status = mkdir(current_directory.c_str(), kMode)) &&
        errno != EEXIST) {
      VLOG(2) << "Unable to make path! Error: " << strerror(errno);
      return make_dir_status == 0;
    }
  }
  return true;
}

bool removeIfExistsAndCreatePath(const std::string& path) {
  CHECK(!path.empty()) << "The given path is empty.";
  if (!removePath(path)) {
    return false;
  }
  if (createPath(path)) {
    return false;
  }
  return true;
}

void tokenizeString(
    const std::string& input_string, const std::string& delimiters,
    std::vector<std::string>* tokens) {
  CHECK(!input_string.empty());
  CHECK(!delimiters.empty());
  CHECK_NOTNULL(tokens)->clear();

  std::string input_string_copy = input_string;
  char* input_c_str = strdup(input_string_copy.c_str());
  char* delimiters_c_str = strdup(delimiters.c_str());
  char* token = std::strtok(input_c_str, delimiters_c_str);
  while (token != NULL) {
    std::string string_token(token);
    tokens->push_back(string_token);
    token = std::strtok(NULL, delimiters_c_str);
  }
}

void simplifyPath(std::string* path) {
  std::string::size_type idx;
  for (idx = path->find("/./"); idx != std::string::npos;
       idx = path->find("/./")) {
    path->replace(idx, 3u, "/");
  }

  // Delete all the duplicate slashes.
  for (idx = path->find("//"); idx != std::string::npos;
       idx = path->find("//")) {
    path->replace(idx, 2u, "/");
  }

  // Delete trailing "/.".
  if (path->size() > 1u && path->substr(path->size() - 2u, 2u) == "/.") {
    path->erase(path->size() - 2u, 2u);
  }

  // Delete trailing slash.
  if (path->size() > 1u && path->back() == '/') {
    path->erase(path->size() - 1u, 1u);
  }

  // Delete leading "./".
  if (path->size() > 1u && strncmp(path->c_str(), "./", 2) == 0) {
    path->erase(0u, 2u);
  }
}

bool isSamePath(
    const std::string& path_a_input, const std::string& path_b_input) {
  CHECK(!path_a_input.empty());
  CHECK(!path_b_input.empty());

  std::string path_a = path_a_input;
  std::string path_b = path_b_input;

  simplifyPath(&path_a);
  simplifyPath(&path_b);

  // ".." is not supported unless it is used the same way in both paths.
  const bool dotdot_at_end = path_a.find("/..") == (path_a.size() - 3u) ||
                             path_b.find("/..") == (path_b.size() - 3u);
  const bool dotdot_at_start =
      path_a.find("../") == 0u || path_b.find("../") == 0u;
  const bool dotdot_in_between = path_a.find("/../") != std::string::npos ||
                                 path_b.find("/../") != std::string::npos;
  if (dotdot_in_between || dotdot_at_end || dotdot_at_start) {
    LOG(ERROR) << "Path comparison might produce wrong results for paths "
               << "containing '..' ! path_a: " << path_a
               << " path_b: " << path_b;
  }

  // Compare the strings, they should now be the same.
  return path_a == path_b;
}

void splitPathByLastOccurenceOf(
    const std::string& path, const std::string& delimiter,
    const bool left_first, std::string* left, std::string* right) {
  CHECK_NOTNULL(left);
  CHECK_NOTNULL(right);
  const size_t position_delimiter = path.find_last_of(delimiter);
  if (position_delimiter != std::string::npos) {
    *left = path.substr(0, position_delimiter);
    *right = path.substr(position_delimiter + 1);
  } else {
    if (!left_first) {
      left->clear();
      *right = path.substr(position_delimiter + 1);
    } else {
      right->clear();
      *left = path.substr(position_delimiter + 1);
    }
  }
}

void splitPathAndFilename(
    const std::string& path_with_file, std::string* path,
    std::string* filename) {
  CHECK_NOTNULL(path);
  CHECK_NOTNULL(filename);
  constexpr bool kLeftFirst = false;
  splitPathByLastOccurenceOf(path_with_file, "/\\", kLeftFirst, path, filename);
}

void splitFilePathAndExtension(
    const std::string& path_with_file_and_extension,
    std::string* path_with_file, std::string* extension) {
  CHECK_NOTNULL(path_with_file);
  CHECK_NOTNULL(extension);
  constexpr bool kLeftFirst = true;
  splitPathByLastOccurenceOf(
      path_with_file_and_extension, ".", kLeftFirst, path_with_file, extension);
}

bool createPathToFile(const std::string& path_to_file) {
  std::string path, filename;
  splitPathAndFilename(path_to_file, &path, &filename);
  if (path.empty()) {
    // Nothing to do.
    return true;
  }
  return createPath(path);
}

bool copyFile(
    const std::string& source, const std::string& destination, mode_t mode,
    bool overwrite) {
  CHECK(!source.empty());
  CHECK(!destination.empty());

  if (!fileExists(source)) {
    LOG(ERROR) << "The given source file " << source
               << " does not exist. Copying failed.";
    return false;
  }

  if (fileExists(destination) && (!overwrite)) {
    LOG(ERROR) << "The destination file already exists and should not be "
                  "overwritten. Aborting copy-operation.";
    return false;
  }

  std::string destination_path;
  std::string filename;
  splitPathAndFilename(destination, &destination_path, &filename);

  if (!pathExists(destination_path)) {
    LOG_IF(ERROR, !createPath(destination_path))
        << "Failed to create destination path " << destination_path;
  }

  const int source_filedescriptor = open(source.c_str(), O_RDONLY, 0);
  if (source_filedescriptor < 0) {
    LOG(ERROR) << "Unable to open source file " << source;
    return false;
  }

  const int destination_filedescriptor =
      open(destination.c_str(), O_WRONLY | O_CREAT, mode);
  if (destination_filedescriptor < 0) {
    LOG(ERROR) << "Unable to create destination file " << destination;
    close(source_filedescriptor);
    return false;
  }

  char buffer[BUFSIZ];
  ssize_t size;
  while ((size = read(source_filedescriptor, buffer, BUFSIZ)) > 0) {
    CHECK_EQ(write(destination_filedescriptor, buffer, size), size);
  }

  close(source_filedescriptor);
  close(destination_filedescriptor);

  VLOG(3) << "Successfully copied file " << source << " to " << destination
          << '.';
  return true;
}

bool isSameRealPath(
    const std::string& real_path_A, const std::string& real_path_B) {
  CHECK(!real_path_A.empty());
  CHECK(!real_path_B.empty());
  CHECK(pathExists(real_path_A)) << "Is not a real path: " << real_path_A;
  CHECK(pathExists(real_path_B)) << "Is not a real path: " << real_path_B;

  return isSamePath(getRealPath(real_path_A), getRealPath(real_path_B));
}

bool isSameRealFilePath(
    const std::string& real_file_A, const std::string& real_file_B) {
  CHECK(!real_file_A.empty());
  CHECK(!real_file_B.empty());

  CHECK(fileExists(real_file_A)) << "Is not a real file: " << real_file_A;
  CHECK(fileExists(real_file_B)) << "Is not a real file: " << real_file_B;

  return isSamePath(getRealPath(real_file_A), getRealPath(real_file_B));
}

}  // namespace common
