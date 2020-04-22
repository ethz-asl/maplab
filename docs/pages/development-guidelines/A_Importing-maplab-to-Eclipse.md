## Importing maplab to Eclipse

**Adapted from wiki page written by Simon Lynen**

### Install LLVM plugin:
* Help -> "Install New Software".
* From the "Work With" dropdown list select "All available Sites".
* Un-Select "Hide Items that are already installed".
* Enter "LLVM" into the search bar, press "Enter" and wait.
* Make sure the entry "C/C++ LLVM-Familiy Compiler Build Support" is checked.
* Click the "Next" button etc. to finish the installation.

### Configure LLVM plugin:
* Click on "Window" -> "Preferences"
* Under "C/C++" select "LLVM"
* Set the llvm installation directory:

  `/opt/llvm-3.7.0-iwyu/bin`

* Press the "New" button next to "Include directories" and add:

  `/opt/llvm-3.7.0-iwyu/include`

* Press the "New" button next to "Library search path directories" and add:

  `/opt/llvm-3.7.0-iwyu/lib`

### Create project for maplab mapping
Select "File" -> "New" -> "C++-Project"
* Set a project name
* Set "[PATH_TO_CATKIN_WS]/src/maplab" as project location.
* Select "Makefile Project" -> "Empty Project"
* Select "LLVM with Clang (Linux)" from the box on the right.
Click "Finish"

### Set additional indexer settings
* Select "File" -> "Properties"
* Expand "C++-General" and select "Paths and Symbols"
* In the "Includes" tab select "GNU C++" under "Languages" click "Add" and add:

   ```
   /opt/llvm-3.7.0-iwyu/include
   /usr/include/c++/4.8
   /usr/include/x86_64-linux-gnu/c++/4.8
   [PATH_TO_CATKIN_WS]/devel/include
   ```

* In the "Libraries" tab make sure you have an entry "stdc++"
* In the "Library Paths" tab click "Add" and add:

    ```
    /opt/llvm-3.7.0-iwyu/lib
    /usr/lib/gcc/x86_64-linux-gnu/4.8
    ```

* Open in project settings "C/C++ General", "Indexer", and click on "Enable project specific settings" and "Allow heuristic resolution of includes".
* Close the properties window by hitting "OK".
* If using gcc 4.8, Select Window / Preference
    * Under C/C++ / Build / Settings / Discovery / CDT GCC Built-In Compiler Settings, add "-std=c++11" to the command line, i.e. it should say "${COMMAND} ${FLAGS} -E -P -v -dD "${INPUTS}" -std=c++11".

* Rebuild the index
  * Right click on your project
  * Select "Index" -> "Rebuild" (Note: Until the background process is done some symbols will not resolve).

### Configure build targets
*For every package you want to build in Eclipse:*
* Select "File" -> "Properties"
* Click on "C++ Build" in the menu on the left.
* Click the "Manage Configurations"
* Click "New" and enter the package name as "Name" for the configuration.
* Back at the "C++ Build" page enter ```make -j8``` (or ```make tests -j8``` for test building) as build command.
* Navigate to `[PATH_TO_CATKIN_WS]/build/[PACKAGE_NAME]` as the build location.

The individual packages appear now as targets to build in the menu denoted by the "hammer" symbol.

