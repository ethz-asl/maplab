## Console Plugin System

### Installation
To get maplab into a state with all core plugins (VIMap file system integration, BA, LC, etc.), run
```
catkin build maplab
```

The package `maplab` is a metapackage containing the maplab console and all non-experimental plugins to ease the installation of a functioning console. You can type `help` in the console to get a list of all installed plugins.

If a plugin doesn't show up in a console, you may need to recompile it with rerunning cmake (as this will create the entry in the plugin file list):
```
catkin build --no-deps --force-cmake <missing_plugin_name>
```

### How it works
Each plugin adds an entry with the path to the library `*.so` to `$CATKIN_WS/devel/share/console-plugins.txt` ("plugin list") during compilation. This list is then opened by the maplab console application. The application tries to dynamically load the library referred in the `console-plugins.txt` file.

### Uninstalling a plugin
Delete the library `.so`: Delete the file under `$CATKIN_WS/devel/lib/lib<PACKAGE_NAME>.so`. Maplab will still try to load the library, but detect that the file is missing and delete the entry from the list.

Note: to readd the plugin to the console, it's not sufficient to only recompile the package. You also need to make sure that cmake is rerun by running catkin with the `--force-cmake` argument.

### Adding a new plugin
See [[Coding-Examples:-Creating-a-custom-console-plugin]]
