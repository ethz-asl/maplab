## Known Issues

Please also have a look at the [FAQ](D_FAQ.html), there might be a solution to your issue there. We encourage you to contribute to this repository if you have a solution to these issues

### Building with unstable internet connection

Building maplab will download and check-out dependencies, which can be very slow depending on the speed of you internet connection. A more detailed answer can be found in the [FAQ](D_FAQ.html#installation).

### Protobuf errors when changing protoc files

Sometimes when changing the `.protoc` files the program will segfault when loading a map for example. This is cause by an incompatibility with ccache. Clear the work space `catkin clean --yes` and also ccache `ccache -C -z`.
