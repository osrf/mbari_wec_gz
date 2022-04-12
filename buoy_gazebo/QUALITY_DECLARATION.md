This document is a declaration of software quality for the `buoy_gazebo` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# buoy_gazebo Quality Declaration

The package `buoy_gazebo` claims to be in the **Quality Level 5** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-quality-categories) of the ROS2 developer guide.

## Version Policy [1]

### Version Scheme [1.i]

`buoy_gazebo` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`buoy_gazebo` is at an unstable version, i.e. `< 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

TODO

### API Stability Policy [1.iv]

TODO

### ABI Stability Policy [1.v]

TODO

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

TODO

## Change Control Process [2]

`buoy_gazebo` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process).

### Change Requests [2.i]

All changes will occur through a pull request, check [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Contributor Origin [2.ii]

TODO

### Peer Review Policy [2.iii]

All pull requests will be peer-reviewed, check [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Continuous Integration [2.iv]

TODO

###  Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

TODO

### Public API Documentation [3.ii]

TODO

### License [3.iii]

The license for `buoy_gazebo` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](./package.xml) manifest file, and a full copy of the license is in the [`LICENSE`](../LICENSE) file.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `buoy_gazebo`.

## Testing [4]

### Feature Testing [4.i]

TODO

### Public API Testing [4.ii]

TODO

### Coverage [4.iii]

TODO

### Performance [4.iv]

TODO

### Linters and Static Analysis [4.v]

`buoy_gazebo` uses and passes all the ROS2 standard linters and static analysis tools for a C++ package as described in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

## Dependencies [5]

Below are evaluations of each of `buoy_gazebo`'s run-time and build-time dependencies that have been determined to influence the quality.

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

It also has several test dependencies, which do not affect the resulting quality of the package, because they are only used to build and run the test code.

### Direct and Optional Runtime ROS Dependencies [5.i]/[5.ii]

TODO

### Direct Runtime non-ROS Dependency [5.iii]

TODO

## Platform Support [6]

TODO

## Security

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
