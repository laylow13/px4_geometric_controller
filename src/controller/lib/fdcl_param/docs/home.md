# FDCL Param Class 

This library provides a C++ tool to read parameters from a text file, or save them to a text file.

<a name="contents"></a>
## Contents
1. [Basic Introduction](#intro)
2. [Using the Class](#using-the-class)
3. [Contributing](#contributing)

<a name="intro"></a>
## Basic Introduction

### Configuration File

The format of the compatible text file is illustrated by the following sample file saved as `fdcl.cfg`

```
UAV:
 m: 2.5
 J: 3, 0, 0,  0, 2, 0,  0, 0, 1
Control:
 kR: 1.8
 kW: 2.0
IMU:
 dev: "/dev/tty"
I2C:
 dev: "/dev/i2c"
 addr: 50
GPS:
 available: 1
```

The name of the parameter is composed of the group name, and the variable name. For example, in the above file, `UAV` is the group name and `m` is the variable name.

More precisely, if the new line begins with letters without any space, the letters before `:` are considered as the **group**. 
Or, if the new line begins with `space` (`tab` works too), the the letters between `space` and `:` are considered as the **variable**. 

The parameter name is constructed by concatenating **group** and **variable**. 
Note that the **variable** names can be repeated if they do not belong to the same **group**.

The followig types are suppored for the parameter value:
1. bool (0: false, 1: true)
2. double
3. integer
4. string (nested in " ")
5. Eigen double matrix of an arbitrary size

Note the string parameter should be nested in ```" "```. 
For an eigen matrix, each row is concetanted in a single line, where each element is separated by `,  ` For example, in the above configuration file, the 3x3 matrix J is defined as

```
J = [ 3 2 1; 
      0 2 0;
      0 0 1];
```
in the Matlab notation.

### Usage

The usuage of the library is illustrated by the following sample code.

```
#include <iostream>
#include <string>
#include "Eigen/Dense"

#include "fdcl/param.hpp"

using namespace std;

int main(void)
{
    // define variables to store the variables read from
    // the config file
    double m;
    string dev;
    int addr;
    Eigen::Matrix<double, 3, 3> J;
    bool GPS;

    // initialize the FDCL param class	
    fdcl::param cfg;

    // open the config file
    cfg.open("fdcl.cfg");

    // read variables
    // Note: the **group** name and the **variable** name are 
    // joined with `.` in between. 
    cfg.read("UAV.m",m);
    cout << "m=" << m << endl;

    cfg.read("UAV.J",J);
    cout << "J=" << J << endl;
    
    cfg.read("IMU.dev",dev);
    cout << "dev=" << dev << endl;
    
    cfg.read("GPS.available",GPS);
    cout << "GPS=" << GPS << endl;

    // update variables already in the config file
    cfg.save("I2C.addr",50);

    cfg.read("I2C.addr",addr);
    cout << "addr=" << addr << endl;
    
    cfg.save("GPS.available",true);
    cfg.read("GPS.available",GPS);
    cout << "GPS=" << GPS << endl;

    // close the config file
    cfg.close();
}
```

### Using Eigen Matrices

The package supports Eigen matrices for save and read functions, which are declared as template functions. 
Therefore, those functions must be explicitly instantiated according to the particular type of the Eigen matrices used. 

For example, the above example uses the Eigen marix type `Eigen::Matrix<double, 3, 3>`, and at the end of `fdcl_param.cpp` the following explicit istantiattion are included:

```
template void fdcl_param::read(const string param_name, Eigen::MatrixBase< Eigen::Matrix <double,3,3> >& M);
template void fdcl_param::save(const string param_name, Eigen::MatrixBase< Eigen::Matrix <double,3,3> >& M);

```

When using other types or sizes of Eigen matrices, the corresponding instantiations must be included at the end of `param.cpp`


### What it does NOT do
* Save function does NOT add a new entry to the configuration text file. When the save function is called, the parameter must alred exist in the text file.
* No comment support in the configuration text file.

[back to content](#contents)

<a name="using-the-class"></a>
## Using the Class

To add this to class to your code, using CMake and cloning it as a git submodule is recommended. 
This instructions assumes that you are going to add your submodules to a directory named `libraries` in the main project directory.
If your submodule directory is different, make sure to change the path wherever it says `libraries`.
First, add this as a submodule in git.

```
cd 'main/project/dir'
git submodule add https://github.com/fdcl-gwu/fdcl_param.git ./libraries/fdcl_param
git submodule update --init --recursive
```

NOTE: Whenever you clone your main project, you must recursively update the submodules:
```
git submodule update --init --recursive
```

Now, in the main project's CMake file (CMakeLists.txt), do the followings:
```
include_directories(${PROJECT_SOURCE_DIR}/libraries/fdcl_param/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/libraries/fdcl_param fdcl_param)
```

Also, whenever you make a file that uses fdcl_param class, add `fdcl_param` to the linker:
```
target_link_libraries(name/of/the/library/or/exec
    PRIVATE fdcl_param
)
```

Then, you can simply call `#include "fdcl/param.hpp"` in your source/header files in the main directory.

[back to content](#contents)

<a name="contributing"></a>
## Contributing
* Anyone is welcome to contribute, but make sure you follow the existing coding style.
* Make sure to document all your changes/additions with Doxygen style comments.

### Generating the Documentation
Document generation is done with Doxygen
If you do not have Doxygen, install it first
```
sudo apt-get install -y doxygen graphviz
```

Use Doxygen to generate the documentation
```
cd docs/Doxygen
doxygen Doxygen
```

This will generate the documentation. 
Commit and push to update the online documentation.
[back to contents](#contents)

[back to content](#contents)
