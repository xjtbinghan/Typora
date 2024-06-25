# 1.为什么要使用Cmake

1.cmake可以统一执行多个文件的编译，比起g++方便很多。

2.对于多个语言的项目，cmake可以统一管理并提供支持和接口。

# 2.Cmake如何使用

对于一个项目来说，可以采用内部和外部构建cmake的方式：

**内部构建是：**在项目所在目录下构建CMakeLists.txt ，然后直接在该目录下使用（cmake .）  命令。这时，会在该目录下产生一些临时文件，然后使用（make）命令执行Makefile文件，这时会产生可执行文件，可执行文件的名字就是CMakeList.txt中add_executable命令后面的参数指定的项目名。**内部构建的坏处是**会把临时的文件全部都保存在项目文件内，看着很乱，不方便管理。所以我们一般推荐外部构建。

**外部构建是：**在项目所在目录下构建CMakeLists.txt ，然后在项目目录下创建一个build文件夹，cd进入到build之后，再使用（cmake .. ）命令，这时会把临时文件都生成在build文件夹中，然后在该文件夹中使用（make）命令就生成了可执行文件。运行可执行文件即可。**外部构建的好处**是，可以直接把临时的文件保存在build目录下，方便管理。

```bash
#使用流程

创建并编写CMakeLists.txt文件
#对于外部构建，先mkdir build 并cd到build目录中
cmake  ${CMakeLists.txt文件所在目录}  #生成Makefile等临时文件
make  #生成可执行文件
./www #执行文件 
```

**其中的路径问题：**cmake命令后面的路径指向CMakeLists.txt 文件路径。 make命令所在终端的路径为Makefile文件路径。

# 3.CMakeLists.txt文件内容详解

讲解

```cmake
# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)

# 项目的名称
project(MyProject)

# 指定C++标准
set(CMAKE_CXX_STANDARD 11)

# 设置可执行文件的名称和源文件
add_executable(myapp main.cpp)

# 指定包含目录
include_directories(include)

# 添加子目录，用于构建子项目或模块
add_subdirectory(subdir)

# 链接外部库
target_link_libraries(myapp external_library)

```

一般来说有这三个就行

```cmake
#该命令指定cmake的最低版本 可选非必须但是不加会有警告
cmake_minimum_required(VERSION 2.8) 

#指定工程的名字、工程描述、所需的语言等等，最简单的就是只写工程名称，省略其他的，这样会默认支持所有语言。
project(www  )  

#生成可执行程序
add_executable(erjinzhi_exe              #生成可执行程序的项目名
                ./10_xiti.cpp   #下面是原文件路径和名称，元文件之间可以通过空格也可以通过分号;进行间隔。
                ./use_xiti.cpp)
```

该文件必须的命令就只有如上三个，在创建好后CMakeLists.txt文件后就可以使用cmake命令了 。此时包含Makefile的临时文件生成在执行cmake命令时所在的目录。因此外部构建要在build目录下执行cmake。

# 4.VsCode + Cmake

​		需要使用Vscode进行调试，需要配置Vscode的launch和tasks两个json文件，把cmake写进tasks中，这样我们每一次启动F5运行和调试，就能够自动把我们修改后的代码编译了并且进入调试环节了。

**核心内容如下图**

![image-20231017154912320](/home/hanbing/公共的/typora实验记录/ORB_SLAM2/assets/image-20231017154912320.png)

### tasks.json文件

下面的代码就等同于，在调试之前运行cmake编译。

```bash
cd build
cmake ..
make 
```



```json
{
        "version": "2.0.0",
        "options":{
            "cwd":"${workspaceFolder}/build"  
        },
        "tasks": [
            {
                "type": "shell",              
                "label":"cmake",              
                "command":"cmake",            
                "args":[".."]                 
            },
            {
                "label":"make",
                "group": {                    
                    "kind": "build",
                    "isDefault": true
                },
                "command":"make",
                "args":[]
            },
            {
                "label":"Build",
                "dependsOrder": "sequence", 
                "dependsOn":[               
                    "cmake",
                    "make"
                ]
            }
        ]
}
```

### launch.json文件

需要注意的是：

program：表示每次运行和调试是运行哪个可执行文件；

args：表示该执行文件需要的参数；

cwd：表示当前的工作路径；

**PreLaunchTask：**表示在launch前，需要执行的tasks文件中的哪个模块，而要想在launch之前编译，就把编译的操作在tasks.json中写进Build这个label中就可以了。

```json
 {
    "version": "0.2.0",
    "configurations": [

        {
            "name": "g++ - 生成和调试活动文件",
            "type": "cppdbg",
            "request": "launch",
            "program": "/home/hanbing/公共的/paper_code/ORB_SLAM2_detailed_comments/Examples/RGB-D/rgbd_tum",
            "args": ["Vocabulary/ORBvoc.txt","Examples/RGB-D/TUM1.yaml","rgbd_dataset_freiburg1_desk","Examples/RGB-D/associations/fr1_desk.txt"],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "为 gdb 启用整齐打印",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ],
            "preLaunchTask": "Build",
            "miDebuggerPath": "/usr/bin/gdb"
        }
    ]
}
```

