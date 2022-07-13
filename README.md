# 3D Industrial Inspection System

## Compiler & development tools

Qt 5.12.12 with MSVC 2019-64bit

you can download the corresponding tools from here as followings:

- qt-unified-windows-x64-online.exe from [here](https://download.qt.io/official_releases/online_installers/).
- qt-vsaddin-msvc2022-2.8.0.vsix from [here](https://download.qt.io/development_releases/vsaddin/2.8.0/).
- Visual Studio 2022 Community from [here](https://visualstudio.microsoft.com/vs/)

## Before compiler

The following instruction only works on Windows, particularly Win10+ platform.

1. Add more global environment variables.

- **MS3D** : "xxxxx\Measurement_System_in_3D_Requirements"
- **OSG** : xxx\Measurement_System_in_3D_Requirements\OSG 3.6.4
- Add "**%OSG%\bin**" into PATH variable.

2. Switch the **solution configuration** into **release** mode instead of Debug which is selected by default in Visual Studio 2022.

3. Make sure that **C++ 17** is selected because the \<filesystem\> will be used in this project.

## Applications in this project

1. Labeling tool

It was implemented in "*main_labeling_points.cpp*", and you can uncomment `#define LABELING_PONITS` at the first line of code to use it.

2. All-in-one process

The procedure: **registration -> searching -> measurement -> evaluation** can be execuated with one program which is in "*LocalTest.cpp*" under project "*CloudProcess*".

Run it and check the result with the labeling tool mentioned above.
