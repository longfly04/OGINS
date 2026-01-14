# Compile under Windows
The MSVC(Microsoft Visual C/C++) compiler is generally used in Windows. We recommend you to compile KF-GINS in the VSCode software.

You should first install the MSVC compiler and VScode software (including the necessary plug-ins, such as C/C++, C/C++ Extension Pack, CMake, and CMake Tools).

After preparing your own compilation environment, you can clone the repository locally and open the KF-GINS folder in VSCode:

Set compiler: open the Command Palette (Ctrl+Shift+P) and type "CMake: Select a Kit", select the MSVC compilier
Set compile parameter: type "CMake: Select Variant" in the Command Palette, select "Release"
Configure CMake: type "CMake: Configure" in the Command Palette
Compile Project: type "CMake: Build" in the Command Palette