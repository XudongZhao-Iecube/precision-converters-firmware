# Precision Converters Firmware

[Analog Devices Inc.](http://www.analog.com/en/index.html) Precision Converters Firmware Applications

## About
This repository contains the embedded firmware applications for Analog Devices Precision Converters. The applications are targetted for primaraly Mbed and STM32 platforms, 
however they can be easily retargetted to other platforms as well. 
The firmware application are developed to interface with Precision Converters (ADCs/DACs) in order to configure/access device parameters and capture device data over serial 
communication link.

## Building for Mbed platform
The recommended way to build and debug the firmware using Mbed platform is by using the [Keil web based cloud IDE](https://studio.keil.arm.com/auth/login/) from ARM.
There is a seperate folder for each target application (such as ad7124, ad7606, etc), which contains the application specific files. In order to compile a 
specific target firmware application, the other application directories must be ignored through a ".mbedignore" file.

Below is an example to compile the project for ad7124 target application:

<img src="https://user-images.githubusercontent.com/62383520/146757820-d7e0f218-0c55-425d-abe1-5da4d90438cf.png" width="600">

## Adding mbed library dependancy:
The target firmware applications have dependancies on the the other ADI libraries. These libraries are imported by Mbed build system using .lib files, 
which points to a specific commit on the git repository. As these libraries are maintained seperately, the firmware application must point to a specific
version/commit ID of those. The detailed guide to update libraries is available in the "mbed_lib_dependnacy.txt" file from respective project.

Typical instructions to update the library dependnacy are given below:

Step 1: Create new ".lib" files into a project folder and add specified contents into them:
*Note: Make sure these files are not present in other project folder at a same time as this would duplicate the library content and increase your storage space.

```
File: mbed_platform_drivers.lib
Content: https://os.mbed.com/teams/AnalogDevices/code/platform_drivers/#your_commit_id
```

```
File: no-OS.lib
Content: https://github.com/analogdevicesinc/no-OS/#your_commit_id
```
<img src="https://user-images.githubusercontent.com/62383520/146760556-f222d81e-ef5f-46e8-8219-e4545e8fc862.png" width="600">

Step 2: Clean-build the project after adding library dependnacy

<img src="https://user-images.githubusercontent.com/62383520/146761476-85d5a8f5-2b75-426e-918e-f235ad460b1f.png" width="600">


## Building for STM32 platform
[Refer STM32_build.md for building the project for mbed platform](https://github.com/mphalke/precision-converters-firmware/blob/main/STM32_build.md)
