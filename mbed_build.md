# Mbed Project Build Guide

## Selecting Project Using '.mbedignore' File
The recommended way to build and debug the firmware using Mbed platform is by using the [Keil web based cloud IDE](https://studio.keil.arm.com/auth/login/) from ARM.
Precision Converters Firmware repository contains a seperate folder for each project/application (such as ad7124_iio-application, ad7606_iio-application, etc), 
which contains an application specific files. In order to compile the specific project, the other project directories must be ignored through an "[.mbedignore](https://github.com/mphalke/precision-converters-firmware/blob/main/.mbedignore)" file.

Below is an "[.mbedignore](https://github.com/mphalke/precision-converters-firmware/blob/main/.mbedignore)" file example to compile the project for ad7124_iio-application project/application:

<img src="https://user-images.githubusercontent.com/62383520/146757820-d7e0f218-0c55-425d-abe1-5da4d90438cf.png" width="600">

## Importing Repository into Keil IDE
Select "File->Import Project.." option

<img src="https://user-images.githubusercontent.com/62383520/147880764-37319265-c962-440d-b59f-19fc134a71a6.png" width="400">

## Adding Library Dependancy
The firmware projects/applications have dependancies on the the other ADI libraries. These libraries are imported by Mbed build system using .lib files, 
which points to a specific commit ID on the github repository. As these libraries are maintained seperately, the firmware application must point to a specific
version/commit ID of those by refering through '[library_dependnacy.md](https://github.com/mphalke/precision-converters-firmware/blob/main/library_dependancy.md)' file.

*Note: The library names and content information is available in the "[library_dependnacy.md](https://github.com/mphalke/precision-converters-firmware/blob/main/library_dependancy.md)" file from respective project folders.*

Typical instructions to add the library dependancy for Mbed platform are given below:

Step 1: Create new ".lib" files into a project folder and add specified contents into them. To create a new file, right click on project folder and select
'New File' option

*Note: Make sure these files are not present in other project folder at a same time as this would duplicate the library content and increase your storage space.*

```
File: mbed_platform_drivers.lib
Content: https://os.mbed.com/teams/AnalogDevices/code/platform_drivers/#your_commit_id
```

<img src="https://user-images.githubusercontent.com/62383520/146760556-f222d81e-ef5f-46e8-8219-e4545e8fc862.png" width="1200">

Step 2:  Fix library issues by clicking on exclamatory mark option in Mbed Libraries tab at the bottom explorer bar

<img src="https://user-images.githubusercontent.com/62383520/147882990-48fa0e20-45f5-4e82-9f19-9d35094e3605.png" width="600">

Step 2: Clean-build the project after adding library dependancy

<img src="https://user-images.githubusercontent.com/62383520/147882857-c35d5099-4073-4cde-9902-c0abeecbdd1f.png" width="400">

Successful build should generate the binary file which can be copied (or dragged and dopped) into SDP-K1 (or selected target device) USB hosted drive.
Reference: https://os.mbed.com/platforms/SDP_K1/
