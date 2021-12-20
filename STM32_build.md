# STM32 Project Creation and Building Guide

## Checking out the repository
Run below command to checkout the git repository and all dependancy:

```
git clone --recursive https://github.com/mphalke/precision-converters-firmware
cd precision-converters-firmware
```

## Updating library dependnacy
Update the commit ID of ADI dependent libraries (whichever applicables) for respective target firmware application

```
cd libs/no-OS
git checkout your_commit_id
cd libs/libtinyiiod
git checkout your_commit_id
```

Refer the 'library_dependancy.txt' file from respective target application folder to know the specific commit ID for each library


## Create a new STM32 project from existing .ioc file:

  **`Step 1: Open STM32CubeIDE and select File->New->STM32 Project From Existing STM32CubeMX Configuration File (.ioc)`**
  
  <img src="https://user-images.githubusercontent.com/62383520/146762834-535c327e-03e2-4d07-9881-a035e2dd2141.png" width="600">
  
  ----------------------------------------------------------------------------------------
  
  **`Step 2: Select the .ioc file and project location`**
  
  <img src="https://user-images.githubusercontent.com/62383520/146764489-e1c6da05-5b00-4a51-9cc4-151b27a31b85.png" width="600">
  
   ----------------------------------------------------------------------------------------
   
   **`Step 3: Select the firmware package and version`**
   
  <img src="https://user-images.githubusercontent.com/62383520/146764312-2d1a8b35-96ff-4942-8b8a-ee6c4e428544.png" width="400">
  
   ----------------------------------------------------------------------------------------


## Adding linked resources into build system:
The target application source files are externally linked to a STM32 project using .extSettings file. By default these files are not added into a build
system for compilation. In order to build these files, perform below setting:

1. Right click on project and select properties
2. Expand the C/C++ General and select Paths and Symbols option

  <img src="https://user-images.githubusercontent.com/62383520/146765826-8fd9591e-fab9-4c65-9582-baedeae7800e.png" width="400">
  
3. Add the externally linked folder into build system 

  <img src="https://user-images.githubusercontent.com/62383520/146766695-e0486b00-1d98-4d57-9de4-f466d055f721.png" width="400">

4. Apply the settings and close the tab



