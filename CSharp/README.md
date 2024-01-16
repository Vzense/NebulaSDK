
# CSharp Wrapper for NebulaSDK

## Overview
This CSharp package facilitates depth IR and RGB data acquisition and processing for NebulaSDK.

### Requirements

- .NET Framework : 4.6.x
- Visual Studio 2017

## Installation

- **Install the Vzense CSharp package**

  - [Install NebulaSDK](https://github.com/Vzense/NebulaSDK)
	  ```console
    git clone https://github.com/Vzense/NebulaSDK
    ```
	
- **Build depend libraries envrionment for CSharp**

    <b>x64</b> and <b>x86</b> are supportted by project, copy the corresponding files to the 'Bin/x64' or 'Bin/x86' is necessary. Take <b>x64</b> as an example:
  - Method1: 
    
    Copy all files in <b>NebulaSDK/Windows/Bin/x64</b> to <b>NebulaSDK/C#/Bin/x64</b> manually
  - Method2: 
  
    Run the <b>NebulaSDK/C#/install.py</b>
    ```console
    python install.py x64
    ```
	
## Details
- NebulaSDK_CSharp.dll is the CSharp dynamic library of NebulaSDK
