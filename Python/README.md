## Python Wrapper for NebulaSDK API

Python wrapper is an opensource project of Vzense Nebula API.

The goal of this project is to help developers use Vzense TOF camera via python method easily.

- PythonSDK version: V1.0.7

### Supported Devices

- DS77Lite 
- DS77CLite
- DS77Pro  
- DS77CPro  

### Requirements

- python version : 3.7.x
- python modules : ctypes, numpy, opencv-python(display only)

### Directory

- **DS77**: the API and Sample code for DS77Lite/DS77Pro
- **DS77C**: the API and Sample code for DS77CLite/DS77CPro

|system|details|
|---|---|
|Windows64|windows 64 bit|
|Windows32|windows 32 bit|
|Ubuntu20.04|the same with Ubuntu18.04 PC SDK|
|Ubuntu18.04|for PC with x86_64-linux-gnu(v7.5.0)|
|AArch64|for aarch64 with aarch64-linux-gnu(v7.5.0)|

### Quick Start

- step1. install modules:
         
```	 
	  pip install numpy
	  pip install opencv-python 
```
- step2. Switch to Samples under the product directory, run the sample that you need. 
    	 
         For example, go to the DS77/Samples/FrameViewer, then run 'python FrameViewer_DS77.py'