# AprilTag Class Documentation

## Overview

The `AprilTag` class is a MATLAB handle class designed to detect, localize, and visualize AprilTags using a calibrated camera.

It is compatible with the **AuRoRA platform** and supports:

- Camera connection and disconnection  
- Camera calibration loading  
- AprilTag detection  
- Pose estimation (Tag ↔ Camera ↔ World)    
- 3D CAD visualization  
- Coordinate transformation utilities  

Because it inherits from `handle`, objects of this class are passed by reference, meaning modifications affect the original object instance.

---
## Properties

### Camera-Related Properties

#### `pCam`
Stores the webcam object used to capture images.

#### `pCamParams`
Stores intrinsic calibration parameters of the camera (`cameraParams` object).  
These parameters are required for accurate pose estimation.

---
### Detection Flags

#### `pFlag`
Boolean flag indicating whether a tag was successfully detected.

```matlab
if tag.pFlag
    % Tag detected
end
```

---

### CAD Visualization

#### `pCAD`
Structure that stores CAD model data, graphic handles, and original vertices for 3D visualization.

---

### Tag Parameters

#### `pPar`
Structure containing tag parameters:
- `family` – AprilTag family (e.g., `tag36h11`)
- `size` – Physical tag size (meters)
- `ID` – Desired tag ID
- `H` – Homogeneous transformation matrix

---

### Pose Information

#### `pPos`
Structure that stores:
- `Xt2c` – Tag pose relative to the camera
- `Xt2w` – Tag pose relative to the world
---

## Constructor

```matlab
tag = AprilTag(ID)
```

### Behavior
- If no ID is provided, it defaults to `0`
- Initializes parameters and flags

---

## Public Methods

### Initialization

#### `iParameters(tag, varargin)`
Initializes tag parameters such as family, size, and internal structures.

#### `iFlags(tag)`
Initializes detection flags (typically sets `pFlag = false`).

---

### Camera Control

#### `rCameraConnect(tag, camera)`
Connects a webcam object to the class.

```matlab
cam = webcam(1);
tag.rCameraConnect(cam);
```

#### `rCameraDisconnect(tag)`
Disconnects and releases the camera resource.

---

### Calibration

#### `mLoadCalibration(tag, file)`
Loads camera calibration parameters from a `.mat` file.
If no file is provided, a file selection dialog is opened.

---

### Tag Detection

#### `mTagRead(tag)`
Main detection routine:
- Captures image
- Detects AprilTag
- Estimates pose
- Updates `pPos` and `pFlag`

---

### Visualization

#### `mTagPlot(tag, mode, I, Xcam_w)`
Plots tag pose and coordinate axes for visualization and debugging.

#### `mDrawAxes(tag, I)`
Draws coordinate axes directly onto the image.

---

### Localization

#### `mInverseLocalization(tag, Xtag_w)`
Computes camera pose from known tag pose using transformation inversion.

#### `mWorldLocalization(tag, Xcam_w)`
Computes tag pose in the world frame using camera pose information.

---

### CAD Visualization

#### `mCADload(tag, modelName)`
Loads a 3D CAD model.

#### `mCADmake(tag, mode)`
Creates the CAD visualization object.

#### `mTagPlot2(tag, mode, I, Xcam_w)`
Alternative plotting method with CAD integration.

---

## Private Methods

### `mTransformToPose(obj, X)`
Converts a homogeneous transformation matrix into a pose vector:
`[x, y, z, roll, pitch, yaw]`

### `mTransformToMatrix(obj, T)`
Converts a pose vector back into a 4×4 homogeneous transformation matrix.

---

## Mathematical Background

The class relies on homogeneous transformations:

```
T = [ R  t
      0  1 ]
```

Where:
- `R` is a rotation matrix
- `t` is a translation vector

These transformations are used to relate camera, tag, and world reference frames.

---

## Typical Workflow

```matlab
tag = AprilTag(5);
tag.iParameters();

cam = webcam(1);
tag.rCameraConnect(cam);

tag.mLoadCalibration('cameraParams.mat');
tag.mTagRead();

if tag.pFlag
    disp(tag.pPos.Xt2c)
end
```

---

## Applications

- Mobile robot localization  
- Drone navigation  
- World-frame mapping  
- Pose-based control  
- Augmented reality  

---

## Conclusion

The `AprilTag` class provides a complete and modular solution for vision-based localization using AprilTags.  
Its structure makes it suitable for robotics, control, and experimental research applications.

---

## Exercise – Testing the `AprilTag` Class and Pose Visualization

### Objective

The objective of this exercise is to allow the user to test the basic functionality of the `AprilTag` class, verifying that tag detection, pose estimation, and access to the `pPos` variable are working correctly.

By the end of this exercise, the user should be able to:
- Detect an AprilTag in real time using a camera
- Verify the detection status using the `pFlag` variable
- Access the tag pose relative to the camera (`pPos.Xt2c`)
- Draw the tag coordinate axes over the camera image

---

### Description

Using the `AprilTag` class, develop a MATLAB script that:

1. Creates an object of the `AprilTag` class, defining the size, family and id of the tag to be detected  
2. Connects the camera to the object  
3. Loads the camera calibration parameters  
4. Performs AprilTag detection in real time  
5. Checks whether the tag was detected using the `pFlag` variable  
6. If the detection is valid:
   - Accesses the homogeneous pose matrix of the tag relative to the camera (`pPos.Xt2c`)
   - Displays this matrix in the *Command Window*
   - Draws the tag coordinate axes on the camera image  

The program should run in a continuous loop and terminate when the user presses a key.

---

### Inputs

- Family, size and id of the AprilTag to be detected  
- Camera connected to the computer  
- Camera calibration file (`cameraParams.mat`)  

---

### Expected Outputs

- Real-time camera image display  
- Tag coordinate axes overlaid on the image when the tag is detected  
- Position x,y,z and the angles throught `pPos.Xt2c` displayed in the *Command Window*  

