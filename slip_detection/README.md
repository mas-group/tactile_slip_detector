# Description
This package detects a slippage of a grasped object using tactile sensor information.
It contains the following components (nodes):

1. **Intensity extractor:**
Extracts tactile features (e.g. mean) for each tactile sensor. The purpose of this component
is to reduce the dimensionality of the data (from a tactile matrix) to a scalar value.
2. **Tactile flow extractor:**
Extracts the tactile flow of a (set of) tactile image(s) based on an optical flow algorithm
using 2d-convolution.
3. **Slip detector:**    
Combines the output of the previous two components to detect if slippage of a grasped object has occured.

**Note:** The following nodes require that a valid tactile adapter is running.
The examples below assume a tactile adapter for the SDH-2 tactile sensors.

## Intensity (feature) extractor
This component extracts a feature (e.g. mean, sum) of intensity for each
tactile sensor. A different property can be configured through a launch file
by changing the parameter 'tactile_data_type'.

### Usage
1. Launch the component:    
```roslaunch slip_detection mean_intensities.launch```
2. Verify that tactile data is available [example]:    
```rostopic echo /dsa_controller/tactile_data```
3. Subscribe to the result of the component:    
```rostopic echo /slip_detection/intensity_extractor/intensity_data```
4. Start the component:    
```rostopic pub /slip_detection/intensity_extractor/event_in std_msgs/String 'e_start'```

### To stop the component
```rostopic pub /slip_detection/intensity_extractor/event_in std_msgs/String 'e_stop'```

## Tactile flow (feature) extractor
This component extracts the amount of flow a contact has in a series of
tactile sensors (tactile images).

### Usage
1. Launch the component:    
```roslaunch slip_detection tactile_flow_extractor.launch```
2. Verify that tactile data is available [example]:    
```rostopic echo /dsa_controller/tactile_data```
3. Subscribe to the result of the component:    
```rostopic echo /slip_detection/tactile_flow_extractor/tactile_flow```
4. Start the component:    
```rostopic pub /slip_detection/tactile_flow_extractor/event_in std_msgs/String 'e_start'```

### To stop the component
```rostopic pub /slip_detection/tactile_flow_extractor/event_in std_msgs/String 'e_stop'```

## Slip detector
This component outputs the amount of flow a contact has in a series of tactile
sensors (tactile images). The launch file assumes a SDH-2 hand with tactile sensors.

### Usage
1. Launch the component:    
```roslaunch slip_detection slip_detector.launch```
2. Verify that tactile data is available [example]:    
```rostopic echo /dsa_controller/tactile_data```
3. Subscribe to the result(s) of the component:    
```rostopic echo /slip_detection/slip_detector/slip```    
```rostopic echo /slip_detection/slip_detector/event_out```    
4. Start the components:    
```rostopic pub /slip_detection/sdh_tactile_adapter/event_in std_msgs/String "e_start"```    
```rostopic pub /slip_detection/intensity_extractor/event_in std_msgs/String "e_start"```    
```rostopic pub /slip_detection/tactile_flow_extractor/event_in std_msgs/String "e_start"```    
```rostopic pub /slip_detection/slip_detector/event_in std_msgs/String "e_start"```    

### To stop the component
```rostopic pub /slip_detection/slip_detector/event_in std_msgs/String "e_stop"```
