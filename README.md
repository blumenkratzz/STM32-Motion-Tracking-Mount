---
cssclass: academia_pdf
---


# **Motion Tracking Camera Mount - User Manual**


## **1. In the Box**

- Motion Tracking Camera Mount Assembly
- Power Adapter (12V DC)
- User Manual
- [Optional] Wall-mount screws/zipties



## **2. Product Overview**

The Motion Tracking Mount is an _indoor-only_ device that tracks a person horizontally and vertically. It is ideal for applications like personal filming, automation, or monitoring.

### Key Features:

- **Range:** Tracks subjects within 2–7 meters.
- **Field of View:**
    - Horizontal: 150°
    - Vertical: 40–45°
- **Tracking Speed:** Up to 1 foot per second.
- **Response Time:** Adjustable (0.3s to 3s) using the **configurable knob**.
- **Indicators:** Green LED shows active tracking.
- **Return to Home:** Mount defaults to a "home position" when the target is out of range.

<div style="height: 100px;"></div>

Next page contains an overview of components to help users identify key features.

<div style="page-break-after: always;"></div>

### **Identifying the Mount Components**

1. **Mount from a User’s Perspective**  
    _Below is an image showing the mount from a standard distance of approximately 40 cm, as viewed by a person:_

<div style="height: 300px;"></div>

![350x350](assets/Mount_User_Perspective%203.jpg)
Image 1: Full mount  image from user perspective.
    
2. **Highlighted Components (Top View)**  
    _The second image shows the mount from a top-down view with the following components highlighted:_
    
    - **Tracking LED**: Indicates active tracking status.
    - **Potentiometer Knob**: Used for adjusting tracking response speed.
    - **Payload Clamp**: Secures the user’s device (e.g., camera or sensors).

<div style="height: 350px;"></div>

![500x500](assets/Pasted%20image%2020241206193052.png)
Image 2: Top-view image highlighting LED (in red), configurable knob (in yellow), and clamp (in green).

<div style="page-break-after: always;"></div>

## **3. Assembly**

1. **Mount Setup:**
    - Secure the 3D-printed base to a **wall** or **surface** using screws/zipties.
2. **Power Connection:**
    - Connect the supplied **12V adapter** to the mount and AC outlet.
3. **Sensor Alignment:**
    - Ensure the 3 **MLX90640 thermal sensors** are unobstructed.



## **4. How It Works**

### **Tracking System**

- **Sensors:** The 3 MLX90640 FIR sensors provide a combined **150° horizontal** and **45° vertical** FOV.
- The mount follows the **closest person** in view.

### **Tracking Speed Configuration**

- Use the **configurable knob**:
    - **0s:** Fastest response (~0.3s).
    - **Up to 3s:** Slowest tracking response.

### **LED Indicator**

- **Solid Green:** Tracking active.
- **Off:** No target detected; mount is idle.



## **5. Operation Instructions**

1. **Power On:**
    - Plug in the device; it will initialize and move to the home position.
2. **Position Yourself:**
    - Stand **2–7 meters** away within the sensors' range.
3. **Start Tracking:**
    - The mount automatically locks onto the closest person.
4. **Adjust Speed:**
    - Turn the **knob** to modify the response time.

<div style="page-break-after: always;"></div>

## **6. Specifications**

|**Feature**|**Details**|
|---|---|
|Power Input|12V DC (AC Adapter included)|
|Tracking Range|2–7 meters|
|Horizontal Rotation|150°|
|Vertical Rotation|45°|
|Max Tracking Speed|1 foot/second|
|Response Time (Adjustable)|0.3–3 seconds|
|LED Indicator|Green (tracking status)|
|Operating Environment|Indoor only|
|Sensors|MLX90640 FIR (3 units, 55°x35° FOV each)|
|Motors|ST-3025BL Servo Motors|
|Mount Material|3D-printed using PLA filament|




## **7. Troubleshooting**

|**Issue**|**Solution**|
|---|---|
|**Mount does not power on.**|Check the power connection and ensure the adapter is plugged into a working outlet.|
|**LED is off; no tracking.**|Ensure the target is within the **2–7 meters** range and the sensors are unobstructed.|
|**LED is on, but motors don’t move.**|Unplug the system for **15 seconds**, then reconnect it to reset the controller.|
|**Motors move incorrectly or chaotically.**|Ensure there are no heaters, radiators, or strong heat sources in the sensors' field of view, as they can interfere with readings.|
|**Tracking is too slow or fast.**|Adjust the **configurable knob** to modify the tracking speed.|
|**Mount does not move horizontally.**|Inspect the horizontal servo motor for obstructions or mechanical issues.|


## **8. Maintenance and Safety**

- Keep sensors clean and free of obstructions.
- Do not expose to moisture or outdoor environments.
- Avoid tampering with internal electronics.

