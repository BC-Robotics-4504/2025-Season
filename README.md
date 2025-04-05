# 2025-Season: Smoky Mountain Regional

## Robotpy Setup

BC Robotics (Team #4504) has designed their 2024 FIRST Robotics submission using [Robotpy](https://robotpy.readthedocs.io/en/stable/install/robot.html) with the [MagicBot Framework](https://robotpy.readthedocs.io/en/stable/frameworks/magicbot.html). To keep the relevant libraries current, the following code will need to be run regularly in a terminal interface:

```bash
python -m pip install --upgrade pip
python -m pip install robotpy wpilib
robotpy installer download-python
robotpy installer install-python
robotpy sync
robotpy installer download robotpy[all]
robotpy installer install robotpy[all]
```

### Swerve Drive

- **[SparkMax](https://www.revrobotics.com/rev-11-2158/)** (8x):
  - Drives [MK4i Swerve Module](https://www.swervedrivespecialties.com/products/mk4i-swerve-module) (4x)

| Component                 | Type             | ID/Details | Inverted | Gear Ratio | Wheel Diameter | Absolute Encoder | Z-Offset   |
|---------------------------|------------------|------------|----------|------------|----------------|------------------|------------|
| Front Left Angle Motor    | SparkMaxTurning  | 6          | No       | 1          | 1              | Yes              | 5.7535123  |
| Front Left Speed Motor    | SparkMaxDriving  | 5          | No       | 1          | 0.1143         |                  |            |
| Rear Left Angle Motor     | SparkMaxTurning  | 8          | No       | 1          | 1              | Yes              | 5.6867370  |
| Rear Left Speed Motor     | SparkMaxDriving  | 7          | No       | 1          | 0.1143         |                  |            |
| Rear Right Angle Motor    | SparkMaxTurning  | 2          | No       | 1          | 1              | Yes              | 5.5975077  |
| Rear Right Speed Motor    | SparkMaxDriving  | 1          | No       | 1          | 0.1143         |                  |            |
| Front Right Angle Motor   | SparkMaxTurning  | 4          | No       | 1          | 1              | Yes              | 0.0182671  |
| Front Right Speed Motor   | SparkMaxDriving  | 3          | No       | 1          | 0.1143         |                  |            |



### **[GameSir G7 Wired Controller](https://www.amazon.com/dp/B0BM9HRCCV?ref_=cm_sw_r_apin_dp_ER34REM3C1FQSY0W5MQR)**

- Input device ID `0`
- **Left Joystick (Lx, Ly)**: Controls the robot's movement in the field. The X-axis (Lx) translates to movement in the x-direction while the Y-axis translates to movement in the y-direction.
- **Right Joystick (Rx)**: Controls the robot's rotation on the field. The X-axis (Rx) influences the angular velocity of the robot.


## Autonomous Operation

  **Our autonomous plan is as follows...**

  1. Backup 2.75844 meters to exit community
  
