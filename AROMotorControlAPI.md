# AROMotorControl API Documentation

The `AROMotorControl` class provides methods to interface with the motors.

## Public Methods

### `setZero()`
Sets the zero position for the specified motor. 
#### Parameters
- `motor_id` (int): The ID of the motor (default: 1). Possible Values are 1 or 2.

#### Returns
- `result` (bool): Returns `True` if the zero position was set successfully.

Please note that after calling this method, the power supply needs to be turned off and on again for the setZero command to take effect. A suggested approach to set zero position as facing forwards for both motors at the same time is:
- Rotate both motors to the desired position and optionally use the 3d-printed aligner to lock the motors in place
- run the following code:
```python
mc = AROMotorControl()
mc.setZero(1)
mc.setZero(2)
input("press any key to continue") 
```
- when you see this message turn off your power supply, wait for at least 2 seconds, turn it back on again, wait for 2 seconds, remove the aligner, and press enter (or any other key).
- you can verify if the process was succeeful by calling
```python
mc.readPosition(motor_id)
```

### `readPosition()`
Reads the position of the specified motor.

#### Parameters
- `motorid` (int): The ID of the motor (default: 1). Possible Values are 1 or 2.

#### Returns
- `position` (int): The angle of the motor in degrees.

### `readPositionContinuous()`

This method is used to read the positions of both motor continuously at a specified frequency. This method returns a generator that can be used to yield the motor angles at each interval indefinitely.

### Parameters:
- `frequency` (int, default=100): The frequency, in Hertz, at which to read the angle.

### Returns:

- A generator that yields the motor angle at each interval.

### Example Usage:

```python
motor_control = AROMotorControl()
motor_positions = motor_control.readAngleContinuous(motorid=1, frequency=100)
while some_condition:
    pos1, pos2 = next(motor_positions)
    if some_condition:
        break
```
### `applyCurrentToMotor()`
Applies a current to the specified motor.

#### Parameters
- `motorid` (int): The ID of the motor (default: 1).
- `current` (float): The current to be applied in Amps (default: 0.18).

#### Returns
- `motor_temp` (int): The temperature of the motor.
- `current` (int): The current applied to the motor
- `speed` (int): The speed.
- `position` (int): The angle in degrees.

### `positionControl()`
Controls the position of the specified motor using the inbuilt controllers of the motor. This method can be used to test or debug the setup

#### Parameters
- `motorid` (int): The ID of the motor (default: 1). Possible values are 1 or 2.
- `rotation_dir` (int): The direction of rotation (default: 0). 0 means clockwise and 1 means anticlockwise
- `speed_limit` (int): The speed limit (default: 0).
- `position` (int): The desired position (default: 0).
- `run_async` (bool): Run in asynchronous mode.

#### Returns
- `motor_temp` (int): The temperature of the motor.
- `torque` (int): The torque.
- `speed` (int): The speed.
- `angle` (int): The angle.

<!-- ### `readPID()`
Reads the PID parameters of the specified motor.

#### Parameters
- `motorid` (int): The ID of the motor (default: 1). Possible Values are 1 or 2.
#### Returns
- `kpCurrent` (int): The current proportional gain.
- `kiCurrent` (int): The current integral gain.
- `KpVel` (int): The velocity proportional gain.
- `KiVel` (int): The velocity integral gain.
- `KpPos` (int): The position proportional gain.
- `KiPos` (int): The position integral gain.

### `setPIDInRAM()`
Sets the PID parameters in RAM for the specified motor.

#### Parameters
- `motorid` (int): The ID of the motor (default: 1).
- `KpCurrent` (int): The desired current proportional gain (default: 0).
- `KiCurrent` (int): The desired current integral gain (default: 0).
- `KpVel` (int): The desired velocity proportional gain (default: 0).
- `KiVel` (int): The desired velocity integral gain (default: 0).
- `KpPos` (int): The desired position proportional gain (default: 0).
- `KiPos` (int): The desired position integral gain (default: 0).

#### Returns
- `True` (bool): Returns `True` if the PID parameters were set successfully.

### `setPIDInROM()`
Sets the PID parameters in ROM for the specified motor.

#### Parameters
- `motorid` (int): The ID of the motor (default: 1).
- `KpCurrent` (int): The desired current proportional gain (default: 0).
- `KiCurrent` (int): The desired current integral gain (default: 0).
- `KpVel` (int): The desired velocity proportional gain (default: 0).
- `KiVel` (int): The desired velocity integral gain (default: 0).
- `KpPos` (int): The desired position proportional gain (default: 0).
- `KiPos` (int): The desired position integral gain (default: 0).

#### Returns
- `result` (bool): Returns `True` if the PID parameters were set successfully. -->