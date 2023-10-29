# AROMotorControl API Documentation

The `AROMotorControl` class provides methods to interface with the motors.

## Public Methods

### `readPosition(self, motorid=1, duration=10, frequency=100)`
Reads the position of the specified motor.

#### Parameters
- `motorid` (int): The ID of the motor (default: 1). Possible Values are 1 or 2.

#### Returns
- `value` (int): The angle of the motor.

### `readPositionContinuous`

This function is used to read the positions of both motor continuously at a specified frequency. This function returns a generator that yields the motor angles at each interval indefinitely.

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


### `readPID(self, motorid=1, duration=10)`
Reads the PID parameters of the specified motor.

#### Parameters
- `motorid` (int): The ID of the motor (default: 1).
#### Returns
- `kpCurrent` (int): The current proportional gain.
- `kiCurrent` (int): The current integral gain.
- `KpVel` (int): The velocity proportional gain.
- `KiVel` (int): The velocity integral gain.
- `KpPos` (int): The position proportional gain.
- `KiPos` (int): The position integral gain.

### `setPIDInRAM(self, motorid=1, KpCurrent=0, KiCurrent=0, KpVel=0, KiVel=0, KpPos=0, KiPos=0)`
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

### `setPIDInROM(self, motorid=1, KpCurrent=0, KiCurrent=0, KpVel=0, KiVel=0, KpPos=0, KiPos=0)`
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
- `result` (bool): Returns `True` if the PID parameters were set successfully.

### `positionControl(self, motorid=1, rotation_dir=0, speed_limit=0, position=0)`
Controls the position of the specified motor using the inbuilt controllers of the motor. This method can be used to test or debug the setup

#### Parameters
- `motorid` (int): The ID of the motor (default: 1).
- `rotation_dir` (int): The direction of rotation (default: 0). 0 means clockwise and 1 means anticlockwise
- `speed_limit` (int): The speed limit (default: 0).
- `position` (int): The desired position (default: 0).

#### Returns
- `motor_temp` (int): The temperature of the motor.
- `torque` (int): The torque.
- `speed` (int): The speed.
- `angle` (int): The angle.

### `setZero(self, motor_id=1)`
Sets the zero position for the specified motor.

#### Parameters
- `motor_id` (int): The ID of the motor (default: 1).

#### Returns
- `result` (bool): Returns `True` if the zero position was set successfully.

### `applyCurrentToMotor(self, motorid=1, current=0.18)`
Applies a current to the specified motor.

#### Parameters
- `motorid` (int): The ID of the motor (default: 1).
- `current` (float): The current to be applied (default: 0.18).

#### Returns
- `motor_temp` (int): The temperature of the motor.
- `current` (int): The current.
- `speed` (int): The speed.
- `angle` (int): The angle.
