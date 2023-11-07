from motor_control.AROMotorControl import AROMotorControl

motor_control = AROMotorControl()

def run_try_catch(func, *args, **kwargs):
    '''helper function to encaspsulate try / catch template'''
    try:
        # Please run your code in a try-catch block so that when you terminate 
        # your program (using ctrl+c or otherwise) or an error occurs, the motors stop.
        # Otherwise the motors will keep executing the last command applied to them.
        func(*args, **kwargs)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, stopping motors...")
    except Exception as e:
        print(f"an error occurred: {e}")
    finally:
        motor_control.applyCurrentToMotor(1, 0)
        motor_control.applyCurrentToMotor(2, 0)
        print("motors stopped!")

if __name__ == "__main__":

    try:
        # Please run your code in a try-catch block so that when you terminate 
        # your program (using ctrl+c or otherwise) or an error occurs, the motors stop.
        # Otherwise the motors will keep executing the last command applied to them.
        pass
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, stopping motors...")
    except Exception as e:
        print(f"an error occurred: {e}")
    finally:
        motor_control.applyCurrentToMotor(1, 0)
        motor_control.applyCurrentToMotor(2, 0)
        print("motors stopped!")