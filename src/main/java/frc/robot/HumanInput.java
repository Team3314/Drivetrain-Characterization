
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class HumanInput {
	
	private final Joystick leftStick;
	private final Joystick rightStick;
    
	public HumanInput() {
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
	}
	public boolean getQuasiStaticBack() {
		return rightStick.getRawButton(3);
	}
	public boolean getVoltageStepBack() {
		return rightStick.getRawButton(4);
	}
	public boolean getQuasiStaticForward() {
		return rightStick.getRawButton(5);
	}
	public boolean getVoltageStepForward() {
		return rightStick.getRawButton(6);
	}
	public boolean getStop() {
		return rightStick.getRawButton(2);
	}
	public boolean getStart() {
		return rightStick.getRawButton(10);
	}
    public double getLeftThrottle() {
		return -leftStick.getRawAxis(1);
	}
	public double getRightThrottle() {
		return -rightStick.getRawAxis(1);
	}
	public boolean getLowGear() {
		return leftStick.getRawButton(4);
	}
	public boolean getHighGear() {
		return leftStick.getRawButton(6);
	}
	public boolean getGyrolock() {
		return rightStick.getRawButton(1);
	}
}