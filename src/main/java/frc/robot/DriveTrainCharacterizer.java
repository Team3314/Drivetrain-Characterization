package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;

public class DriveTrainCharacterizer {
	
	public static enum TestMode {
		QUASI_STATIC, STEP_VOLTAGE;
	}

	public static enum Direction {
		Forward, Backward;
	}
	
	private Drive drive = Robot.drive;
	private Direction direction = Direction.Forward;
	private TestMode mode = TestMode.QUASI_STATIC;
	private double voltageStep = 1.0 / 48.0 / 50.0;
	double driveSpeed;
	
	public void initialize() {
		drive.resetSensors();
		double scale;
		if (direction.equals(Direction.Forward)) {
			scale = 1;
		} 
		else {
			scale = -1;
		}
		if (mode.equals(TestMode.QUASI_STATIC)) {
			driveSpeed = 0;
			voltageStep *= scale;
		} 
		else {
            driveSpeed = .5 * scale; //6v
            drive.setRampRate(0);
		}
	}
	
	public void run() {
		if(mode.equals(TestMode.QUASI_STATIC))
			driveSpeed += voltageStep;
		drive.set(driveSpeed, driveSpeed);

    }
    
    public void setMode(TestMode mode, Direction dir) {
        direction = dir;
        this.mode = mode;
    }
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Left Rio Drive speed FPS", drive.getLeftRioDriveSpeedInches() / 12);
        SmartDashboard.putNumber("Right Rio Drive speed FPS", drive.getRightRioDriveSpeedInches() / 12);
        SmartDashboard.putNumber("Left NEO Drive speed FPS", drive.getLeftNEODriveSpeedInches() / 12);
		SmartDashboard.putNumber("Right NEO Drive speed FPS", drive.getRightNEODriveSpeedInches() / 12);
		SmartDashboard.putString("Mode" ,mode.toString());
		SmartDashboard.putString("Direction", direction.toString());

    }
}