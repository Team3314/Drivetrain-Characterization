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
	private Direction direction;
	private TestMode mode;
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
            driveSpeed = 1 * scale;
            drive.setRampRate(80);
		} 
		else {
            driveSpeed = .5 * scale; //6v
            drive.setRampRate(0);
		}
	}
	
	public void run() {
        drive.set(driveSpeed, driveSpeed);
    }
    
    public void setMode(TestMode mode, Direction dir) {
        direction = dir;
        this.mode = mode;
    }
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Left Drive speed FPS", drive.getLeftDriveSpeedInches() / 12);
        SmartDashboard.putNumber("Right Drive speed FPS", drive.getRightDriveSpeedInches() / 12);
    }
}