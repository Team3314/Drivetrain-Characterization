package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.CustomPIDOutput;
import frc.robot.infrastructure.Drivetrain;
import frc.robot.infrastructure.IdleMode;
import frc.robot.infrastructure.EncoderTransmission;
import frc.robot.infrastructure.SpeedControllerMode;

/*
SPARK FF TO DUTY CYCLE CONVERSION
DUTY CYCLE = FF * 300
*/

public class Drive extends Drivetrain implements Subsystem {

//Hardware

    private AHRS navx;

    private DoubleSolenoid shifter;

    public enum DriveMode {
		IDLE,
		OPEN_LOOP,
        GYROLOCK,
        POSITION,
        VELOCITY,
        MOTION_PROFILE
	}

    //Control Modes
    private DriveMode currentDriveMode = DriveMode.OPEN_LOOP;
    SpeedControllerMode controlMode = SpeedControllerMode.kIdle;
    
    //Hardware states
    private boolean mIsHighGear, integratedEncoder;
    private IdleMode idleMode;
    private double rawLeftSpeed, rawRightSpeed, desiredAngle, tickToInConversion, offsetL, offsetR;
    
    private double leftDrivePositionInches, rightDrivePositionInches, leftDriveSpeedTicks, rightDriveSpeedTicks, 
        leftDriveSpeedInches, rightDriveSpeedInches, ticksLeftHighGear, ticksLeftLowGear, ticksRightHighGear, ticksRightLowGear, ticksPerRev, inchesPerRev;
    private PIDController gyroControl;
    private CustomPIDOutput gyroPIDOutput;

    public Drive(EncoderTransmission left, EncoderTransmission right, AHRS gyro, DoubleSolenoid shifter){
        super(left, right);
        integratedEncoder = left.encoderIsSparkMax();
        if(integratedEncoder)
            ticksPerRev = Constants.kNEODriveEncoderCodesPerRev;
        else {
            ticksPerRev = Constants.kDriveEncoderCodesPerRev;
            inchesPerRev = Constants.kRevToInConvFactor;
        }

    	
		//Hardware
    	this.shifter = shifter;
        navx = gyro;
        

        gyroPIDOutput = new CustomPIDOutput();
    	gyroControl = new PIDController(Constants.kGyroLock_kP, Constants.kGyroLock_kI, Constants.kGyroLock_kD,
            Constants.kGyroLock_kF, navx, gyroPIDOutput);
		//Sets the PID controller to treat 180 and -180 to be the same point, 
		//so that when turning the robot takes the shortest path instead of going the long way around
		//Effectively changes PID input from a line to a circle
		gyroControl.setOutputRange(-Constants.kGyroOutputRange, Constants.kGyroOutputRange);		// Limits speed of turn to prevent overshoot
		gyroControl.setAbsoluteTolerance(Constants.kAbsoluteGyroTolerance);

    }

    public void update(){
        if(mIsHighGear) {
            shifter.set(Constants.kHighGear);
            ticksLeftHighGear =  getLeftPositionTicks() - ticksLeftLowGear;
            ticksRightHighGear = getRightPositionTicks() - ticksRightLowGear;
            if(integratedEncoder) {
                inchesPerRev = Constants.kRevToInConvFactorHighGear;
                tickToInConversion = inchesPerRev / ticksPerRev;
            }   
    	}
    	else {
            shifter.set(Constants.kLowGear);
            ticksLeftLowGear =  getLeftPositionTicks() - ticksLeftHighGear;
            ticksRightLowGear = getRightPositionTicks() - ticksRightHighGear;
            if(integratedEncoder) {
                inchesPerRev = Constants.kRevToInConvFactorLowGear;
                tickToInConversion = inchesPerRev / ticksPerRev;
            }
        }
        updateSpeedAndPosition();
        switch(currentDriveMode) {
            case IDLE:
                controlMode = SpeedControllerMode.kDutyCycle;
                setIdleMode(IdleMode.kBrake);
                rawLeftSpeed = 0;
                rawRightSpeed = 0;
                break;
            case OPEN_LOOP:
                rawLeftSpeed = leftDemand * Math.abs(leftDemand);
                rawRightSpeed = rightDemand * Math.abs(rightDemand);
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case GYROLOCK:
                rawLeftSpeed = leftDemand + gyroPIDOutput.getOutput();
                rawRightSpeed = rightDemand - gyroPIDOutput.getOutput();
                gyroControl.setSetpoint(desiredAngle);
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kDutyCycle;
                break;
            case POSITION:
                rawLeftSpeed = leftDemand;
                rawRightSpeed = rightDemand;
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kPosition;
                break;
            case VELOCITY:
                if(mIsHighGear) {
                    rawLeftSpeed = leftDemand * Constants.kMaxSpeed;
                    rawRightSpeed = rightDemand * Constants.kMaxSpeed;
                }
                else {
                    rawLeftSpeed = leftDemand * Constants.kMaxSpeed;
                    rawRightSpeed = rightDemand * Constants.kMaxSpeed;
                }
                setIdleMode(IdleMode.kCoast);
                controlMode = SpeedControllerMode.kVelocity;
                break;
            case MOTION_PROFILE:
                setIdleMode(IdleMode.kBrake);
                controlMode = SpeedControllerMode.kVelocity;
                rawLeftSpeed = leftDemand;
                rawRightSpeed = rightDemand;
                break;
        }
        rightDrive.set(rawRightSpeed, controlMode);
        leftDrive.set(rawLeftSpeed, controlMode);

    }

    public void setIdleMode(IdleMode mode) {
    	if(idleMode != mode) {
            idleMode = mode;

            leftDrive.setIdleMode(mode);
            rightDrive.setIdleMode(mode);
    	}
    }
    
    public void setDesiredAngle(double angle) {
    	desiredAngle = -angle;
    }
    
    public double getDesiredAngle() {
    	return desiredAngle;
    }
    
    public double getAngle() {
    	return -navx.getYaw();
    }
    
    public double getAcceleration(){
        return Math.round(100* navx.getWorldLinearAccelY());
    }

    public double getLeftPositionTicks() {
        return leftDrive.getPosition() - offsetL;
    }

    public double getRightPositionTicks() {
        return rightDrive.getPosition() - offsetR;
    }
    
    public double getLeftPosition() {
    	return leftDrivePositionInches;
    }
    
    public double getRightPosition() {
    	return -rightDrivePositionInches;
    }
    
    public double getAveragePosition() {
    	return (getLeftPosition()+getRightPosition())/2;
    }
    public double getLeftDriveSpeedInches() {
        return leftDriveSpeedInches;
    }
    public double getRightDriveSpeedInches() {
        return rightDriveSpeedInches;
    }
    
    public void setDriveMode(DriveMode mode) {
        if(mode != currentDriveMode) {
            if(mode == DriveMode.GYROLOCK) {
                gyroControl.enable();
                setDesiredAngle(getAngle());
            }
            else {
                gyroControl.disable();
            }

            if(mode == DriveMode.MOTION_PROFILE) {

            }
            currentDriveMode = mode;
        }
    }

    public void updateSpeedAndPosition() {
        leftDrivePositionInches = ticksLeftHighGear * Constants.kRevToInConvFactorHighGear + ticksLeftLowGear * Constants.kRevToInConvFactorLowGear;
        rightDrivePositionInches = ticksRightHighGear * Constants.kRevToInConvFactorHighGear + ticksRightLowGear * Constants.kRevToInConvFactorLowGear;
        leftDriveSpeedTicks = leftDrive.getVelocity();
        rightDriveSpeedTicks = rightDrive.getVelocity();
        leftDriveSpeedInches = leftDriveSpeedTicks * tickToInConversion;
        rightDriveSpeedInches = rightDriveSpeedTicks * tickToInConversion;

    }
    
    public void setHighGear(boolean highGear) {
        mIsHighGear = highGear;
    }
    
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Left Encoder Position Ticks", getLeftPositionTicks());
    	SmartDashboard.putNumber("Right Encoder Position Ticks", getRightPositionTicks());
    	SmartDashboard.putNumber("Left Encoder Inches", getLeftPosition());
    	SmartDashboard.putNumber("Right Encoder Inches", getRightPosition());
    	SmartDashboard.putNumber("Left Encoder Speed RPS", leftDriveSpeedTicks);
        SmartDashboard.putNumber("Right Encoder Speed RPS", rightDriveSpeedTicks);
        SmartDashboard.putNumber("Left Encoder Speed Inches", leftDriveSpeedInches);
        SmartDashboard.putNumber("Right Encoder Speed Inches", rightDriveSpeedInches);
        SmartDashboard.putNumber("Avg. Position", getAveragePosition());
    	SmartDashboard.putNumber("Left Master Current", leftDrive.getOutputCurrent(0));
    	SmartDashboard.putNumber("Left Slave 1 Current", leftDrive.getOutputCurrent(1));
    	SmartDashboard.putNumber("Left Slave 2 Current", leftDrive.getOutputCurrent(2));
    	SmartDashboard.putNumber("Right Master Current", rightDrive.getOutputCurrent(0));
    	SmartDashboard.putNumber("Right Slave 1 Current", rightDrive.getOutputCurrent(1));
    	SmartDashboard.putNumber("Right Slave 2 Current", rightDrive.getOutputCurrent(2));
    	SmartDashboard.putString("Drive Mode", String.valueOf(currentDriveMode));
    	SmartDashboard.putString("Neutral Mode", String.valueOf(idleMode));
    	SmartDashboard.putNumber("Raw Left Speed", rawLeftSpeed);
    	SmartDashboard.putNumber("Raw Right Speed", rawRightSpeed);
    	SmartDashboard.putNumber("Desired Angle", desiredAngle);
    	SmartDashboard.putNumber("Current angle", getAngle());
        SmartDashboard.putNumber("Gyro adjustment", gyroPIDOutput.getOutput());
        SmartDashboard.putBoolean("Gyro Turn Done", gyroInPosition());
    	SmartDashboard.putNumber("Left Voltage", leftDrive.getOutputVoltage());
        SmartDashboard.putNumber("Right Voltage", rightDrive.getOutputVoltage());
        SmartDashboard.putNumber("Accelerometer", getAcceleration());
    }
  
    public void resetDriveEncoders() {
        offsetL = leftDrive.getPosition();
        offsetR = rightDrive.getPosition();
        ticksLeftHighGear = 0;
        ticksLeftLowGear = 0;
        ticksRightHighGear = 0;
        ticksRightLowGear = 0;
    }
    
    public void resetSensors() {
    	navx.reset();
    	resetDriveEncoders();
    }
    
    public boolean gyroInPosition() {
    	return gyroControl.onTarget();
    }

    public boolean driveInPosition() {
        return false;
    }

    public void setRampRate(double rate){ 
        leftDrive.setRampRate(rate);
        rightDrive.setRampRate(rate);
    }
}