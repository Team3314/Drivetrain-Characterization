package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.CustomPIDOutput;
import frc.robot.infrastructure.Drivetrain;
import frc.robot.infrastructure.EncoderAdapter;
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
        VISION_CONTROL, 
        MOTION_PROFILE
	}

    //Control Modes
    private DriveMode currentDriveMode = DriveMode.OPEN_LOOP;
    SpeedControllerMode controlMode = SpeedControllerMode.kIdle;
    
    //Hardware states
    private boolean mIsHighGear;
    private IdleMode idleMode;
    private double rawLeftSpeed, rawRightSpeed, desiredAngle, tickToInConversion, neoOffsetL, neoOffsetR;
    
    private double leftRioDrivePositionInches, rightRioDrivePositionInches, leftRioDrivePositionTicks, rightRioDrivePositionTicks, leftRioDriveSpeedTicks, rightRioDriveSpeedTicks, 
        leftRioDriveSpeedInches, rightRioDriveSpeedInches, rioTicksPerRev, rioInchesPerRev;

    private double leftNeoDrivePositionInches, rightNeoDrivePositionInches, leftNeoDrivePositionTicks, rightNeoDrivePositionTicks, leftNeoDriveSpeedTicks, rightNeoDriveSpeedTicks, 
    leftNeoDriveSpeedInches, rightNeoDriveSpeedInches, ticksLeftNeoHighGear, ticksLeftNeoLowGear, ticksRightNeoHighGear, ticksRightNeoLowGear, neoInchesPerRev;
    private PIDController gyroControl;
    private CustomPIDOutput gyroPIDOutput;

    private EncoderAdapter leftRioEncoder, rightRioEncoder;

    public Drive(EncoderTransmission left, EncoderTransmission right, AHRS gyro, DoubleSolenoid shifter, EncoderAdapter leftEnc, EncoderAdapter rightEnc){
        super(left, right);
        leftRioEncoder = leftEnc;
        rightRioEncoder = rightEnc;
        

    	
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
        gyroControl.setInputRange(-180, 180);
        gyroControl.setContinuous(true);

    }

    public void update(){
       if(mIsHighGear) {
            shifter.set(Constants.kHighGear);
            ticksLeftNeoHighGear =  getLeftNeoPositionTicks() - ticksLeftNeoLowGear;
            ticksRightNeoHighGear = getRightNeoPositionTicks() - ticksRightNeoLowGear;
            neoInchesPerRev = Constants.kRevToInConvFactorHighGear;
    	}
    	else {
            shifter.set(Constants.kLowGear);
            ticksLeftNeoLowGear =  getLeftNeoPositionTicks() - ticksLeftNeoHighGear;
            ticksRightNeoLowGear = getRightNeoPositionTicks() - ticksRightNeoHighGear;
            neoInchesPerRev = Constants.kRevToInConvFactorLowGear;
        }
        tickToInConversion = neoInchesPerRev / Constants.kNEODriveEncoderCodesPerRev;
        updateSpeedAndPosition();
        switch(currentDriveMode) {
            case IDLE:
                controlMode = SpeedControllerMode.kDutyCycle;
                setIdleMode(IdleMode.kBrake);
                rawLeftSpeed = 0;
                rawRightSpeed = 0;
                break;
            case OPEN_LOOP:
                rawLeftSpeed = leftDemand;
                rawRightSpeed = rightDemand;
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

    public double getLeftNeoPositionTicks() {
        return leftDrive.getPosition() - neoOffsetL;
    }

    public double getRightNeoPositionTicks() {
        return rightDrive.getPosition() - neoOffsetR;
    }
    
    public double getLeftNeoPosition() {
    	return leftNeoDrivePositionInches;
    }
    
    public double getRightNeoPosition() {
    	return -rightNeoDrivePositionInches;
    }
    
    public double getAverageNeoPosition() {
    	return (leftNeoDrivePositionInches-rightNeoDrivePositionInches)/2;
    }

    public double getLeftRioPositionTicks() {
        return -leftRioEncoder.getEncoderCounts();
    }

    public double getRightRioPositionTicks() {
        return rightRioEncoder.getEncoderCounts();
    }
    
    public double getLeftRioPosition() {
    	return leftRioDrivePositionInches;
    }
    
    public double getRightRioPosition() {
    	return rightRioDrivePositionInches;
    }
    
    public double getAverageRioPosition() {
    	return (leftRioDrivePositionInches-rightRioDrivePositionInches)/2;
    }

    public double getLeftRioDriveSpeedInches() {
        return leftRioDriveSpeedInches;
    }
    public double getRightRioDriveSpeedInches() {
        return leftRioDriveSpeedInches;
    }
    public double getLeftRioDriveSpeedTicks() {
        return leftRioDriveSpeedInches;
    }
    public double getRightRioDriveSpeedTicks() {
        return leftRioDriveSpeedInches;
    }
    public double getLeftNEODriveSpeedInches() {
        return leftNeoDriveSpeedInches;
    }
    public double getRightNEODriveSpeedInches() {
        return leftNeoDriveSpeedInches;
    }
    public double getLeftNEODriveSpeedTicks() {
        return leftNeoDriveSpeedInches;
    }
    public double getRightNEODriveSpeedTicks() {
        return leftNeoDriveSpeedInches;
    }
    
    public void setDriveMode(DriveMode mode) {
        if(mode != currentDriveMode) {
            if(mode == DriveMode.GYROLOCK) {
                gyroControl.enable();
                setDesiredAngle(getAngle());
            }
            else if(mode == DriveMode.VISION_CONTROL) {
                gyroControl.enable();
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
        leftNeoDrivePositionTicks = getLeftNeoPositionTicks();
        rightNeoDrivePositionTicks = getRightNeoPositionTicks();
        leftNeoDrivePositionInches = ticksLeftNeoHighGear * Constants.kNeoTicksToInHighGear + ticksLeftNeoLowGear * Constants.kNeoTicksToInLowGear;
        rightNeoDrivePositionInches = ticksRightNeoHighGear * Constants.kNeoTicksToInHighGear + ticksRightNeoLowGear * Constants.kNeoTicksToInLowGear;
        leftNeoDriveSpeedTicks = leftDrive.getVelocity();
        rightNeoDriveSpeedTicks = rightDrive.getVelocity();
        leftNeoDriveSpeedInches = leftNeoDriveSpeedTicks * tickToInConversion;
        rightNeoDriveSpeedInches = rightNeoDriveSpeedTicks * tickToInConversion;
        leftRioDrivePositionTicks = getLeftRioPositionTicks();
        rightRioDrivePositionTicks = getRightRioPositionTicks();
        leftRioDrivePositionInches = leftRioDrivePositionTicks * Constants.kDriveTicksToInches;
        rightRioDrivePositionInches = rightRioDrivePositionTicks * Constants.kDriveTicksToInches;
        leftRioDriveSpeedTicks = -leftRioEncoder.getVelocity();
        rightRioDriveSpeedTicks = rightRioEncoder.getVelocity();
        leftRioDriveSpeedInches = leftRioDriveSpeedTicks * Constants.kDriveTicksToInches;
        rightRioDriveSpeedInches = rightRioDriveSpeedTicks * Constants.kDriveTicksToInches;

    }
    
    public void setHighGear(boolean highGear) {
        mIsHighGear = highGear;
    }
    
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Left NEO Encoder Position Ticks", leftNeoDrivePositionTicks);
    	SmartDashboard.putNumber("Right NEO Encoder Position Ticks", rightNeoDrivePositionTicks);
    	SmartDashboard.putNumber("Left NEO Encoder Inches", getLeftNeoPosition());
    	SmartDashboard.putNumber("Right NEO Encoder Inches", getRightNeoPosition());
    	SmartDashboard.putNumber("Left EO Encoder Speed RPS", leftNeoDriveSpeedTicks);
        SmartDashboard.putNumber("Right NEO Encoder Speed RPS", rightNeoDriveSpeedTicks);
        SmartDashboard.putNumber("Left NEO Encoder Speed Inches", leftNeoDriveSpeedInches);
        SmartDashboard.putNumber("Right NEO Encoder Speed Inches", rightNeoDriveSpeedInches);
    	SmartDashboard.putNumber("Left Rio Encoder Position Ticks", leftRioDrivePositionTicks);
    	SmartDashboard.putNumber("Right Rio Encoder Position Ticks", rightRioDrivePositionTicks);
    	SmartDashboard.putNumber("Left Rio Encoder Inches", getLeftRioPosition());
    	SmartDashboard.putNumber("Right Rio Encoder Inches", getRightRioPosition());
    	SmartDashboard.putNumber("Left Rio Encoder Speed Ticks", leftRioDriveSpeedTicks);
        SmartDashboard.putNumber("Right Rio Encoder Speed Ticks", rightRioDriveSpeedTicks);
        SmartDashboard.putNumber("Left Rio Encoder Speed Inches", leftRioDriveSpeedInches);
        SmartDashboard.putNumber("Right Rio Encoder Speed Inches", rightRioDriveSpeedInches);
    	SmartDashboard.putNumber("Left Master Current", leftDrive.getOutputCurrent(0));
    	SmartDashboard.putNumber("Left Slave 1 Current", leftDrive.getOutputCurrent(1));
    	SmartDashboard.putNumber("Left Slave 2 Current", leftDrive.getOutputCurrent(2));
    	SmartDashboard.putNumber("Right Master Current", rightDrive.getOutputCurrent(0));
    	SmartDashboard.putNumber("Right Slave 1 Current", rightDrive.getOutputCurrent(1));
    	SmartDashboard.putNumber("Right Slave 2 Current", rightDrive.getOutputCurrent(2));
    	SmartDashboard.putString("Drive Mode", String.valueOf(currentDriveMode));
    	SmartDashboard.putString("Neutral Mode", String.valueOf(idleMode));
    	SmartDashboard.putNumber("Left Voltage", leftDrive.getOutputVoltage());
        SmartDashboard.putNumber("Right Voltage", rightDrive.getOutputVoltage());
        SmartDashboard.putNumber("Accelerometer", getAcceleration());
        SmartDashboard.putBoolean("high Gear", mIsHighGear);
    }
  
    public void resetDriveEncoders() {
        neoOffsetL = leftDrive.getPosition();
        neoOffsetR = rightDrive.getPosition();
        leftRioEncoder.zero();
        rightRioEncoder.zero();
        ticksLeftNeoHighGear = 0;
        ticksLeftNeoLowGear = 0;
        ticksRightNeoHighGear = 0;
        ticksRightNeoLowGear = 0;
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

    public void setClosedLoopRampTime(double time) {
        leftDrive.setClosedLoopRampTime(time);
        rightDrive.setClosedLoopRampTime(time);
    }

    public void setOpenLoopRampTime(double time){ 
        leftDrive.setOpenLoopRampTime(time);
        rightDrive.setOpenLoopRampTime(time);
    }

    public boolean collision(){
        return Math.round(100* navx.getWorldLinearAccelY()) > 55 ;
    }
}