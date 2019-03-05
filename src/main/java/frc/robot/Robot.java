/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.DriveTrainCharacterizer.Direction;
import frc.robot.DriveTrainCharacterizer.TestMode;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static RobotMap map = new RobotMap();
  public static Drive drive = new Drive(map.leftDrive, map.rightDrive, map.navx, map.shifter, map.leftDriveEncoder, map.rightDriveEncoder);
  public static HumanInput HI = new HumanInput();


  public DriveTrainCharacterizer characterizer = new DriveTrainCharacterizer();

  boolean characterizerRunning = false, lastStart = false, lastStop = false;
  
  public Runnable smartDashboardRunnable = new Runnable(){
  
    @Override
    public void run() {
      outputToSmartDashboard();
    }
  };
  public Notifier smartDashboardNotifier = new Notifier(smartDashboardRunnable);

  @Override
  public void robotInit() {
    smartDashboardNotifier.startPeriodic(.02);
    drive.resetSensors();
  }

  @Override
  public void disabledInit() {
    drive.resetDriveEncoders();
    characterizerRunning = false;
  }


  @Override
  public void disabledPeriodic() {
    allPeriodic();
  }

  @Override
  public void robotPeriodic() {
  }
  @Override
  public void autonomousInit() {
    drive.resetSensors();
  }

  @Override
  public void autonomousPeriodic() {
    allPeriodic();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    allPeriodic();
    if(HI.getQuasiStaticForward())
      characterizer.setMode(TestMode.QUASI_STATIC, Direction.Forward);
    else if(HI.getVoltageStepForward()) 
      characterizer.setMode(TestMode.STEP_VOLTAGE, Direction.Forward);
    else if(HI.getQuasiStaticBack())
      characterizer.setMode(TestMode.QUASI_STATIC, Direction.Backward);
    else if(HI.getVoltageStepBack())
      characterizer.setMode(TestMode.STEP_VOLTAGE, Direction.Backward);

    // Drive Controls
    if(HI.getStart() && !lastStart) {
      characterizerRunning = true;
      characterizer.initialize();
    }
    else if(HI.getStop() && !lastStop) {
      characterizerRunning = false;
      drive.setRampRate(0);
    }
    if(characterizerRunning) {
      characterizer.run();
    }
    else {
      if(HI.getGyrolock()) {
        drive.setDriveMode(DriveMode.GYROLOCK);
        drive.setTank(HI.getLeftThrottle(), HI.getLeftThrottle(), 2);
      }
      else {  
        drive.setDriveMode(DriveMode.OPEN_LOOP);
        drive.setTank(HI.getLeftThrottle(), HI.getRightThrottle(), 2);
      }
      if(HI.getHighGear()) {
        drive.setHighGear(true);
      }
      else if(HI.getLowGear()) {
        drive.setHighGear(false);
      }
    }
    lastStart = HI.getStart();
    lastStop = HI.getStop();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void outputToSmartDashboard() {
    drive.outputToSmartDashboard();
    characterizer.outputToSmartDashboard();
  }

  public void allPeriodic() {
    drive.update();

  }
}
