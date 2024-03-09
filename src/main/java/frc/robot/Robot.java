// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.config.CTREConfigs;
import frc.robot.RobotContainer.SimpleDirection;
import frc.team_8840_lib.info.console.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  RobotContainer container;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("DEFAULT", "DEFAULT");
    m_chooser.addOption("SHOOT_AND_STRAIGHT", "SHOOT_AND_STRAIGHT");
    m_chooser.addOption("SHOOT_AND_LEFT", "SHOOT_AND_LEFT");
    m_chooser.addOption("SHOOT_AND_RIGHT", "SHOOT_AND_RIGHT");
    m_chooser.addOption("SHOOT_TWICE", "SHOOT_TWICE");
    SmartDashboard.putData("Autonomous selection", m_chooser);
    ctreConfigs = new CTREConfigs();
    container = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    String m_autoSelected = m_chooser.getSelected();
    Logger.Log("We got selection: " + m_autoSelected);

    Command autonomousCommand = getCommand(m_autoSelected);

    // schedule the autonomous command - adds it to the scheduler
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

  }

  private Command getCommand(String s) {
    switch (s) {
      case "COMPLEX":
        return container.getA2BlueCommand();
      case "SHOOT_AND_LEFT":
        return container.shootAndDriveCommand(SimpleDirection.diagonalLeft);
      case "SHOOT_AND_RIGHT":
        return container.shootAndDriveCommand(SimpleDirection.diagonalRight);
      case "SHOOT_TWICE":
        return container.shootAndDriveAndShootAgainCommand(SimpleDirection.straight);
      default:
        return container.shootAndDriveCommand(SimpleDirection.straight);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
