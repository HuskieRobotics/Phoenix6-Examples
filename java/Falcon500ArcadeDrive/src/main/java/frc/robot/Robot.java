// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final String CANBUS_NAME = "rio";
  private final TalonFX leftLeader = new TalonFX(1, CANBUS_NAME);
  private final TalonFX leftFollower = new TalonFX(2, CANBUS_NAME);
  private final TalonFX rightLeader = new TalonFX(3, CANBUS_NAME);
  private final TalonFX rightFollower = new TalonFX(4, CANBUS_NAME);

  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  private final XboxController joystick = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Configure the devices */
    var appliedConfiguration = new TalonFXConfiguration();

    /* User can optionally chagne the configs or leave it alone to perform a factory default */

    leftLeader.getConfigurator().apply(appliedConfiguration);
    leftFollower.getConfigurator().apply(appliedConfiguration);
    rightLeader.getConfigurator().apply(appliedConfiguration);
    rightFollower.getConfigurator().apply(appliedConfiguration);

    /* Set up followers to follow leaders */
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    /* Get forward and rotational throttle from joystick */
    double fwd = joystick.getLeftY();
    double rot = joystick.getRightX();
    /* Set output to control frames */
    leftOut.output = fwd + rot;
    rightOut.output = fwd - rot;
    /* And set them to the motors */
    leftLeader.setControl(leftOut);
    rightLeader.setControl(rightOut);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    /* Zero out controls so we aren't just relying on the enable frame */
    leftOut.output = 0;
    rightOut.output = 0;
    leftLeader.setControl(leftOut);
    rightLeader.setControl(rightOut);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
