// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SignalControl;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final SwerveAuto m_autoCommand = new SwerveAuto(swerveSubsystem, m_intake);

  public static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static SwerveModule LeftFrontSwerveModule = new SwerveModule(1, Constants.LFa, Constants.LFv, Constants.LF0, false);      //(number, angleMotorPort, velocityMotorPort, zeroPosition)
  public static SwerveModule RightFrontSwerveModule = new SwerveModule(2, Constants.RFa, Constants.RFv, Constants.RF0,true);
  public static SwerveModule RightBackSwerveModule = new SwerveModule(3, Constants.RBa, Constants.RBv, Constants.RB0, true);
  public static SwerveModule LeftBackSwerveModule = new SwerveModule(4, Constants.LBa, Constants.LBv, Constants.LB0, true);
  public static SwerveDrive swerveDrive = new SwerveDrive(swerveSubsystem);

  private static ShootingSubsystem m_intake = new ShootingSubsystem();
  public static SignalControl sc = new SignalControl(m_intake);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public static Joystick driveJoystick = new Joystick(0);
  public static Joystick ctrlJoystick = new Joystick(1);

  public static void driveRumble() {
    driveJoystick.setRumble(RumbleType.kLeftRumble, 1);
    driveJoystick.setRumble(RumbleType.kRightRumble, 1);
    Timer.delay(0.1);
    driveJoystick.setRumble(RumbleType.kLeftRumble, 0);
    driveJoystick.setRumble(RumbleType.kRightRumble, 0);
    Timer.delay(0.1);
  }

  public static void ctrlRumble() {
    ctrlJoystick.setRumble(RumbleType.kLeftRumble, 1);
    ctrlJoystick.setRumble(RumbleType.kRightRumble, 1);
    Timer.delay(0.1);
    ctrlJoystick.setRumble(RumbleType.kLeftRumble, 0);
    ctrlJoystick.setRumble(RumbleType.kRightRumble, 0);
    Timer.delay(0.1);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
