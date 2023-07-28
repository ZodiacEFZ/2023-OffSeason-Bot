// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {
    serializer.configFactoryDefault();
    serializer.setInverted(true);
    serializer.set(0.4);

    shooter.configFactoryDefault();
    shooter.setInverted(true);

    intakeEncoder.setSensorPhase(true);
    intakeEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }

  private WPI_TalonFX shooter = new WPI_TalonFX(Constants.spinMotorPort);
  private WPI_TalonFX serializer = new WPI_TalonFX(Constants.serializerPort);
  private WPI_TalonFX intakeArm = new WPI_TalonFX(Constants.intakeArmPort);
  private WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.intakeMotorPort);

  private WPI_TalonSRX intakeEncoder = new WPI_TalonSRX(Constants.intakeEncoderPort);

  private int upPos = 0, downPos = 0;
  public boolean downStatus = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.ctrlJoystick.getRawButtonPressed(1)) {
      down();
    } else if (RobotContainer.ctrlJoystick.getRawButtonPressed(4)) {
      up();
    }
    if (RobotContainer.ctrlJoystick.getRawButton(3)) {
      shooter.set(0.8);
    } else {
      shooter.set(0);
    }
    if (RobotContainer.ctrlJoystick.getRawButton(2)) {
      serializer.set(-0.4);
    } else {
      serializer.set(0.4);
    }
  }

  public void up() {
    intakeArm.set(ControlMode.Position, upPos);
    intakeMotor.set(0);
  }

  public void down() {
    intakeArm.set(ControlMode.Position, downPos);
    intakeArm.set(0.5);
  }
}
