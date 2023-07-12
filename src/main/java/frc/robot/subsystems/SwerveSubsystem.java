// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    ahrs = new AHRS(SPI.Port.kMXP);
  }

  AHRS ahrs;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  // field-oriented
  private double s2 = Math.sqrt(2);
  private double maxv = 1;
  private double[] velocityGoal = new double[8], thetaGoal = new double[8];

  public void field_oriented(double m, double f, double r, double a) {
    double sin = Math.sin(a);
    double cos = Math.cos(a);
    double m1 = m * cos + f * sin;
    double f1 = f * cos - m * sin;

    car_oriented(m1, f1, r);
  }

  public double zero(double x) {
    if (Math.abs(x) < 0.01)
      return 0;
    return x;
  }

  // car-oriented
  public void car_oriented(double m, double f, double r) {
    double k, Vm = 0;
    double v1 = Math.sqrt((m - r / s2) * (m - r / s2) + (f - r / s2) * (f - r / s2));
    double v2 = Math.sqrt((m - r / s2) * (m - r / s2) + (f + r / s2) * (f + r / s2));
    double v3 = Math.sqrt((m + r / s2) * (m + r / s2) + (f + r / s2) * (f + r / s2));
    double v4 = Math.sqrt((m + r / s2) * (m + r / s2) + (f - r / s2) * (f - r / s2));
    Vm = Math.max(Math.max(v1, v2), Math.max(v3, v4));

    if (Vm > maxv) {
      k = maxv / Vm;
      m *= k;
      f *= k;
      r *= k;
    }

    v1 = Math.sqrt((m - r / s2) * (m - r / s2) + (f - r / s2) * (f - r / s2));
    v2 = Math.sqrt((m - r / s2) * (m - r / s2) + (f + r / s2) * (f + r / s2));
    v3 = Math.sqrt((m + r / s2) * (m + r / s2) + (f + r / s2) * (f + r / s2));
    v4 = Math.sqrt((m + r / s2) * (m + r / s2) + (f - r / s2) * (f - r / s2));

    double theta1 = Math.atan2(zero(f - r / s2), zero(m - r / s2));
    double theta2 = Math.atan2(zero(f + r / s2), zero(m - r / s2));
    double theta3 = Math.atan2(zero(f + r / s2), zero(m + r / s2));
    double theta4 = Math.atan2(zero(f - r / s2), zero(m + r / s2));

    if (theta1 < 0)
      theta1 += 2 * Math.PI;
    if (theta2 < 0)
      theta2 += 2 * Math.PI;
    if (theta3 < 0)
      theta3 += 2 * Math.PI;
    if (theta4 < 0)
      theta4 += 2 * Math.PI;

    velocityGoal[1] = v1;
    velocityGoal[2] = v2;
    velocityGoal[3] = v3;
    velocityGoal[4] = v4;
    thetaGoal[1] = theta1;
    thetaGoal[2] = theta2;
    thetaGoal[3] = theta3;
    thetaGoal[4] = theta4;

    //log the data
    SmartDashboard.putNumberArray("v_goal", velocityGoal);
    SmartDashboard.putNumberArray("a_goal", thetaGoal);
  }

  public double[] get_theta() {
    return thetaGoal;
  }

  public double[] get_velocity() {
    return velocityGoal;
  }

  public double get_field_angle() {
    double angle = ahrs.getYaw();
    angle = -angle;
    angle = (angle % 360 + 360) % 360;
    return angle;
  }
}
