// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  TalonFX frontRight;
  TalonFX frontLeft;
  TalonFX backRight;
  TalonFX backLeft;

  public static double P;
  public static double I;
  public static double D;

  public static final double WHEEL_DIAMETER_METERS = 0.158;
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
  public static final double TRACK_WIDTH_METERS = 0.5883;
  public static final double TICKS_PER_ROTATION = 2048;
  public static final double GEAR_RATIO = 11.2444;

  double lastTime;
  double totalError;
  double lastError;

  public Drivetrain() {
    frontRight = new TalonFX(0);
    frontLeft = new TalonFX(1);
    backRight = new TalonFX(2);
    backLeft = new TalonFX(3);

    P = 0.7;
    I = 0.2;
    D = 0.1;

    lastTime = Timer.getFPGATimestamp();
    totalError = 0.0;
    lastError = 0.0;

    backRight.follow(frontRight);
    backLeft.follow(frontLeft);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double ticksToMeters(double ticks)
  {
    return ticks / GEAR_RATIO / TICKS_PER_ROTATION * WHEEL_CIRCUMFERENCE_METERS;
  }

  public double getMotorSpeed(double target)
  {
    double error = target - ticksToMeters(frontRight.getSelectedSensorPosition());
    double proportional = P*error;
    double dt = Timer.getFPGATimestamp() - lastTime;
    double de = error-lastError;
    double errorRate = de/dt;

    if(Math.abs(error) < 1)
    {
      totalError += error*dt;
    }

    double output = proportional + I*totalError + D*errorRate;
    lastError = error;
    lastTime = Timer.getFPGATimestamp();

    return output;
  }

  public void stopMotors()
  {
    frontRight.set(ControlMode.PercentOutput, 0.0);
    frontLeft.set(ControlMode.PercentOutput, 0.0);
  }

  public void startMotors(double speed)
  {
    frontLeft.set(ControlMode.PercentOutput, speed);
    frontRight.set(ControlMode.PercentOutput, speed);
  }
}

