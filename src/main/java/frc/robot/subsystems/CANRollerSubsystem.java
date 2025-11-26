// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Configs;

/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {
  private final SparkMax rollerMotor;
  private final SparkFlex armMotor;
  private final RelativeEncoder armEncoder;

  public CANRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    armMotor = new SparkFlex(RollerConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    rollerMotor.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armMotor.configure(Configs.RollerConfigs.armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public Command c_rollerIntakeCommand() {
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          rollerMotor.set(0.4);
        },
        // When the command stops, stop the wheels
        () -> {
          f_rollerStop();
        });
  }

  public Command c_rollerReverseIntakeCommand() {
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          rollerMotor.set(-0.5);
        },
        // When the command stops, stop the wheels
        () -> {
          f_rollerStop();
        });
  }

  public Command c_rotateArmUpCommand() {
      return this.startEnd(
          // When the command is initialized, set the wheels to the intake speed values
          () -> {
            armMotor.set(0.1);
          },
          // When the command stops, stop the wheels
          () -> {
            f_armStop();
          });
  }

  public Command c_rotateArmDownCommand() {
      return this.startEnd(
          // When the command is initialized, set the wheels to the intake speed values
          () -> {
            armMotor.set(-0.1);
          },
          // When the command stops, stop the wheels
          () -> {
            f_armStop();
          });
  }

  public void f_armStop(){
    armMotor.set(0);
  }

  public void f_rollerStop(){
    rollerMotor.set(0);
  }
}