package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;



public class Configs {
  public static final class RollerConfigs {

    public static final SparkFlexConfig armMotorConfig = new SparkFlexConfig();
    
    static {

      // Configure basic settings of the Algae Claw Rotation motor
      armMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .softLimit.reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(3.5)
        .forwardSoftLimitEnabled(true);

    }  

  }
}