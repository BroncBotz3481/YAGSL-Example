package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem {
    SparkMax spark_max = new SparkMax(9, MotorType.kBrushed);

    public void testFunction() { // make motor stop
        spark_max.stopMotor();
    }

    public void testFunction2() { // make motor go at full speed
        spark_max.set(1.0);
    }

    public void testFunction3() { // this is a great idea
        spark_max.setVoltage(100.0);
    }
}
