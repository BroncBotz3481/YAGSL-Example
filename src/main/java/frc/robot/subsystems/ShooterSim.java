// package frc.robot.subsystems;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;

// public class ShooterSim {
//   private DCMotor m_armGearbox;
//   private SingleJointedArmSim armSim;
//   private DutyCycleEncoderSim m_encoderSim;
//   // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
//   private Mechanism2d m_mech2d;
//   private MechanismRoot2d m_armPivot;
//   private MechanismLigament2d m_armTower;
//   private MechanismLigament2d m_arm;
//   private DutyCycleEncoder absoluteEncoder;

//   public ShooterSim(DutyCycleEncoder absoluteEncoder) {
//     this.absoluteEncoder = absoluteEncoder;
//     createSimulation();
//   }

//   private void createSimulation() {
//     m_armGearbox = DCMotor.getFalcon500(1);
//     armSim =
//         new SingleJointedArmSim(
//             m_armGearbox,
//             16,
//             0.731599134,
//             0.6858,
//             -Math.PI / 2,
//             3 * Math.PI / 2,
//             true,
//             VecBuilder.fill(1 / 256));
//     m_encoderSim = new DutyCycleEncoderSim(absoluteEncoder);
//     // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
//     m_mech2d = new Mechanism2d(60, 60);
//     m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
//     m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
//     m_arm =
//         m_armTower.append(
//             new MechanismLigament2d(
//                 "Arm",
//                 30,
//                 Units.radiansToDegrees(armSim.getAngleRads()),
//                 6,
//                 new Color8Bit(Color.kYellow)));
//   }

//   public void update(double motorPercent) {
//     // In this method, we update our simulation of what our arm is doing
//     // First, we set our "inputs" (voltages)
//     armSim.setInput(motorPercent * 12);

//     // Next, we update it. The standard loop time is 20ms.
//     armSim.update(0.020);

//     // Finally, we set our simulated encoder's readings and simulated battery voltage
//     m_encoderSim.setDistance(armSim.getAngleRads());

//     // Update the Mechanism Arm angle based on the simulated arm angle
//     m_arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
//   }

//   public Mechanism2d getMech2d() {
//     return m_mech2d;
//   }
// }

// https://github.com/FRC1466/robot-code-2023/blob/main/src/main/java/frc/robot/subsystems/manipulator/VirtualFourBar.java

