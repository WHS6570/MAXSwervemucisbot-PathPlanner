package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  CANSparkMax m_engulfmotor = new CANSparkMax(45, MotorType.kBrushless);
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private Rev2mDistanceSensor distOnboard; 
  //private Rev2mDistanceSensor distMXP;
  public IntakeSubsystem() {
    distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    distOnboard.setAutomaticMode(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  @Override
  public void periodic() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  double target = tv.getDouble(2.0);
  if (seeIR()<=6) {
    LED.blinkin.set(0.63); //0.27 is flashin gold
  } else   if (target == 1) {
    LED.blinkin.set(0.77);
    } else {
    LED.blinkin.set(0.41);}
  }
 public void engulf(double speed) {
    m_engulfmotor.set(speed);
  }
public void stop() {
m_engulfmotor.set(0);
SmartDashboard.putNumber("IR", seeIR());
}
public double seeIR() {
  double IR = distOnboard.getRange();
  return IR;
  }
}