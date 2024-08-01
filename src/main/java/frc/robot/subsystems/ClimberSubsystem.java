package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax m_climbingmotor = new CANSparkMax(31, MotorType.kBrushless);
  Servo ratchetServo = new Servo(0);
  //private final RelativeEncoder m_climbingEncoder;
  RelativeEncoder m_climbingEncoder = m_climbingmotor.getEncoder();
  boolean lockstatus = true;


  public ClimberSubsystem() {
  m_climbingmotor.setSoftLimit(SoftLimitDirection.kReverse,-319);
  m_climbingmotor.setSoftLimit(SoftLimitDirection.kForward,-5);
  m_climbingmotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  m_climbingmotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  ratchetServo.setDisabled();


  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void climb(double climbspeed) {
    m_climbingmotor.set(-climbspeed*Math.abs(climbspeed));
    SmartDashboard.putNumber("climbpos", m_climbingEncoder.getPosition());

  }
  public boolean isLocked() {
    return lockstatus;

  }
public void unlock(){
  ratchetServo.set(1);
 lockstatus = false;
}
public void lock(){
  ratchetServo.setDisabled();
  lockstatus = true;
}
public double getPos() {
return m_climbingEncoder.getPosition();
}
public void report() {
SmartDashboard.putNumber("climbpos", m_climbingEncoder.getPosition());
 
}
}