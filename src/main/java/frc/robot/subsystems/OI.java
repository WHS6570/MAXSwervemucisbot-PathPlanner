package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

public class OI {
public static  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
public static  XboxController m_shooterController = new XboxController(OIConstants.kShooterControllerPort);

}
