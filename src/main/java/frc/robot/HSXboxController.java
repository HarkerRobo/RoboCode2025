package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Represents a standard Xbox controller.
 *
 * @author Benjamin Xia
 * @version 02/23/25
 */
public class HSXboxController extends CommandXboxController {
  public HSXboxController(int port) {
    super(port);
  }

  public Trigger getUpDPad() {
    return super.povUp();
  }

  public Trigger getDownDPad() {
    return super.povDown();
  }

  public Trigger getLeftDPad() {
    return super.povLeft();
  }

  public Trigger getRightDPad() {
    return super.povRight();
  }

  public boolean getUpDPadState() {
    return super.getHID().getPOV() == 0;
  }

  public boolean getDownDPadState() {
    return super.getHID().getPOV() == 180;
  }

  public boolean getLeftDPadState() {
    return super.getHID().getPOV() == 270;
  }

  public boolean getRightDPadState() {
    return super.getHID().getPOV() == 90;
  }
}