
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climb extends SubsystemBase 
{
    private static Climb instance;

    private TalonFX master;

    private Climb() 
    {
        master = new TalonFX(RobotMap.Climb.ID, RobotMap.CAN_CHAIN);

        config();
    }

    private void config() 
    {
        master.clearStickyFaults();

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.Inverted = RobotMap.Climb.INVERTED;

        masterConfig.Feedback.SensorToMechanismRatio = RobotMap.Climb.CLIMB_GEAR_RATIO;

        masterConfig.Slot0.kP = RobotMap.Climb.kP;

        masterConfig.Voltage.PeakForwardVoltage = RobotMap.MAX_VOLTAGE;
        masterConfig.Voltage.PeakReverseVoltage = -RobotMap.MAX_VOLTAGE;

        masterConfig.CurrentLimits.StatorCurrentLimit = RobotMap.Climb.STATOR_CURRENT_LIMIT;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        masterConfig.CurrentLimits.SupplyCurrentLimit = RobotMap.Climb.SUPPLY_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        master.getConfigurator().apply(masterConfig);
    }

    /**
     * @return rotations
     */
    public double getPosition() 
    {
        return master.getPosition().getValueAsDouble();
    }

    public void moveToPosition(double desired)
    {
        master.setControl(new PositionVoltage(desired));
    }

    /**
     * 
     * @param desired [-1, 1] of voltage
     */
    public void setSpeed (double desired)
    {
        master.setControl(new DutyCycleOut(desired));
    }

    public void zeroClimb() 
    {
        master.getConfigurator().setPosition(0);
    }

    public static Climb getInstance() 
    {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }
}
