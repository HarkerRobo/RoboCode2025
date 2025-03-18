
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Elevator extends SubsystemBase 
{
    private static Elevator instance;

    private TalonFX master;
    private TalonFX follower;

    private DigitalInput limitSwitch;

    private SysIdRoutine sysIdRoutine;

    private double desiredPosition;

    private Elevator() 
    {
        master = new TalonFX(Constants.Elevator.MASTER_ID, Constants.CAN_CHAIN);

        follower = new TalonFX(Constants.Elevator.FOLLOWER_ID, Constants.CAN_CHAIN);

        config();

        limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_ID);

        desiredPosition = 0.0;

    }

    private final SysIdRoutine _sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2.0).per(Second), // Use default ramp rate (1 V/s)
              Volts.of(2.0), // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> Elevator.getInstance().setVoltage(output.in(Volts)), null, this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return _sysId.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return _sysId.dynamic(direction);
    }

    private void config() 
    {
        master.clearStickyFaults();
        follower.clearStickyFaults();

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.Inverted = Constants.Elevator.MASTER_INVERTED;

        masterConfig.Feedback.SensorToMechanismRatio = Constants.Elevator.ELEVATOR_GEAR_RATIO;

        masterConfig.Slot0.kP = Constants.Elevator.kP;
        masterConfig.Slot0.kI = Constants.Elevator.kI;
        masterConfig.Slot0.kD = Constants.Elevator.kD;

        masterConfig.Slot0.kV = Constants.Elevator.kV;
        masterConfig.Slot0.kG = Constants.Elevator.kG;

        masterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MM_CRUISE_VELOCITY;
        masterConfig.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MM_ACCELERATION;
        masterConfig.MotionMagic.MotionMagicJerk = Constants.Elevator.MM_JERK;

        masterConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        masterConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        masterConfig.CurrentLimits.StatorCurrentLimit = Constants.Elevator.STATOR_CURRENT_LIMIT;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        masterConfig.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.SUPPLY_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Elevator.FORWARD_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Elevator.REVERSE_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        master.getConfigurator().apply(masterConfig);

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        followerConfig.MotorOutput.Inverted = Constants.Elevator.FOLLOWER_INVERTED;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        follower.getConfigurator().apply(followerConfig);

        follower.setControl(new Follower(Constants.Elevator.MASTER_ID, false));
    }

    /**
     * @param desired rotations
     */
    public boolean checkExtend(double desired) 
    {
        return Math.abs(desired - getPosition()) < Constants.Elevator.MAX_ERROR;
    }

    /**
     * @return rotations
     */
    public double getPosition() 
    {
        return master.getPosition().getValueAsDouble();
    }
    
    /**
     * @return rotations per second
     */
    public double getVelocity()
    {
        return master.getVelocity().getValueAsDouble();
    }

    /**
     * @param power range [-1, 1]
     */
    public void setElevatorPower(double power) 
    {
        master.setControl(new DutyCycleOut(power));
    }

    public void setVoltage(double power)
    {
        master.setControl(new VoltageOut(power));
    }

    public void setSensorPosition(double position)
    {
        master.getConfigurator().setPosition(position);
    }

    public void moveToPosition(double desiredPosition)
    {
        this.desiredPosition = desiredPosition;
        master.setControl(new MotionMagicVoltage(desiredPosition));
    }

    public double getDesiredPosition()
    {
        return desiredPosition;
    }

    public void setDesiredPosition(double position)
    {
        desiredPosition = position;
    }

    public boolean isLimitHit() 
    {
        return !limitSwitch.get();
    }

    public boolean isStalling()
    {
        return master.getStatorCurrent().getValueAsDouble() >= Constants.Elevator.ELEVATOR_STALLING_CURRENT;
    }

    public boolean isExtended()
    {
        return getPosition() > Constants.Elevator.ELEVATOR_EXTENDED_HIGHT;
    }

    public static Elevator getInstance() 
    {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
}