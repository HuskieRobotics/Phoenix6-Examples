package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class SwerveRotationMechanism extends SubsystemBase {
    private final TalonFX[] m_motorsToTest = new TalonFX[4];
    private final DutyCycleOut[] m_joystickControls = new DutyCycleOut[4];
    private final VoltageOut[] m_sysidControls = new VoltageOut[4];

    private static final double MK4I_L3_ANGLE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));

    private SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                null, // Reduce dynamic voltage to 4 to prevent motor brownout
                null,          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> { 
                    for(int i = 0; i < 4; ++i) {
                        m_motorsToTest[i].setControl(m_sysidControls[i].withOutput(volts.in(Volts)));
                    }
                },
                null,
                this));

    public SwerveRotationMechanism() {
        setName("Swerve Rotation");

        m_motorsToTest[0] = new TalonFX(Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.CANBUS);
        m_motorsToTest[1] = new TalonFX(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.CANBUS);
        m_motorsToTest[2] = new TalonFX(Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.CANBUS);
        m_motorsToTest[3] = new TalonFX(Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.CANBUS);

        for (int i = 0; i < 4; ++i) {
            m_joystickControls[i] = new DutyCycleOut(0);
            m_sysidControls[i] = new VoltageOut(0);
        }

        for (int i = 0; i < 4; ++i) {
            TalonFXConfiguration cfg = new TalonFXConfiguration();
            cfg.Feedback.SensorToMechanismRatio = MK4I_L3_ANGLE_GEAR_RATIO;
            m_motorsToTest[i].getConfigurator().apply(cfg);

            /* Speed up signals for better charaterization data */
            BaseStatusSignal.setUpdateFrequencyForAll(250,
                m_motorsToTest[i].getPosition(),
                m_motorsToTest[i].getVelocity(),
                m_motorsToTest[i].getMotorVoltage());

            /* Optimize out the other signals, since they're not particularly helpful for us */
            m_motorsToTest[i].optimizeBusUtilization();
        }

        SignalLogger.start();
    }

    public Command joystickDriveCommand(DoubleSupplier output) {
        return run(()-> {
            for(int i = 0; i < 4; ++i) {
                m_motorsToTest[i].setControl(m_joystickControls[i].withOutput(output.getAsDouble()));
            }
        });
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }
}
