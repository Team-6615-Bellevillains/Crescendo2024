package frc.robot.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class TunableProfiledPIDController {

    private final static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final static NetworkTable tuningTable = networkTableInstance.getTable("tuning");
    private final static double updateInterval = 3;
    private final static ArrayList<TunableProfiledPIDController> controllers = new ArrayList<>();

    private final Field m_constraintsField;
    private final String identifier;

    private final DoubleSupplier measurementSupplier;

    private double lastUpdatedTS = Timer.getFPGATimestamp();
    private double lastMeasurement;

    private ProfiledPIDController profiledPIDController;

    public TunableProfiledPIDController(String identifier, double Kp, double Ki, double Kd,
            TrapezoidProfile.Constraints constraints, DoubleSupplier measurementSupplier) {
        profiledPIDController = new ProfiledPIDController(Kp, Ki, Kd, constraints);

        this.identifier = identifier;

        this.measurementSupplier = measurementSupplier;

        tuningTable.getEntry(appendIdentifier("Kp")).setDouble(Kp);
        tuningTable.getEntry(appendIdentifier("Ki")).setDouble(Ki);
        tuningTable.getEntry(appendIdentifier("Kd")).setDouble(Kd);
        tuningTable.getEntry(appendIdentifier("kMaxVelo")).setDouble(constraints.maxVelocity);
        tuningTable.getEntry(appendIdentifier("kMaxAccel")).setDouble(constraints.maxAcceleration);

        try {
            m_constraintsField = ProfiledPIDController.class.getDeclaredField("m_constraints");
            m_constraintsField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        controllers.add(this);
    }

    public ProfiledPIDController getController() {
        return profiledPIDController;
    }

    public static void updateControllersIfOutdated() {
        for (TunableProfiledPIDController controller : controllers) {
            controller.updateConstantsIfOutdated();
        }
    }

    public double calculateAndUpdateLastMeasurement(double currentMeasurement) {
        lastMeasurement = currentMeasurement;
        return profiledPIDController.calculate(currentMeasurement);
    }

    public double calculateAndUpdateLastMeasurement(double currentMeasurement, double goal) {
        lastMeasurement = currentMeasurement;
        return profiledPIDController.calculate(currentMeasurement, goal);
    }

    public String appendIdentifier(String input) {
        return "ppid" + input + identifier;
    }

    public void updateConstantsIfOutdated() {
        if (Timer.getFPGATimestamp() - lastUpdatedTS < updateInterval)
            return;

        double tableKP = tuningTable.getValue(appendIdentifier("Kp")).getDouble();
        double tableKI = tuningTable.getValue(appendIdentifier("Ki")).getDouble();
        double tableKD = tuningTable.getValue(appendIdentifier("Kd")).getDouble();
        double tableMaxVelocity = tuningTable.getValue(appendIdentifier("kMaxVelo")).getDouble();
        double tableMaxAcceleration = tuningTable.getValue(appendIdentifier("kMaxAccel")).getDouble();

        TrapezoidProfile.Constraints currConstraints = getConstraints();

        if (profiledPIDController.getP() != tableKP
                || profiledPIDController.getI() != tableKI
                || profiledPIDController.getD() != tableKD
                || currConstraints.maxVelocity != tableMaxVelocity
                || currConstraints.maxAcceleration != tableMaxAcceleration) {
            profiledPIDController.setP(tableKP);
            profiledPIDController.setI(tableKI);
            profiledPIDController.setD(tableKD);
            profiledPIDController
                    .setConstraints(new TrapezoidProfile.Constraints(tableMaxVelocity, tableMaxAcceleration));
            profiledPIDController.reset(measurementSupplier.getAsDouble());
        }

        lastUpdatedTS = Timer.getFPGATimestamp();
    }

    public TrapezoidProfile.Constraints getConstraints() {
        try {
            return (TrapezoidProfile.Constraints) m_constraintsField.get(profiledPIDController);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }
}
