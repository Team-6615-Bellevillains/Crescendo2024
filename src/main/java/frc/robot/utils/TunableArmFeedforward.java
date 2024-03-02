package frc.robot.utils;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class TunableArmFeedforward {

    private final static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final static NetworkTable tuningTable = networkTableInstance.getTable("tuning");
    private final static double updateInterval = 3;
    private final static ArrayList<TunableArmFeedforward> feedforwards = new ArrayList<>();

    private ArmFeedforward armFeedforward;
    private String identifier;

    private double lastUpdatedTS = Timer.getFPGATimestamp();

    public TunableArmFeedforward(String identifier, double ks, double kg, double kv) {
        this(identifier, ks, kg, kv, 0);
    }

    public TunableArmFeedforward(String identifier, double ks, double kg, double kv, double ka) {
        armFeedforward = new ArmFeedforward(ks, kg, kv, ka);

        this.identifier = identifier;

        tuningTable.getEntry(appendIdentifier("ks")).setDouble(ks);
        tuningTable.getEntry(appendIdentifier("kg")).setDouble(kg);
        tuningTable.getEntry(appendIdentifier("kv")).setDouble(kv);
        tuningTable.getEntry(appendIdentifier("ka")).setDouble(ka);

        feedforwards.add(this);
    }

    public ArmFeedforward getController() {
        return armFeedforward;
    }

    public String appendIdentifier(String input) {
        return "armFF" + input + identifier;
    }

    public void updateConstantsIfOutdated() {
        if (Timer.getFPGATimestamp() - lastUpdatedTS < updateInterval)
            return;

        double tableKS = tuningTable.getValue(appendIdentifier("ks")).getDouble();
        double tableKG = tuningTable.getValue(appendIdentifier("kg")).getDouble();
        double tableKV = tuningTable.getValue(appendIdentifier("kv")).getDouble();
        double tableKA = tuningTable.getValue(appendIdentifier("ka")).getDouble();

        if (armFeedforward.ks != tableKS
                || armFeedforward.kg != tableKG
                || armFeedforward.kv != tableKV
                || armFeedforward.ka != tableKA) {
            SmartDashboard.putNumber(String.format("Last update to %s", identifier), Timer.getFPGATimestamp());

            armFeedforward = new ArmFeedforward(tableKS, tableKG, tableKV, tableKA);
        }

        lastUpdatedTS = Timer.getFPGATimestamp();
    }

    public static void updateControllersIfOutdated() {
        for (TunableArmFeedforward feedforward : feedforwards) {
            feedforward.updateConstantsIfOutdated();
        }
    }

}
