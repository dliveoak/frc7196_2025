package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {

    private  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private  NetworkTableEntry tx = table.getEntry("tx");
    private  NetworkTableEntry ty = table.getEntry("ty");
    private  NetworkTableEntry ta = table.getEntry("ta");
    private  NetworkTableEntry tv = table.getEntry("tv");
    private  NetworkTableEntry tid = table.getEntry("tid");

    private  NetworkTable tableRight = NetworkTableInstance.getDefault().getTable("limelight-right");
    private  NetworkTableEntry txRight = tableRight.getEntry("tx");
    private  NetworkTableEntry tyRight = tableRight.getEntry("ty");
    private  NetworkTableEntry taRight = tableRight.getEntry("ta");
    private  NetworkTableEntry tvRight = tableRight.getEntry("tv");
    private  NetworkTableEntry tidRight = tableRight.getEntry("tid");


    public double[] getXYA()
    {
        double x = tx.getDouble(0.0);
        double y  = ty.getDouble(0.0);
        double a = ta.getDouble(0.0);
        return new double[]{x, y, a};
    }

    public double[] getXYARight()
    {
        double x = txRight.getDouble(0.0);
        double y  = tyRight.getDouble(0.0);
        double a = taRight.getDouble(0.0);
        return new double[]{x, y, a};
    }

    // Get the ID of the current target, with priority going to the left LL. If neither LL sees a target, return 0 (default ID)
    public int getID()
    {
        boolean validTargetLeft = tv.getInteger(0) == 1;
        boolean validTargetRight = tvRight.getInteger(0) == 1;

        if (validTargetLeft)
        {
            return (int) tid.getInteger(0); // left LL has a target, return its id
        }
        else if (validTargetRight)
        {
            return (int) tidRight.getInteger(0); // right LL has a target, return its id
        }
        return 0; // no valid target
    }

    // calculate target angle based on tag ID.
    // TODO: populate switch-case statement with angles from field
    public Double calcTargetAngle(int tagId)
    {
        switch (tagId)
        {
            case 6: // red reef
                return 120.0;
            case 7: 
                return 180.0;
            case 8: 
                return 240.0;
            case 9: 
                return 300.0;  
            case 10: 
                return 0.0; 
            case 11:
                return 60.0; 
            case 17: // blue reef
                return 120.0;
            case 18: 
                return 180.0;
            case 19: 
                return 240.0;
            case 20: 
                return 300.0;  
            case 21: 
                return 0.0; 
            case 22:
                return 60.0; 
            default:
                return null; // default value
        }
    }
}

