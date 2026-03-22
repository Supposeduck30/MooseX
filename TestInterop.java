import com.apexpathing.util.math.Vector;
import com.apexpathing.util.math.Pose;

public class TestInterop {
    public static void main(String[] args) {
        Vector v = new Vector(1.0, 2.0);
        double x = v.x();
        v.setX(3.0);

        Pose p = new Pose(1.0, 2.0, 3.0);
        double heading = p.heading();
        p.setHeading(4.0);
    }
}
