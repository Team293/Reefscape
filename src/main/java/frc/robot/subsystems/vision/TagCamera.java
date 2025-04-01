package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.function.Consumer;

public interface TagCamera {

    class VisionData {
        public Pose3d pose;
        public double timestamp;
        public Matrix<N3, N1> stdv;

        public VisionData() {
            pose = new Pose3d();
            timestamp = 0;
            stdv = VecBuilder.fill(0, 0, 0);
        }
    }

    public String getName();

    public void refreshEstimate(Consumer<VisionData> data);
}
