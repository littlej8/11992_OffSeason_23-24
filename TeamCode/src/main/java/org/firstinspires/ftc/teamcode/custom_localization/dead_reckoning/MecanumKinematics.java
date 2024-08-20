package org.firstinspires.ftc.teamcode.custom_localization.dead_reckoning;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.custom_localization.util.Pose;
import org.firstinspires.ftc.teamcode.custom_localization.util.Vector2D;

public class MecanumKinematics {
    private final SimpleMatrix inverseKinematics;
    private final SimpleMatrix forwardKinematics;

    private final double IN_PER_TICK;

    public MecanumKinematics(double wheelRadius,
                             double ticksPerRevolution,
                             Vector2D flWheelPos,
                             Vector2D frWheelPos,
                             Vector2D blWheelPos,
                             Vector2D brWheelPos) {
        IN_PER_TICK = (2 * Math.PI * wheelRadius) / ticksPerRevolution;

        inverseKinematics = new SimpleMatrix(4, 3);

        setInverseKinematics(flWheelPos, frWheelPos, blWheelPos, brWheelPos);
        forwardKinematics = inverseKinematics.pseudoInverse();
    }

    // return change in position based on wheel positions
    public Pose toTwist(double[] start, double[] end) {
        SimpleMatrix wheelDeltasVector = new SimpleMatrix(4, 1);
        wheelDeltasVector.setColumn(
                0,
                0,
                (end[0] - start[0]) * IN_PER_TICK,
                (end[1] - start[1]) * IN_PER_TICK,
                (end[2] - start[2]) * IN_PER_TICK,
                (end[3] - start[3]) * IN_PER_TICK
        );
        SimpleMatrix twist = forwardKinematics.mult(wheelDeltasVector);
        return new Pose(twist.get(0, 0), twist.get(1, 0), twist.get(2, 0));
    }

    private void setInverseKinematics(Vector2D fl,
                                      Vector2D fr,
                                      Vector2D bl,
                                      Vector2D br) {
        inverseKinematics.setRow(0, 0, 1, -1, -(fl.x + fl.y));
        inverseKinematics.setRow(1,  0, 1, 1, fr.x - fr.y);
        inverseKinematics.setRow(2,  0, 1, 1, bl.x - bl.y);
        inverseKinematics.setRow(3, 0, 1, -1, -(br.x + br.y));
    }
}
