package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public abstract class PineapplesAuto extends OpMode {

    public enum Alliance { BLUE, RED }
    protected abstract Alliance getAlliance();

    private static final double FIELD_SIZE_IN = 144.0;

    protected Pose mirrorBlueToRed(Pose bluePose) {
        double x = FIELD_SIZE_IN - bluePose.getX();
        double y = bluePose.getY();
        double h = AngleUnit.normalizeRadians(Math.PI - bluePose.getHeading());
        return new Pose(x, y, h);
    }

    protected Follower follower;
    protected PathChain[] pathChains;
    protected Pose[] poseArray;

    public int state = 0;
    private static final double SPEED_FACTOR = 0.15; // slower than before
    private static final long STATE_DELAY_MS = 800; // 0.8s delay after each path
    private long stateFinishedTime = 0;

    protected static final Pose[] poseArrayBlue = {
            new Pose(21, 125, Math.toRadians(145)),
            new Pose(58, 85 , Math.toRadians(135)),
            new Pose(24, 120 , Math.toRadians(-90)),
            new Pose(24, 85 , Math.toRadians(-90)),
            new Pose(58, 85 , Math.toRadians(135)),
            new Pose(24, 120 , Math.toRadians(-90)),
            new Pose(24, 60 , Math.toRadians(-90)),
            new Pose(58, 85 , Math.toRadians(135)),
            new Pose(24, 120 , Math.toRadians(-90)),
            new Pose(24, 40 , Math.toRadians(-90)),
            new Pose(0, 0, 0),
            new Pose(0, 0, 0),
    };

    @Override
    public void init() {
        Alliance alliance = getAlliance();
        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = (alliance == Alliance.BLUE) ? poseArrayBlue[i] : mirrorBlueToRed(poseArrayBlue[i]);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseArray[0]);
        follower.update();

        buildPaths();
    }

    @Override
    public void start() {
        state = 0;
        stateFinishedTime = 0;
    }

    @Override
    public void loop() {
        follower.update();

        if (state < pathChains.length) {
            if (!follower.isBusy() && System.currentTimeMillis() - stateFinishedTime >= STATE_DELAY_MS) {
                follower.followPath(pathChains[state], true);
                telemetry.addData("Moving to path", state);
                telemetry.update();
                stateFinishedTime = System.currentTimeMillis();
                state++;
            } else {
                telemetry.addData("Waiting for path", state);
                telemetry.update();
            }
        } else {
            telemetry.addData("Auto", "Finished all paths");
            telemetry.update();
        }
    }

    protected void buildPaths() {
        pathChains = new PathChain[9];

        pathChains[0] = simplePathChain(0, 1, SPEED_FACTOR);
        pathChains[1] = simplePathChain(1, 2, SPEED_FACTOR);
        pathChains[2] = simplePathChain(2, 3, SPEED_FACTOR);
        pathChains[3] = simplePathChain(3, 4, SPEED_FACTOR);
        pathChains[4] = simplePathChain(4, 5, SPEED_FACTOR);
        pathChains[5] = simplePathChain(5, 6, SPEED_FACTOR);
        pathChains[6] = simplePathChain(6, 7, SPEED_FACTOR);
        pathChains[7] = simplePathChain(7, 8, SPEED_FACTOR);
        pathChains[8] = simplePathChain(8, 9, SPEED_FACTOR);


    }

    protected PathChain simplePathChain(int start, int end, double headingTime) {
        return follower.pathBuilder()
                .addPath(new BezierLine(poseArray[start], poseArray[end]))
                .setLinearHeadingInterpolation(
                        poseArray[start].getHeading(),
                        poseArray[end].getHeading(),
                        headingTime
                )
                .build();
    }
}


