package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class goBack {
    private Follower follower;
    private Path goToZero;

    public goBack(Follower followerInstance) {
        follower = followerInstance; // use the existing robot follower
    }

    public void setTargetToCurrent() {
        Pose current = follower.getPose();
        Pose end = new Pose(0, 0);
        goToZero = new Path(new BezierLine(current, end));
        follower.followPath(goToZero);
    }

    public void update() {
        follower.update();
    }

    public boolean arrived() {
        return follower.atParametricEnd();
    }
}
