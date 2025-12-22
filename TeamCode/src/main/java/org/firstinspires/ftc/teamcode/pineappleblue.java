package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AS Teleop BLUE", group = "Teleop")
public class pineappleblue extends PineapplesBOT {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}