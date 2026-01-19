package org.firstinspires.ftc.teamcode;
//1
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AS Teleop RED v.01", group = "Teleop")
public class pineapplered extends PineapplesBOT {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}