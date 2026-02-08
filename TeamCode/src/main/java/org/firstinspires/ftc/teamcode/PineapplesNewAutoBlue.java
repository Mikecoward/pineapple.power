package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PineappleBot New Auto BLUE", group = "Autonomous")
public class PineapplesNewAutoBlue extends PineapplesBaseAuto {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}