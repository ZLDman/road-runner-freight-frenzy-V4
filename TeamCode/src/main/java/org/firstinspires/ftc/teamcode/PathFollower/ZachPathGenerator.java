package org.firstinspires.ftc.teamcode.PathFollower;

import com.acmerobotics.dashboard.config.Config;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class ZachPathGenerator {

    private double endX = 0;

    public ZachPathGenerator( double end) {
        this.endX = end;
    }

    /*END*/

    public double getEndX() {
        return endX;
    }

    public void setEndX(double endX) {
        this.endX = endX;
    }
}
