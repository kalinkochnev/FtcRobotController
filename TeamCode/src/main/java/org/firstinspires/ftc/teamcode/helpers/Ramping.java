package org.firstinspires.ftc.teamcode.helpers;

import java.util.Arrays;

public class Ramping {
    public void applyRamp() {
       angleHold = Math.toRadians(angleHold);

       double targetTicks = Motors.distanceToEncoderTicks(distance); // Distance in ticks to the end goal

       double rampKickIn = 0.25; // Ramping kicks in at the first quarter of the drive and last quarter
       double rampOffset = 0.02;
       double rampupTicks = rampKickIn * targetTicks;

       double[] maxMotorPowers = this.motors.translationWeights(angleHold, maxPower);
       double[] currMotorPowers = new double[]{0, 0, 0, 0};

       int ticksTraveled = 0;

       this.motors.resetEncoders();
       while (ticksTraveled < targetTicks && !this.opMode.isStopRequested()) {
           ticksTraveled = (int) (Math.round(this.motors.getNetPositionVector().getMagnitude() * 100) / 100);


           double ticksTillTarget = targetTicks - ticksTraveled;
           telemetry.addData("Ticks till target: ", ticksTillTarget);
           // IF ramping
           if (ticksTraveled <= rampupTicks) { // If in the beginning percent of the movement to do start ramping
               telemetry.addData("Ramp status: ", "Up");
               for (int motor = 0; motor < 4; motor++) {
                   // The motor power scales proportionally to the distance already travelled
                   // So the further it travels, the faster it ramps up (to anyone who is interested, the ticks travelled
                   // grows exponentially/continuously during this time... yay calculus)
                   currMotorPowers[motor] = maxMotorPowers[motor] * (rampOffset + (ticksTraveled / rampupTicks));
               }
           } else if (ticksTillTarget <= rampupTicks) { // If in the last percent of the movement to do the end ramping
               telemetry.addData("Ramp status: ", "Down");
               for (int motor = 0; motor < 4; motor++) {
                   // Ramps down speed exponentially
                   currMotorPowers[motor] = maxMotorPowers[motor] - rampOffset * (-rampOffset + (ticksTillTarget / rampupTicks));
               }
           } else {
               telemetry.addData("Ramp status: ", "None");
               currMotorPowers = maxMotorPowers.clone();
           }
           telemetry.addData("Motor powers: ", Arrays.toString(currMotorPowers));
           telemetry.update();


           this.motors.setPowers(currMotorPowers);
       }
       motors.setPowers(0);

    }
}
