package com.stuypulse.robot.commands.amper;

import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.subsystems.amper.Amper;

public class AmperOuttakeDistance extends Command {
    
    private final Amper amper;
    private final double distance;

    private double startingDistance;

    public AmperOuttakeDistance(double distance){
        amper = Amper.getInstance();
        addRequirements(amper);
        this.distance = distance;
    }

    
    @Override
    public void initialize(){
        startingDistance = amper.getNoteDistance();
    }

    @Override
    public void execute(){
        amper.score();
    }

    @Override
    public void end(boolean interrupted){
        amper.stopRoller();
    }

    @Override
    public boolean isFinished(){
        return amper.getNoteDistance() >= distance + startingDistance;
    }
    //BEN STINKS GRRRR

}