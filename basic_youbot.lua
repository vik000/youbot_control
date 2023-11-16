--lua

sim=require'sim'

function setMovement(forwBackVel,leftRightVel,rotVel)
    -- Apply the desired wheel velocities:
    sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
    sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
end

function sysCall_thread()
    --Prepare initial values and retrieve handles:
    wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheelJoints[1]=sim.getObject('./rollingJoint_fl')
    wheelJoints[2]=sim.getObject('./rollingJoint_rl')
    wheelJoints[3]=sim.getObject('./rollingJoint_rr')
    wheelJoints[4]=sim.getObject('./rollingJoint_fr')
    armJoints={}
    for i=0,4,1 do
        armJoints[i+1]=sim.getObject('./youBotArmJoint'..i)
    end

    setMovement(0,0.5,0)
    sim.wait(10)
    setMovement(0,0,0.5)
end