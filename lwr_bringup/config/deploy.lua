require "rttlib"
require "rttros"

tc=rtt.getTC()
d=tc:getPeer("Deployer")

d:import("rtt_rosnode")
-- Start of user code imports
d:import("lwr_fri")
d:import("lwr_impedance_controller")
d:import("oro_joint_state_publisher")
-- End of user code

ros=false
prio=0
schedpol=rtt.globals.ORO_SCHED_OTHER

local opttab=utils.proc_args(arg)
local cp=rtt.Variable("ConnPolicy")

function conn2ros(depl, port, topic)
   depl:stream(port,rtt.provides("ros"):topic(topic))
end

d:loadComponent("TrqSum", "SumVectors2")
d:setActivity("TrqSum", 0.0, 5, rtt.globals.ORO_SCHED_RT)
TrqSum = d:getPeer("TrqSum")
TrqSum:configure()

d:loadComponent("NullTrqSum", "SumVectors2")
d:setActivity("NullTrqSum", 0.0, 5, rtt.globals.ORO_SCHED_RT)
NullTrqSum = d:getPeer("NullTrqSum")
NullTrqSum:configure()

d:loadComponent("CartImp", "CartesianImpedance")
d:setActivity("CartImp", 0.0, 5, rtt.globals.ORO_SCHED_RT)
CartImp = d:getPeer("CartImp")
CartImp:configure()

d:loadComponent("LWRDiag", "FRIDiagnostics")
d:setActivity("LWRDiag", 0.02, 5, rtt.globals.ORO_SCHED_RT)
LWRDiag = d:getPeer("LWRDiag")
LWRDiag:configure()

d:loadComponent("Projection", "NullSpaceProjection")
d:setActivity("Projection", 0.0, 5, rtt.globals.ORO_SCHED_RT)
Projection = d:getPeer("Projection")
Projection:configure()

d:loadComponent("JntLim", "JointLimitsAvoidance")
d:setActivity("JntLim", 0.0, 5, rtt.globals.ORO_SCHED_RT)
JntLim = d:getPeer("JntLim")
JntLim:configure()

d:loadComponent("FRI", "FRIComponent")
d:setActivity("FRI", 0.0, 5, rtt.globals.ORO_SCHED_RT)
FRI = d:getPeer("FRI")
FRI:getProperty("fri_port"):set(49938)
FRI:configure()

d:loadComponent("CartImpTrj", "ImpedanceTrajectoryGenerator")
d:setActivity("CartImpTrj", 0.0, 5, rtt.globals.ORO_SCHED_RT)
CartImpTrj = d:getPeer("CartImpTrj")
CartImpTrj:configure()

d:loadComponent("Singular", "SingularityAvoidance")
d:setActivity("Singular", 0.0, 5, rtt.globals.ORO_SCHED_RT)
Singular = d:getPeer("Singular")
Singular:configure()

d:loadComponent("JntPub", "JointStatePublisher")
d:setActivity("JntPub", 0.01, 2, rtt.globals.ORO_SCHED_RT)
JntPub = d:getPeer("JntPub")
sched_order=JntPub:getProperty("joint_names")
sched_order:get():resize(7)
sched_order[0]="right_arm_0_joint"
sched_order[1]="right_arm_1_joint"
sched_order[2]="right_arm_2_joint"
sched_order[3]="right_arm_3_joint"
sched_order[4]="right_arm_4_joint"
sched_order[5]="right_arm_5_joint"
sched_order[6]="right_arm_6_joint"
JntPub:configure()

--d:loadComponent("FTMsr", "FrameTransformer")
--d:setActivity("FTMsr", 0.01, 2, rtt.globals.ORO_SCHED_RT)
--FTMsr = d:getPeer("FTMsr")
--FTMsr:configure()

--d:loadComponent("FTCmd", "FrameTransformer")
--d:setActivity("FTCmd", 0.01, 2, rtt.globals.ORO_SCHED_RT)
--FTCmd = d:getPeer("FTCmd")
--FTCmd:configure()

d:connect("TrqSum.Output", "FRI.JointTorqueCommand", rtt.Variable("ConnPolicy"))
d:connect("NullTrqSum.Output", "Projection.NullSpaceTorque", rtt.Variable("ConnPolicy"))
d:connect("CartImp.JointTorqueCommand", "TrqSum.Input2", rtt.Variable("ConnPolicy"))
d:connect("CartImp.JointImpedanceCommand", "FRI.JointImpedanceCommand", rtt.Variable("ConnPolicy"))
d:connect("CartImp.JointPositionCommand", "FRI.JointPositionCommand", rtt.Variable("ConnPolicy"))
d:stream("LWRDiag.Diagnostics",rtt.provides("ros"):topic("/diagnostics"))
d:connect("Projection.JointTorqueCommand", "TrqSum.Input1", rtt.Variable("ConnPolicy"))
d:connect("JntLim.JointTorqueCommand", "NullTrqSum.Input1", rtt.Variable("ConnPolicy"))
d:connect("FRI.RobotState", "LWRDiag.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRI.FRIState", "LWRDiag.FRIState", rtt.Variable("ConnPolicy"))
d:connect("FRI.CartesianVelocity", "CartImp.CartesianVelocity", rtt.Variable("ConnPolicy"))
d:connect("FRI.CartesianPosition", "CartImpTrj.CartesianPosition", rtt.Variable("ConnPolicy"))
d:connect("FRI.CartesianPosition", "CartImp.CartesianPosition", rtt.Variable("ConnPolicy"))
d:connect("FRI.MassMatrix", "Projection.MassMatrix", rtt.Variable("ConnPolicy"))
d:connect("FRI.MassMatrix", "CartImp.MassMatrix", rtt.Variable("ConnPolicy"))
d:connect("FRI.Jacobian", "CartImp.Jacobian", rtt.Variable("ConnPolicy"))
d:connect("FRI.Jacobian", "Projection.Jacobian", rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition", "JntLim.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition", "CartImp.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition", "JntPub.msrJntPos", rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition", "Singular.JointPosition", rtt.Variable("ConnPolicy"))
d:connect("CartImpTrj.CartesianPositionCommand", "CartImp.CartesianPositionCommand", rtt.Variable("ConnPolicy"))
d:connect("CartImpTrj.CartesianWrenchCommand", "CartImp.CartesianWrenchCommand", rtt.Variable("ConnPolicy"))
d:connect("CartImpTrj.CartesianImpedanceCommand", "CartImp.CartesianImpedanceCommand", rtt.Variable("ConnPolicy"))
d:connect("Singular.JointTorqueCommand", "NullTrqSum.Input2", rtt.Variable("ConnPolicy"))
--d:connect("CartImpTrj.CartesianPositionCommand", "FTCmd.FrameInput", rtt.Variable("ConnPolicy"))
--d:connect("FRI.CartesianPosition", "FTMsr.FrameInput", rtt.Variable("ConnPolicy"))

d:stream("CartImpTrj.ImpedanceTrajectory",rtt.provides("ros"):topic("/cartesiant_trajectory"))
d:stream("FRI.KRL_CMD",rtt.provides("ros"):topic("/lwrarm_controller/fri_set_mode"))
d:stream("JntPub.joints_state",rtt.provides("ros"):topic("/joint_states"))
d:stream("Singular.Manipulability",rtt.provides("ros"):topic("/manipulability"))
d:stream("FRI.CartesianWrench",rtt.provides("ros"):topic("/cartesian_wrench"))

--d:stream("FTMsr.FrameOutput",rtt.provides("ros"):topic("/msr/pose_tool_msr"))
--d:stream("FTCmd.FrameOutput",rtt.provides("ros"):topic("/msr/pose_tool_cmd"))

-- Start of user code usercode
FRI:start()
LWRDiag:start()
JntPub:start()
CartImp:start()
Projection:start()
TrqSum:start()
JntLim:start()
Singular:start()
NullTrqSum:start()
--FTMsr:start()
--FTCmd:start()
--d:waitForSignal()
-- End of user code

