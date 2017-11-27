# LQR Guidance for ship based net recovery of fixed wing UAVs
Linear Quadratic is an optimal control theory problem where the dynamic system in consideration is defined using a set of linear equations and the cost is defined using quadratic equations. LQR optimal guidance has been implemented for the auto landing problem. The guidance is decoupled across two mutually perpendicular planes. 

The landing controller is verified using SITL which uses JSBSim for flight dynamics and environment and ardupilot as autopilot controller. I built my controller on top of the ardupilot autopilot which controls the roll and pitch of the plane by directly overiding over the rc channels of control surfaces. The controller was developed using ros-python and Mavros was used to connect SITL and ROS. 
