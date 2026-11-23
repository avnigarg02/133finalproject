'''squat.py

   Make Atlas squat: the feet stay planted on the ground while the
   pelvis moves down and back up.

   Idea:  Keep the pelvis upright and lower it by d(t) in the world.
   Relative to the pelvis, each foot must then RISE by d(t) while
   keeping its orientation.  Solve that with velocity IK (Jacobian
   inverse + error feedback) on two 6-DOF chains, pelvis->l_foot and
   pelvis->r_foot.  The pelvis motion itself is broadcast as the
   world->pelvis TF.

   Node:        /squat
   Publish:     /joint_states           sensor_msgs/JointState
   Broadcast:   world -> pelvis         TF
'''

import rclpy
import numpy as np
import tf2_ros

from math               import pi, cos, sin
from asyncio            import Future
from rclpy.node         import Node
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header

from utils.TransformHelpers     import *
from hw5code.KinematicChain     import KinematicChain


# All movable joints in the URDF (robot_state_publisher needs every one).
ALLJOINTS = [
    'back_bkx', 'back_bky', 'back_bkz', 'neck_ry',
    'l_arm_elx', 'l_arm_ely', 'l_arm_shx', 'l_arm_shz',
    'l_arm_wrx', 'l_arm_wry', 'l_arm_wry2',
    'r_arm_elx', 'r_arm_ely', 'r_arm_shx', 'r_arm_shz',
    'r_arm_wrx', 'r_arm_wry', 'r_arm_wry2',
    'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx',
    'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx',
]

# Leg joints in CHAIN ORDER (pelvis -> foot), as KinematicChain expects.
def legjoints(side):
    return [f'{side}_leg_{j}' for j in ['hpz', 'hpx', 'hpy', 'kny', 'aky', 'akx']]


class SquatNode(Node):
    def __init__(self, name, future):
        super().__init__(name)
        self.future = future

        # Squat parameters.
        self.depth  = 0.25      # How far the pelvis drops (m)
        self.period = 4.0       # Seconds per squat (down + up)
        self.reps   = 3         # Number of squats, then stop
        self.lam    = 20.0      # IK error-feedback gain

        # Kinematic chains for both legs.
        self.legs = {}
        for side in ['l', 'r']:
            names = legjoints(side)
            chain = KinematicChain(self, 'pelvis', f'{side}_foot', names)

            # Start with slightly bent knees (straight legs are singular).
            # hip pitch + knee + ankle pitch = 0 keeps the foot flat.
            q0 = np.array([0.0, 0.0, -0.3, 0.6, -0.3, 0.0])
            (p0, R0, _, _) = chain.fkin(q0)

            self.legs[side] = dict(names=names, chain=chain,
                                   q=q0.copy(), qdot=np.zeros(6),
                                   p0=p0, R0=R0, pdlast=p0.copy())

        # Pelvis height in the world so the feet sit at z = 0.
        self.h0 = -self.legs['l']['p0'][2]

        # Publishers.
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)
        self.tfbroad  = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while not self.count_subscribers('/joint_states'):
            pass

        self.dt    = 0.01
        self.t     = -self.dt
        self.now   = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.update)

    def shutdown(self):
        self.timer.destroy()
        self.destroy_node()

    def update(self):
        self.t   += self.dt
        self.now += rclpy.time.Duration(seconds=self.dt)

        if self.t > self.reps * self.period:
            self.future.set_result("Squats complete")
            return

        # Pelvis drop d(t): smooth 0 -> depth -> 0 cosine each period.
        w    = 2*pi / self.period
        d    = 0.5 * self.depth * (1 - cos(w * self.t))
        ddot = 0.5 * self.depth * w * sin(w * self.t)

        # Solve IK for each leg: foot rises by d relative to the pelvis.
        for leg in self.legs.values():
            pd = leg['p0'] + np.array([0.0, 0.0, d])
            vd = np.array([0.0, 0.0, ddot])
            Rd = leg['R0']
            wd = np.zeros(3)

            (p, R, Jv, Jw) = leg['chain'].fkin(leg['q'])
            vr = vd + self.lam * ep(leg['pdlast'], p)
            wr = wd + self.lam * eR(Rd, R)
            J  = np.vstack((Jv, Jw))
            qdot = np.linalg.solve(J, np.concatenate((vr, wr)))

            leg['q']      = leg['q'] + self.dt * qdot
            leg['qdot']   = qdot
            leg['pdlast'] = pd

        # Publish every joint (non-leg joints stay at zero).
        pos = dict.fromkeys(ALLJOINTS, 0.0)
        vel = dict.fromkeys(ALLJOINTS, 0.0)
        for leg in self.legs.values():
            for (n, qi, qdi) in zip(leg['names'], leg['q'], leg['qdot']):
                pos[n], vel[n] = qi, qdi

        header = Header(stamp=self.now.to_msg(), frame_id='world')
        self.pubjoint.publish(JointState(
            header=header, name=ALLJOINTS,
            position=[pos[n] for n in ALLJOINTS],
            velocity=[vel[n] for n in ALLJOINTS]))

        # Move the pelvis down in the world.
        ppelvis = np.array([0.0, 0.0, self.h0 - d])
        self.tfbroad.sendTransform(TransformStamped(
            header=header, child_frame_id='pelvis',
            transform=Transform_from_Rp(Reye(), ppelvis)))


def main(args=None):
    rclpy.init(args=args)
    future = Future()
    node = SquatNode('squat', future)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info("Stopping: " +
                           (future.result() if future.done() else "Interrupted"))
    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
