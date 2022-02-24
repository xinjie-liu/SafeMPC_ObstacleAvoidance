import numpy as np


class Robot:
    def __init__(self, mp=np.array([0.0, 0.0]), phi=0, q=np.array([0.0, 0.0])):
        self.l = np.array([1, 1])  # links length
        self.R = 0.5  # radius of the mobile base
        self.h = 0.4  # distance between center and the wheel
        self.q = q  # joint positions
        self.dq = np.array([0.0, 0.0])  # joint velocities
        self.u = np.array([1.0, 1.0])  # velocities of wheels
        self.phi = phi  # orientation (heading) of the mobile base
        # Position of the mobile base:
        self.mp = mp
        # Initial position of the end effector:
        # self.p = np.array([(np.sum(self.l))*np.cos(self.phi),
        #                   (np.sum(self.l))*np.sin(self.phi)])
        self.p = mp + self.rotationMatrix(phi).dot(np.array([self.l[0] * np.cos(q[0]) + self.l[1] * np.cos(q[0] + q[1]),
                                                             self.l[0] * np.sin(q[0]) + self.l[1] * np.sin(
                                                                 q[0] + q[1])]))
        # Initial position of the first joint:
        # self.p_joint_1 = np.array([(self.l[0])*np.cos(self.phi),
        #                   (self.l[1])*np.sin(self.phi)])
        self.p_joint_1 = mp + self.rotationMatrix(phi).dot(
            np.array([self.l[0] * np.cos(q[0]), self.l[0] * np.sin(q[0])]))
        # FIX THIS - I THINK I FIXED IT
        self.theta = phi + np.sum(q)  # orientation of the end effector (FIX THIS)

        self.phi_dot = 0
        self.u_limits = 10
        self.dq_limits = 5

    def rotationMatrix(self, a):
        R = np.array([[np.cos(a), -np.sin(a)],
                      [np.sin(a), np.cos(a)]])

        return R

    # WE DONT REALLY USE THIS ANYMORE, FIX IT
    def ForwardKinematics(self, u_des, dq_des, phi_dot):
        Q_dot = np.array([u_des[0], u_des[1], dq_des[0], dq_des[1]])
        J = self.Jacobian(u_des)
        X_dot = np.dot(J, Q_dot)

        return X_dot

    def ForwardKinematicsConfig(self, q):  # Find the workspace pose based on the configuration
        # Position of the mobile base:
        mp = np.array([q[0], q[1]])

        # Find the rotation matrix based on the heading of the mobile base
        R = self.rotationMatrix(q[2])
        # Find the position of the centre of each link (for the polygons):
        p_centre1 = mp + R.dot(np.array([self.l[0] / 2 * np.cos(q[3]), self.l[0] / 2 * np.sin(q[3])]))
        #
        p_centre2 = mp + R.dot(np.array([self.l[0] * np.cos(q[3]) + self.l[1] / 2 * np.cos(q[3] + q[4]),
                                         self.l[0] * np.sin(q[3]) + self.l[1] / 2 * np.sin(q[3] + q[4])]))

        joint_1 = mp + R.dot(np.array([self.l[0] * np.cos(q[3]), self.l[0] * np.sin(q[3])]))
        #
        l_2 = self.l[1] - 0.25
        joint_2 = mp + R.dot(np.array([self.l[0] * np.cos(q[3]) + l_2 * np.cos(q[3] + q[4]),
                                       self.l[0] * np.sin(q[3]) + l_2 * np.sin(q[3] + q[4])]))

        return mp, p_centre1, p_centre2, joint_1, joint_2

        return

    def Jacobian(self, u):
        cphi = np.cos(self.phi)
        sphi = np.sin(self.phi)
        q1 = self.q[0]
        q2 = self.q[1]

        # Position of the end effector w.r.t the mobile base:
        p_em = np.array([self.l[0] * np.cos(q1) + self.l[1] * np.cos(q1 + q2),
                         self.l[0] * np.sin(q1) + self.l[1] * np.sin(q1 + q2)])
        # a and b are the components associated with dR/dt * p_em (derivative of the rot matrix)
        a = (-sphi * p_em[0] - cphi * p_em[1]) / (2 * self.h)
        b = (cphi * p_em[0] - sphi * p_em[1]) / (2 * self.h)

        J = np.array([[cphi / 2 - a, cphi / 2 + a,
                       cphi * (-self.l[0] * np.sin(q1) - self.l[1] * np.sin(q1 + q2)) - sphi * (
                                   self.l[0] * np.cos(q1) + self.l[1] * np.cos(q1 + q2)),
                       cphi * (-self.l[1] * np.sin(q1 + q2)) - sphi * (self.l[1] * np.cos(q1 + q2))],
                      [sphi / 2 - b, sphi / 2 + b,
                       sphi * (-self.l[0] * np.sin(q1) - self.l[1] * np.sin(q1 + q2)) + cphi * (
                                   self.l[0] * np.cos(q1) + self.l[1] * np.cos(q1 + q2)),
                       sphi * (-self.l[1] * np.sin(q1 + q2)) + cphi * (self.l[1] * np.cos(q1 + q2))],
                      [-1 / (2 * self.h), 1 / (2 * self.h), 1, 1]])
        return J

    def Update(self, mp, phi, p, theta, q, dq, u, p_joint_1):
        self.mp = mp
        self.phi = phi
        self.p = p
        self.theta = theta
        self.q = q
        self.dq = dq
        self.u = u
        self.p_joint_1 = p_joint_1