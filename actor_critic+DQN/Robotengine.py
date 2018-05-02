import numpy as np
import copy
import math # math가 없다고오오??????
from pyquaternion import Quaternion

#Feqc = Mtot * g
#Feqa = np.diag([Mtot, Mtot, Mtot])
#Crossvec = np.zeros((3, 1), dtype=np.float64)
#Teqalpha = np.zeros((3, 3), dtype=np.float64)
#Teqc = np.zeros((3, 1), dtype=np.float64)
#mlsum = np.zeros((3, 1), dtype=np.float64)
#sumDs = np.zeros((3, 3), dtype=np.float64)
def rotate_axis(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = axis.flatten()
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def Q2rot(Q):
    qw = 0.5 * np.sqrt(max(np.matrix.trace(Q) + 1., 0.))
    #  print('trace = ', np.matrix.trace(Q))
    qx = (Q[2][1] - Q[1][2]) / 4 / qw
    qy = (Q[0][2] - Q[2][0]) / 4 / qw
    qz = (Q[1][0] - Q[0][1]) / 4 / qw
    qv = np.array([[qx], [qy], [qz]])
    # quaternion normalization

    qvsquare = sum(qv * qv)
    qnorm = np.sqrt(qw * qw + qvsquare)

    qw /= qnorm
    qv /= qnorm
    # quaternion to Q

    q = Quaternion(np.concatenate([qw, qv.flatten()]).flatten())
    re_Q = q.rotation_matrix
    return re_Q
numsubleg = 3
numLeg = 4
Mtot = 0.9849
dtime = 0.01
g_scal = 9.81
Fricscale = 0.5 * Mtot * g_scal
g = np.array([[0], [0], [-g_scal]])
resis_scale_square = 0.5 * Mtot*g_scal
resis_scale_lin = 1.*Mtot*g_scal


class Rigidbody:
    def __init__(self, m=0.0, Q =np.identity(3, dtype=np.float64), Ib =np.identity(3, dtype=np.float64), wb=np.zeros((3, 1), dtype=np.float64)):
        self.m = m
        self.Q = Q
        self.Ib = Ib
        self.wb = wb


class Robotbody(Rigidbody):
    def __init__(self, m=0.0, rs=np.zeros((3, 1), dtype=np.float64), vs=np.zeros((3, 1), dtype=np.float64),
                 lbtomot=[np.zeros((3, 1), dtype=np.float64) for _ in range(numLeg)]):
        Rigidbody.__init__(self, m=m)
        self.lbtomot = [np.zeros((3, 1), dtype=np.float64) for _ in range(numLeg)]
        self.lbtomot[0] = lbtomot[0]
        self.lbtomot[1] = lbtomot[1]
        self.lbtomot[2] = lbtomot[2]
        self.lbtomot[3] = lbtomot[3]
        self.rs = rs
        self.vs = vs
        self.lbtomot = lbtomot
        self.Rnow = []
        self.Rbefore = []
        for p in range(numLeg):
            self.Rnow += [np.zeros((3, 1), dtype=np.float64)]
            self.Rbefore += [np.zeros((3, 1), dtype=np.float64)]


class subleg(Rigidbody):
    def __init__(self, axis=np.zeros((3, 1), dtype=np.float64), l=np.zeros((3, 1), dtype=np.float64), theta=0.0, omega=0.0):
        Rigidbody.__init__(self)
        self.axis = axis
        self.l = l
        self.theta = theta
        self.omega = omega

class Leg:
    def __init__(self):
        self.sub = [subleg() for _ in range(numsubleg)]


class robot:
    def __init__(self):
        self.body = Robotbody()
        self.leg = [Leg() for _ in range(numLeg)]
        self.numsubleg = 3
        self.numLeg = 4
        self.joints = [np.zeros((3, 1), dtype=np.float64) for _ in range(numsubleg*numLeg)]
        self.dtime = 0.01

    def set_constants(self):
        self.body.rs = np.array([[0], [0], [0.20]])
        self.body.vs = np.zeros((3, 1), dtype=np.float64)
        self.body.wb = np.zeros((3, 1), dtype=np.float64)
        self.body.Q = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
        # Set Axes
        self.leg[0].sub[0].axis = np.array([[1.], [0.], [0.]])
        self.leg[0].sub[1].axis = np.array([[0.], [1.], [0.]])
        self.leg[0].sub[2].axis = np.array([[0.], [1.], [0.]])

        self.leg[1].sub[0].axis = np.array([[1.], [0.], [0.]])
        self.leg[1].sub[1].axis = np.array([[0.], [1.], [0.]])
        self.leg[1].sub[2].axis = np.array([[0.], [1.], [0.]])

        self.leg[2].sub[0].axis = np.array([[1.], [0.], [0.]])
        self.leg[2].sub[1].axis = np.array([[0.], [1.], [0.]])
        self.leg[2].sub[2].axis = np.array([[0.], [1.], [0.]])

        self.leg[3].sub[0].axis = np.array([[1.], [0.], [0.]])
        self.leg[3].sub[1].axis = np.array([[0.], [1.], [0.]])
        self.leg[3].sub[2].axis = np.array([[0.], [1.], [0.]])

        #  Set theta
        for p in range(self.numLeg):
            for i in range(self.numsubleg):
                self.leg[p].sub[i].theta = 0.

        # Set lvectors
        self.body.lbtomot[0] = np.array([[0.065], [0.065], [0.02]])
        self.body.lbtomot[1] = np.array([[-0.065], [0.065], [0.02]])
        self.body.lbtomot[2] = np.array([[0.065], [-0.065], [0.02]])
        self.body.lbtomot[3] = np.array([[-0.065], [-0.065], [0.02]])

        self.leg[0].sub[0].l = np.array([[0.], [0.020], [-0.015]])
        self.leg[0].sub[1].l = np.array([[0.], [0.], [-0.066]])
        self.leg[0].sub[2].l = np.array([[0.], [0.], [-0.1]])

        self.leg[1].sub[0].l = np.array([[0.], [0.020], [-0.015]])
        self.leg[1].sub[1].l = np.array([[0.], [0.], [-0.066]])
        self.leg[1].sub[2].l = np.array([[0.], [0.], [-0.1]])

        self.leg[2].sub[0].l = np.array([[0.], [-0.020], [-0.015]])
        self.leg[2].sub[1].l = np.array([[0.], [0.], [-0.066]])
        self.leg[2].sub[2].l = np.array([[0.], [0.], [-0.1]])

        self.leg[3].sub[0].l = np.array([[0.], [-0.020], [-0.015]])
        self.leg[3].sub[1].l = np.array([[0.], [0.], [-0.066]])
        self.leg[3].sub[2].l = np.array([[0.], [0.], [-0.1]])

        # set Mass
        Mtot = 0.9849
        self.body.Ib = np.array([[75.0e-4, 0., 0.],
                                 [0., 75.0e-4, 0.],
                                 [0., 0., 50.0e-4]])

    def set_omega(self, action):
        action = np.reshape(action, [self.numLeg, self.numsubleg])
        for p in range(numLeg):
            for i in range(numsubleg):
                self.leg[p].sub[i].omega = action[p][i]

    def theta_update(self):
        for p in range(self.numLeg):
            for i in range(self.numsubleg):
                if (self.leg[p].sub[i].theta > math.pi/6.):
                    self.leg[p].sub[i].theta -= 0.005
                elif (self.leg[p].sub[i].theta < -math.pi/6.):
                    self.leg[p].sub[i].theta += 0.005
                else:
                    self.leg[p].sub[i].theta += self.leg[p].sub[i].omega * dtime
                #  주의: theta의 제한범위를 구해야한다.

    def joint_done(self):
        for i in range(len(self.joints)):
            if(self.joints[i].flatten()[2]<0):
                return 1 #끝남 ㅅㅅ
        return 0 #안 끝남
    # Q to quaternion 확인됨


    def timeflow(self):
        self.theta_update()
        '''Momentum = tf.matmul(tf.matmul(self.body.wb, self.body.Ib), self.body.Q) + tf.scalar_mul(self.body.m,
                                                                                                 tf.cross(self.body.rs,
                                                                                                          self.body.vs))'''
        wbnorm = np.sqrt(np.sum(np.square(self.body.wb)))
        if(wbnorm != 0):
            self.body.Q = rotate_axis(self.body.wb/wbnorm, np.linalg.norm(self.body.wb)*dtime).dot(self.body.Q)

        #self.body.Q += dtime * np.matrix.transpose(np.cross(np.matrix.transpose(self.body.wb), np.matrix.transpose(self.body.Q)))
        self.body.Q = Q2rot(self.body.Q)
        self.body.rs += self.body.vs * dtime

        #  wbs = np.matmul(self.body.wb, self.body.Q)  # [1,3] matrix
        tot_lbtomots = []
        #  lbodytoend[0~3] 구하기 in body frame
        #################################################
        lbodytoend = []
        for i in range(self.numLeg):
            lbodytoend += [np.zeros((3, 1), dtype=np.float64)]
        for p in range(self.numLeg):
            R1 = rotate_axis(self.leg[p].sub[0].axis, self.leg[p].sub[0].theta)
            R2 = rotate_axis(self.leg[p].sub[1].axis, self.leg[p].sub[1].theta)
            R3 = rotate_axis(self.leg[p].sub[2].axis, self.leg[p].sub[2].theta)
            self.joints[p * 3] = self.body.rs + self.body.Q.dot(self.body.lbtomot[p])
            self.joints[p * 3 + 1] = self.joints[p * 3] + self.body.Q.dot(self.leg[p].sub[0].l)
            self.joints[p * 3 + 2] = self.joints[p * 3 + 1] + self.body.Q.dot(R1.dot(R2.dot(self.leg[p].sub[1].l)))
            lbodytoend[p] = self.body.lbtomot[p] + self.leg[p].sub[0].l+R1.dot(R2.dot(self.leg[p].sub[1].l))+R1.dot(R2.dot(R3.dot(self.leg[p].sub[2].l)))
        # Force 이것도 함수로 만들면 좋을 것 같은데...
        # 주의: 실제상황에서는 지면이 땅에 닿아있는 발의 개수를 알지는 않음. 따라서 나중에 비선형, np.sign(vznow)의 부호에 따른 F를 다시 짜야함
        Force = []
        Torque = []
        zsum = 0.0000000
        for p in range(self.numLeg):
            self.body.Rnow[p] = self.body.Q.dot(lbodytoend[p]) + self.body.rs
            if (self.body.Rnow[p][2][0] < -0.):
                zsum += self.body.Rnow[p][2][0]
        for p in range(self.numLeg):
            if (self.body.Rnow[p][2][0] < -0.):
                vnow = (self.body.Rnow[p] - self.body.Rbefore[p])/dtime
                #  print('vnow = ', vnow)
                #  print('wb = ', self.body.wb)
                vznow = vnow[2][0]
                F = -Mtot*g*self.body.Rnow[p][2][0]/zsum*1.05
                #  print('zsum = ', zsum)
                #  print('0', F)
                F -= resis_scale_lin * vznow * np.array([[0.], [0.], [1.]])
                F -= resis_scale_square * np.square(vznow) * np.sign(vznow) * np.array([[0.], [0.], [1.]])
                F -= Fricscale * (np.array([[1.], [1.], [0.]]) * vnow)
                #  주의: 제곱항은 파고들때만 되도록 정정해야됨.
                Force += [F]
                Torque += [np.matrix.transpose(
                    np.cross(np.matrix.transpose(self.body.Q.dot(lbodytoend[p])), np.matrix.transpose(F)))]
        totForce = np.zeros((3, 1), dtype=np.float64)
        totTorque = np.zeros((3, 1), dtype=np.float64)
        for F in Force:
            totForce += F
        for T in Torque:
            totTorque += T
        totForce += Mtot*g
        #  print('abs = ', totForce / Mtot)
        self.body.vs += totForce / Mtot * dtime
        totTorquebody = np.linalg.inv(self.body.Q).dot(totTorque)
        #  print('totTorquebody = ', totTorquebody)
        self.body.wb += np.linalg.inv(self.body.Ib).dot(totTorquebody
                                                        - np.matrix.transpose(np.cross(np.matrix.transpose(self.body.wb),
                                                                                       np.matrix.transpose(self.body.Ib.dot(self.body.wb))
                                                                                       )))*dtime
        '''  print('alphabs = ', np.linalg.inv(self.body.Ib).dot(totTorquebody
                                                        - np.matrix.transpose(np.cross(np.matrix.transpose(self.body.wb),
                                                                                       np.matrix.transpose(self.body.Ib.dot(self.body.wb))
                                                                                       ))))'''
        for p in range(numLeg):
            self.body.Rbefore[p] = self.body.Rnow[p]

        # ----------------------------------------------
        '''for p in range(self.numLeg):
            for i in range(numsubleg):
                self.leg[p].sub[i].omega += self.leg[p].sub[i].alpha * dtime  # omega를 시간에 따라 갱신
                self.leg[p].sub[i].theta += self.leg[p].sub[i].omega * dtime  # theta를 시간에 따라 갱신
                self.leg[p].sub[i].Q = tf.scalar_mul(tf.cos(self.leg[p].sub[i].theta), tf.eye(3, dtype=tf.float64)) + \
                                       tf.scalar_mul(1. - tf.cos(self.leg[p].sub[i].theta),
                                                     tf.matmul(self.leg[p].sub[i].axis, self.leg[p].sub[i].axis,
                                                               transpose_a=True)) + \
                                       tf.scalar_mul(tf.sin(self.leg[p].sub[i].theta),
                                                     tf.cross(tf.tile(self.leg[p].sub[i].axis, [3, 1]),
                                                              tf.eye(3, dtype=tf.float64)))
            Qs = [tf.matmul(self.leg[p].sub[0].Q, self.body.Q)]  # Qs는 i번째 subleg에서 space로의 좌표변환
            # List of rotation matrices of each sublegs in space frame
            # Type : list of [3,3] Tensor
            for i in range(1, numsubleg):
                Qs.append(tf.matmul(self.leg[p].sub[i].Q, Qs[i - 1]))

            Is = [tf.matmul(tf.matmul(Qs[i], self.leg[p].sub[i].Ib, transpose_a=True), Qs[i]) for i in range(numsubleg)]

            e = [tf.matmul(self.leg[p].sub[i].axis, Qs[i]) for i in range(numsubleg)]
            # List of axes of each sublegs in space frame
            # Type : list of [None,3] Tensor

            Qalpha = [tf.scalar_mul(self.leg[p].sub[i].alpha, e[i]) for i in range(numsubleg)]

            Qalphasum = [Qalpha[0]]
            for i in range(1, numsubleg): Qalphasum.append(Qalphasum[i - 1] + Qalpha[i])

            Qw = [tf.scalar_mul(self.leg[p].sub[i].omega, e[i]) for i in range(numsubleg)]

            ws = [wbs + Qw[0]]
            for i in range(1, numsubleg):
                ws.append(ws[i - 1] + Qw[i])

            w = [tf.matmul(ws[i], Qs[i], transpose_b=True) for i in range(numsubleg)]

            ls = [[tf.matmul(self.leg[p].sub[i].l[0], Qs[i]),
                   tf.matmul(self.leg[p].sub[i].l[1], Qs[i])] for i in range(numsubleg)]  # ls = 2Dtensor

            lbtomotbs = tf.matmul(self.body.lbtomot[p], self.body.Q)  # lbtomotbs = 2Dtensor

            lbtomots = [lbtomotbs + ls[0][0]]  # lbtomots = 2Dtensor

            for i in range(1, numsubleg):
                lbtomots.append(lbtomots[i - 1] + ls[i - 1][1] + ls[i][0])
            for i in range(numsubleg):
                mlsum += tf.scalar_mul(self.leg[p].sub[i].m, lbtomots[i])
            # 각운동량 디버깅용
            vmotbs = [tf.cross(wbs, lbtomotbs) + tf.cross(ws[0], ls[0][0])]
            for i in range(1, numsubleg):
                vmotbs.append(vmotbs[i - 1] + tf.cross(ws[i - 1], ls[i - 1][1]) + tf.cross(ws[i], ls[i][0]))

            # Calculating External Forces
            vs = self.body.vs
            for i in range(numsubleg):
                Collisiontemp = tf.cast(
                    tf.less(lbtomots[i] + ls[i][1] + self.body.rs, tf.zeros((3, 1), dtype=tf.float64)), tf.float64)
                Collisionz = tf.multiply(Collisiontemp, tf.constant([[0, 0, 1]], tf.float64))
                Collisionxy = tf.matmul(Collisionz, tf.constant([[0, 0, 0], [0, 0, 0], [1, 1, 0]],
                                                                tf.float64))  ##더 연산량을 줄일 수 있을 듯 방법을 강구하라
                vs += tf.cross(ws[i], ls[i][0] + ls[i][1])
                vCollision = tf.cast(tf.less(vs, tf.zeros((3, 1), dtype=tf.float64)), tf.float64)
                Ftemp = tf.multiply(Collisionz, Fadded + tf.multiply((vCollision - Offset), Fsubed))
                Feqc += Ftemp
                Teqc += tf.cross(lbtomots[i] + ls[i][1], Ftemp)
                FrictionTemp = -tf.multiply(tf.scalar_mul(Fricscale, vs), Collisionxy)  ##########하.. 힘이 너무 다 틀렸어
                Feqc += FrictionTemp
                Teqc += tf.cross(lbtomots[i] + ls[i][1], FrictionTemp)

            A = [tf.cross(wbs, tf.cross(wbs, lbtomotbs))
                 + tf.cross(Qalphasum[0], ls[0][0])
                 + tf.cross(ws[0], tf.cross(ws[0], ls[0][0]))]

            for i in range(1, numsubleg):
                A.append(
                    tf.cross(Qalphasum[i - 1], ls[i - 1][1])
                    + tf.cross(Qalphasum[i], ls[i][0])
                    + tf.cross(ws[i - 1], tf.cross(ws[i - 1], ls[i - 1][1]))
                    + tf.cross(ws[i], tf.cross(ws[i], ls[i][0]))
                )

            mlsquare = tf.zeros((1), dtype=tf.float64)
            for i in range(numsubleg):
                mlsquare += tf.scalar_mul(self.leg[p].sub[i].m,
                                          tf.matmul(lbtomots[i], lbtomots[i], transpose_b=True))
            mlsquare = tf.reshape(mlsquare, [-1])
            Dya = tf.zeros([3, 3], dtype=tf.float64)
            for i in range(numsubleg):
                Dya += tf.scalar_mul(self.leg[p].sub[i].m, tf.matmul(lbtomots[i], lbtomots[i], transpose_a=True))
            ###############
            Ds = tf.diag(tf.concat([mlsquare, mlsquare, mlsquare], axis=0)) - Dya
            Teqalpha += Ds
            sumDs += Ds
            # Qb * Ib * Qb.transpose()

            for i in range(numsubleg):
                Feqc -= tf.scalar_mul(self.leg[p].sub[i].m, A[i])
                Crossvec += tf.scalar_mul(self.leg[p].sub[i].m, lbtomots[i])
                Teqc += tf.matmul(tf.cross(tf.matmul(w[i], self.leg[p].sub[i].Ib), w[i]), Qs[i])
                Teqc -= tf.matmul(Qalphasum[i], Is[i])
                Teqalpha += Is[i]
                # Qs_i * I_i * Qs_i^T
            for i in range(numsubleg):
                Momentum += tf.matmul(tf.matmul(w[i], self.leg[p].sub[i].Ib), Qs[i])
                Momentum += tf.scalar_mul(self.leg[p].sub[i].m,
                                          tf.cross(lbtomots[i] + self.body.rs, vmotbs[i] + self.body.vs))
            # leg update
            # float32 -> float64 conversion : 171013 Fine
            # update 'Q's of leg - 20171012 fine
            tot_lbtomots += lbtomots
        Teqalpha += tf.matmul(tf.matmul(self.body.Q, self.body.Ib, transpose_a=True), self.body.Q)
        Teqc += tf.matmul(tf.cross(tf.matmul(self.body.wb, self.body.Ib), self.body.wb), self.body.Q)
        Teqc += tf.cross(mlsum, g)
        Teqanorm = tf.reduce_sum(tf.square(mlsum))
        alphabs = tf.matmul(
            Teqc - tf.scalar_mul(1. / Mtot, tf.cross(mlsum, Feqc)),
            tf.matrix_inverse(
                Teqalpha + tf.scalar_mul(1. / Mtot,
                                         Teqanorm * tf.eye(3, dtype=tf.float64) - tf.matmul(mlsum, mlsum,
                                                                                            transpose_a=True))
                # 여기가 너무 헷갈림.......
            )
        )
        asb = tf.scalar_mul(1. / Mtot, Feqc - tf.cross(mlsum, alphabs))
        alphab = tf.matmul(alphabs, self.body.Q, transpose_b=True)
        self.body.wb += tf.scalar_mul(dtime, alphab)
        self.body.Q += tf.scalar_mul(dtime, tf.cross(tf.concat([wbs, wbs, wbs], axis=0), self.body.Q))
        self.body.vs += tf.scalar_mul(dtime, asb)
        self.body.rs += tf.scalar_mul(dtime, self.body.vs)
        '''