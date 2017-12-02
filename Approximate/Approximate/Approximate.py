import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

RecordFile = open('record.txt','w')

numsubleg = 3
numLeg = 4
Mtot = 1.379
dtime = 0.01
Fupscale = 1.   
Fdownscale = 0.3
Fricscale = Mtot*9.81*0.01
g = tf.constant([[0.,0.,-9.81]],dtype=tf.float32)
Fup = tf.constant([[0,0,Mtot*Fupscale*9.81]],dtype=tf.float32)
Fdown = tf.constant([[0,0,Mtot*Fdownscale*9.81]],dtype=tf.float32)
Fadded = tf.constant([[0,0,Mtot*(Fupscale+Fdownscale)*9.81/2.]],dtype=tf.float32)
Fsubed = tf.constant([[0,0,Mtot*(Fupscale-Fdownscale)*9.81]],dtype=tf.float32)  
Offset = tf.constant([[0,0,0.5]],dtype=tf.float32)
Mtotinv = 1/Mtot
Ibinv = tf.matrix_inverse(tf.constant([[75.0e-5,0.,0.],
                               [0.,75.0e-5,0.],
                               [0.,0.,50.0e-5]],dtype=tf.float32))# 실제 값으로 바꿔야됨 ########################################################
        

#Variables
global_step = tf.Variable(0,trainable = False, name = 'global_step')


# 다리 정지해있는 물리엔진용 placeholders 잡기. 신경망에서는 지워야함
prs = tf.placeholder(tf.float32, [1,3])
pvs = tf.placeholder(tf.float32, [1,3])
pwb = tf.placeholder(tf.float32, [1,3])
pQb = tf.placeholder(tf.float32, [3,3])

class Rigidbody:
    def __init__(self,Q=tf.eye(3,dtype=tf.float32),wb=tf.zeros((1,3),dtype=tf.float32)):
        self.Q = Q
        self.wb = wb

class Robotbody(Rigidbody):
    def __init__(self, rs=tf.zeros((1,3), dtype=tf.float32), vs=tf.zeros((1,3), dtype=tf.float32), lbtomot = [tf.zeros((1,3),dtype=tf.float32) for _ in range(numLeg)]):
        self.lbtomot=[tf.zeros((1,3),dtype=tf.float32) for _ in range(numLeg)]
        self.lbtomot[0] = lbtomot[0]
        self.lbtomot[1] = lbtomot[1]
        self.lbtomot[2] = lbtomot[2]
        self.lbtomot[3] = lbtomot[3]
        self.rs = rs
        self.vs = vs
        self.lbtomot = lbtomot

class subleg(Rigidbody):
    def __init__(self,axis=tf.zeros((1,3),dtype=tf.float32),l = [tf.zeros((1,3),dtype=tf.float32) for _ in range(2)],theta=0.0,omega=0.0,alpha=0.0):
        self.axis = axis
        self.l=[tf.zeros((1,3),dtype=tf.float32) for _ in range(2)]
        self.l[0] = l[0]
        self.l[1] = l[1]
        self.theta = theta
        self.omega = omega
        self.alpha = alpha

class Leg:
    def __init__(self):
        self.sub = [subleg() for _ in range(numsubleg)]

class robot:
    def __init__(self):
        self.body = Robotbody()
        self.leg = [Leg() for _ in range(numLeg)]

    def set_constants(self):
        #Set Axes
        self.leg[0].sub[0].axis = tf.constant([[0.,1.,0.]],dtype=tf.float32)
        self.leg[0].sub[1].axis = tf.constant([[1.,0.,0.]],dtype=tf.float32)
        self.leg[0].sub[2].axis = tf.constant([[1.,0.,0.]],dtype=tf.float32)

        self.leg[1].sub[0].axis = tf.constant([[0.,-1.,0.]],dtype=tf.float32)
        self.leg[1].sub[1].axis = tf.constant([[1.,0.,0.]],dtype=tf.float32)
        self.leg[1].sub[2].axis = tf.constant([[1.,0.,0.]],dtype=tf.float32)

        self.leg[2].sub[0].axis = tf.constant([[0.,1.,0.]],dtype=tf.float32)
        self.leg[2].sub[1].axis = tf.constant([[-1.,0.,0.]],dtype=tf.float32)
        self.leg[2].sub[2].axis = tf.constant([[-1.,0.,0.]],dtype=tf.float32)

        self.leg[3].sub[0].axis = tf.constant([[0.,-1.,0.]],dtype=tf.float32)
        self.leg[3].sub[1].axis = tf.constant([[-1.,0.,0.]],dtype=tf.float32)
        self.leg[3].sub[2].axis = tf.constant([[-1.,0.,0.]],dtype=tf.float32)

        #Set lvectors
        self.body.lbtomot[0] = tf.constant([[0.065,0.065,0.02]],dtype=tf.float32)#############################3여기는 근사를 하면서 내려간 무게중심을 보정해주어야함
        self.body.lbtomot[1] = tf.constant([[-0.065,0.065,0.02]],dtype=tf.float32)
        self.body.lbtomot[2] = tf.constant([[0.065,-0.065,0.02]],dtype=tf.float32)
        self.body.lbtomot[3] = tf.constant([[-0.065,-0.065,0.02]],dtype=tf.float32)

        self.leg[0].sub[0].l[0] = tf.constant([[0.,0.020,0.0]],dtype=tf.float32)
        self.leg[0].sub[0].l[1] = tf.constant([[0.,0.,-0.015]],dtype=tf.float32)
        self.leg[0].sub[1].l[0] = tf.constant([[0.,0.033,0.]],dtype=tf.float32)
        self.leg[0].sub[1].l[1] = tf.constant([[0.,0.033,0.]],dtype=tf.float32)
        self.leg[0].sub[2].l[0] = tf.constant([[0.,0.030,0.]],dtype=tf.float32)
        self.leg[0].sub[2].l[1] = tf.constant([[0.,0.070,0.]],dtype=tf.float32)

        self.leg[1].sub[0].l[0] = tf.constant([[0.,0.020,0.0]],dtype=tf.float32)
        self.leg[1].sub[0].l[1] = tf.constant([[0.,0.,-0.015]],dtype=tf.float32)
        self.leg[1].sub[1].l[0] = tf.constant([[0.,0.033,0.]],dtype=tf.float32)
        self.leg[1].sub[1].l[1] = tf.constant([[0.,0.033,0.]],dtype=tf.float32)
        self.leg[1].sub[2].l[0] = tf.constant([[0.,0.030,0.]],dtype=tf.float32)
        self.leg[1].sub[2].l[1] = tf.constant([[0.,0.070,0.]],dtype=tf.float32)

        self.leg[2].sub[0].l[0] = tf.constant([[0.,-0.020,0.]],dtype=tf.float32)
        self.leg[2].sub[0].l[1] = tf.constant([[0.,0.,-0.015]],dtype=tf.float32)
        self.leg[2].sub[1].l[0] = tf.constant([[0.,-0.033,0.]],dtype=tf.float32)
        self.leg[2].sub[1].l[1] = tf.constant([[0.,-0.033,0.]],dtype=tf.float32)
        self.leg[2].sub[2].l[0] = tf.constant([[0.,-0.030,0.]],dtype=tf.float32)
        self.leg[2].sub[2].l[1] = tf.constant([[0.,-0.070,0.]],dtype=tf.float32)

        self.leg[3].sub[0].l[0] = tf.constant([[0.,-0.020,0.]],dtype=tf.float32)
        self.leg[3].sub[0].l[1] = tf.constant([[0.,0.,-0.015]],dtype=tf.float32)
        self.leg[3].sub[1].l[0] = tf.constant([[0.,-0.033,0.]],dtype=tf.float32)
        self.leg[3].sub[1].l[1] = tf.constant([[0.,-0.033,0.]],dtype=tf.float32)
        self.leg[3].sub[2].l[0] = tf.constant([[0.,-0.030,0.]],dtype=tf.float32)
        self.leg[3].sub[2].l[1] = tf.constant([[0.,-0.070,0.]],dtype=tf.float32)

        #set Mass
        for p in range(numLeg):
            #set Initial theta conditions
            for i in range(numsubleg):
                self.leg[p].sub[i].theta = tf.constant(0., dtype=tf.float32)
            self.leg[p].sub[1].theta = tf.constant(np.pi / 2., dtype=tf.float32)

    def timeflow(self,t = 0.0):

        wbs = tf.matmul(self.body.wb, self.body.Q) #[1,3] matrix
        tot_lbtomots = []

        Feqc = tf.constant([0,0,-Mtot*9.81], dtype = tf.float32)
        Teqc = tf.zeros([1,3], dtype = tf.float32)

        #List of External Forces
        Flist = []

        for p in range(numLeg):
           for i in range(numsubleg):

               self.leg[p].sub[i].omega += self.leg[p].sub[i].alpha * dtime #omega를 시간에 따라 갱신

               self.leg[p].sub[i].theta += self.leg[p].sub[i].omega * dtime #theta를 시간에 따라 갱신

               self.leg[p].sub[i].Q = tf.scalar_mul(tf.cos(self.leg[p].sub[i].theta), tf.eye(3, dtype=tf.float32)) + \
               tf.scalar_mul(1.-tf.cos(self.leg[p].sub[i].theta), tf.matmul(self.leg[p].sub[i].axis, self.leg[p].sub[i].axis, transpose_a = True)) + \
               tf.scalar_mul(tf.sin(self.leg[p].sub[i].theta), tf.cross(tf.tile(self.leg[p].sub[i].axis,[3,1]), tf.eye(3, dtype=tf.float32)))

           Qs = [tf.matmul(self.leg[p].sub[0].Q , self.body.Q)] #Qs는 i번째 subleg에서 space로의 좌표변환
           #List of rotation matrices of each sublegs in space frame
           #Type : list of [3,3] Tensor
           for i in range(1,numsubleg):
               Qs.append(tf.matmul(self.leg[p].sub[i].Q , Qs[i-1]))

           e = [tf.matmul(self.leg[p].sub[i].axis, Qs[i]) for i in range(numsubleg)]
           #List of axes of each sublegs in space frame
           #Type : list of [None,3] Tensor
           
           Qw = [tf.scalar_mul(self.leg[p].sub[i].omega,e[i]) for i in range(numsubleg)]

           ws = [wbs+Qw[0]]
           for i in range(1,numsubleg): 
               ws.append(ws[i-1] + Qw[i])

           ls = [[tf.matmul(self.leg[p].sub[i].l[0],Qs[i]), 
                  tf.matmul(self.leg[p].sub[i].l[1],Qs[i])] for i in range(numsubleg)] #ls = 2Dtensor

           lbtomotbs = tf.matmul(self.body.lbtomot[p] , self.body.Q) # lbtomotbs = 2Dtensor

           lbtomots = [ lbtomotbs + ls[0][0] ] # lbtomots = 2Dtensor

           for i in range(1,numsubleg):
               lbtomots.append(lbtomots[i-1]+ls[i-1][1]+ls[i][0])
               
           #Calculating External Forces
           vstmp = self.body.vs + tf.cross(wbs, lbtomotbs)
           NormalScale = 2000.0
           TanhConst = 100.0

           Zfilter = tf.constant([[0.,0.,1.]])
           negZfilter = tf.constant([[0.,0.,-1.]])
           XYfilter = tf.constant([[1.,1.,0.]])
           for i in range(numsubleg):
               Fz_primi = tf.multiply( negZfilter, self.body.rs + lbtomots[i] + ls[i][1] )

               Fz_primi_cube = tf.multiply( Fz_primi, tf.multiply ( Fz_primi, Fz_primi ) )
               
               Fz = tf.scalar_mul( NormalScale, tf.nn.relu( Fz_primi ) )
               vstmp += tf.cross( ws[i], ls[i][0] + ls[i][1] )
               vstmp_z = tf.reshape(tf.matmul( Zfilter , vstmp , transpose_b = True ), [])

               Vscale = 3. - tf.nn.tanh( TanhConst * vstmp_z )

               Fnormal = tf.scalar_mul( Vscale, Fz )

               Flist.append( Fnormal )

               vstmp_xy= tf.multiply( XYfilter, vstmp )
               Ffric = tf.scalar_mul( -Fricscale, tf.scalar_mul(vstmp_z, vstmp_xy) )
               
               Flist.append( Ffric )
               
               Feqc += Fnormal
               Feqc += Ffric
               #Teqc+=


           tot_lbtomots += lbtomots
        asb = tf.scalar_mul(Mtotinv, Feqc)
        alphab = tf.matmul(tf.matmul(Teqc, self.body.Q, transpose_b = True), Ibinv)
        self.body.wb += tf.scalar_mul(dtime, alphab)
        self.body.Q += tf.scalar_mul(dtime,tf.cross(tf.concat([wbs, wbs, wbs], axis = 0),self.body.Q))
        self.body.vs += tf.scalar_mul(dtime,asb)
        self.body.rs += tf.scalar_mul(dtime,self.body.vs)
        # Q to quaternion
        
        qw = tf.scalar_mul(0.5, tf.sqrt(tf.reduce_sum(tf.diag_part(self.body.Q))+1.))
        qv = tf.reduce_sum(tf.cross(self.body.Q, tf.eye(3, dtype = tf.float32)), axis = 0)/tf.scalar_mul(4., qw)

        # quaternion normalization
        '''
        qvsquare = tf.reduce_sum(tf.square(qv))
        qnorm = tf.sqrt(tf.square(qw)+qvsquare)
        qw /= qnorm
        qv /= qnorm
        # quaternion to Q

        self.body.Q = tf.scalar_mul(qw*qw-qvsquare,tf.eye(3, dtype = tf.float32))\
            + 2 * tf.matmul(tf.reshape(qv, [3, 1]), tf.reshape(qv, [1, 3]))\
            - 2 * qw * tf.cross(tf.tile(tf.reshape(qv, [1,3]), [3,1]), tf.eye(3, dtype = tf.float32))
        '''
        return Flist, asb, Qs, [x + self.body.rs for x in tot_lbtomots]

R = robot()
R.set_constants()
print("set constant")

R.body.rs = prs
R.body.vs = pvs
R.body.wb = pwb
R.body.Q = pQb

Flist, asbR, QsR, return_val = R.timeflow()
detQ = tf.cast(tf.matrix_determinant(R.body.Q), tf.float32)
#R.body.Q = tf.scalar_mul(1/tf.pow(detQ, 1/3.),R.body.Q)
sess=tf.Session()
tf.global_variables_initializer()
nowrs = np.array([[0.,0.,0.15]])
nowvs = np.array([[0.,0.,0.]])
nowwb = np.array([[0.0,0.0,0.3]])
nowQb = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

[nowrs, nowvs, nowwb, nowQb] = sess.run([R.body.rs,R.body.vs,R.body.wb ,R.body.Q] ,feed_dict={prs: nowrs, pvs:nowvs, pwb: nowwb, pQb: nowQb})
plt.ion()
fig = plt.figure(figsize=(8,8))
for i in range(100000):
    [nowrs, nowvs, nowwb, nowQb, asbSess, MWT] = sess.run([R.body.rs,R.body.vs,R.body.wb ,R.body.Q, asbR, return_val] , feed_dict={ prs: nowrs, pvs:nowvs, pwb: nowwb, pQb: nowQb})
    #print("nowvs(%d) = "%i,nowvs)
    #print("nowQb(%d) = "%i,nowQb);
    if(i%1==0):
        Flist1, MWT = sess.run( [Flist,return_val] , feed_dict={prs: nowrs, pvs:nowvs, pwb: nowwb, pQb: nowQb})
        print(*Flist1,sep='\n',end='\n\n')
        #print(Momentum)
        pflat = np.reshape(MWT, [-1])
        ax = fig.add_subplot(111,projection='3d')
        S = ax.scatter(pflat[0::3],pflat[1::3],pflat[2::3])
        ax.set_xlim3d(-0.5,0.5)
        ax.set_ylim3d(-0.5,0.5)
        ax.set_zlim3d(-0.,0.5)
        plt.title(str(dtime*i)+'s')
        plt.draw()
        plt.pause(0.001)
        plt.clf()