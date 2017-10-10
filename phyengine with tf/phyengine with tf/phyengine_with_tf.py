import tensorflow as tf
import numpy as np

numsubleg = 3
numLeg = 4


global_step = tf.Variable(0,trainable = False, name = 'global_step')

Destination = tf.placeholder(tf.float32, [None,3])

class Force:
    def __init__(self,F=tf.zeros(3),r=tf.zeros(3)):
        self.F = F
        self.r = r

class Rigidbody:
    def __init__(self,m=0.0,Q=tf.zeros((3,3)),Ib=tf.zeros((3,3)),wb=tf.zeros((3))):
        self.m = m
        self.Q = Q
        self.Ib = Ib
        self.wb = wb
       
class Robotbody(Rigidbody):
    def __init__(self,m=0.0,rs=tf.zeros((3)),vs=tf.zeros((3)),lbtomot=tf.zeros((4,3))):
        Rigidbody.__init__(self,m=m)
        self.rs = rs
        self.vs = vs
        self.lbtomot = lbtomot

class subleg(Rigidbody):
    def __init__(self,m=0.0,axis=tf.zeros((3)),l=[tf.zeros((3)),tf.zeros((3))],theta=0.0,omega=0.0,alpha=0.0):
        Rigidbody.__init__(self,m=m)
        self.axis = axis
        self.l = l
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
    def setalpha(self,t = 0.0):
        print('YAY')
    def timeflow(self,t = 0.0):
        self.setalpha(t)
        for p in range(numLeg):
           Qs = [tf.matmul(self.body.Q,self.leg[p].sub[0].Q)]
           #List of rotation matrices of each sublegs in space frame
           #Type : list of [3,3] Tensor
           
           for i in range(1,numsubleg):
               Qs.append(tf.matmul(Qs[i-1],self.leg[p].sub[i].Q))
           

           e = [tf.matmul(Qs[i],tf.reshape( self.leg[p].sub[i].axis, [3,1])) for i in range(numsubleg)]
           #List of axes of each sublegs in space frame
           #Type : list of [3,1] Tensor

           Qalpha = [tf.scalar_mul(self.leg[p].sub[i].alpha,e[i]) for i in range(numsubleg)]

           Qw = [tf.scalar_mul(self.leg[p].sub[i].omega,e[i]) for i in range(numsubleg)]

           wbs = tf.matmul(self.body.Q , tf.reshape(self.body.wb,[3,1])) #[3,1] matrix

           ws = [wbs+Qw[0]]
           for i in range(1,numsubleg): 
               ws.append(ws[i-1]+Qw[i])

           w = [tf.matmul(tf.transpose(Qs[i]),tf.reshape(ws[i],[3,1])) for i in range(numsubleg)]

           ls = [[tf.matmul(Qs[i],tf.reshape(self.leg[p].sub[i].l[0],[3,1])), 
                  tf.matmul(Qs[i],tf.reshape(self.leg[p].sub[i].l[1],[3,1]))] for i in range(numsubleg)] #ls = 2Dtensor

           lbtomotbs = tf.matmul(self.body.Q,tf.reshape(self.body.lbtomot[p],[3,1])) # lbtomotbs = 2Dtensor
           lbtomots = [tf.add( lbtomotbs , ls[0][0] )] # lbtomots = 2Dtensor

           for i in range(1,numsubleg):
               lbtomots.append(tf.add(lbtomots[i-1],
                                      tf.add(
                                          ls[i-1][1],
                                          ls[i][0]
                                          )))

           A = [tf.add(
                tf.cross(wbs,tf.cross(wbs,lbtomotbs)),
                tf.add(
                tf.cross(Qalpha[0],lbtomots[0]),
                tf.cross(ws[0],tf.cross(ws[0],lbtomots[0]))

                     ))]
           for i in range(1,numsubleg):
               A.append(
                   tf.add(
                        tf.cross( Qalpha[i-1], tf.add(  ) )
                       )
                   )

R = robot()
R.timeflow()
sess=tf.Session()