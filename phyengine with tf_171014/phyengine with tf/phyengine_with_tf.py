import tensorflow as tf
import numpy as np

numsubleg = 3
numLeg = 4
Mtot = 0.9849
dtime = 0.001
Fupscale = 3.
Fdownscale = 1.5
Fricscale = Mtot*9.81*1.
g = tf.constant([[0,0,-9.81]],dtype=tf.float64)
Fup = tf.constant([[0,0,Mtot*Fupscale*9.81]],dtype=tf.float64)
Fdown = tf.constant([[0,0,Mtot*Fdownscale*9.81]],dtype=tf.float64)
Fadded = tf.constant([[0,0,Mtot*(Fupscale+Fdownscale)*9.81/2.]],dtype=tf.float64)
Fsubed = tf.constant([[0,0,Mtot*(Fupscale-Fdownscale)*9.81]],dtype=tf.float64)  
Offset = tf.constant([[0,0,0.5]],dtype=tf.float64)
MxMyFilter = tf.constant([[-1.,-1.,0]],dtype=tf.float64)

#Variables
global_step = tf.Variable(0,trainable = False, name = 'global_step')


# 다리 정지해있는 물리엔진용 placeholders 잡기. 신경망에서는 지워야함
prs = tf.placeholder(tf.float64, [1,3])
pvs = tf.placeholder(tf.float64, [1,3])
pwb = tf.placeholder(tf.float64, [1,3])
pQb = tf.placeholder(tf.float64, [3,3])


Destination = tf.placeholder(tf.float64, [None,3])
rs_deprec = tf.placeholder(tf.float64, [None,3])

class Rigidbody:
    def __init__(self,m=0.0,Q=tf.zeros((3,3),dtype=tf.float64),Ib=tf.zeros((3,3),dtype=tf.float64),wb=tf.zeros((1,3),dtype=tf.float64)):
        self.m = m
        self.Q = Q
        self.Ib = Ib
        self.wb = wb
       
class Robotbody(Rigidbody):
    def __init__(self,m=0.0,rs=tf.zeros((1,3),dtype=tf.float64),vs=tf.zeros((1,3),dtype=tf.float64),lbtomot=[tf.zeros((1,3),dtype=tf.float64) for _ in range(numLeg)]):
        Rigidbody.__init__(self,m=m)
        self.rs = rs
        self.vs = vs
        self.lbtomot = lbtomot

class subleg(Rigidbody):
    def __init__(self,m=0.0,axis=tf.zeros((1,3),dtype=tf.float64),l=[tf.zeros((1,3),dtype=tf.float64),tf.zeros((1,3),dtype=tf.float64)],theta=0.0,omega=0.0,alpha=0.0):
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
    
    def set_constants(self):
        #Set Axes
        self.leg[0].sub[0].axis = tf.constant([[0.,1.,0.]],dtype=tf.float64)
        self.leg[0].sub[1].axis = tf.constant([[1.,0.,0.]],dtype=tf.float64)
        self.leg[0].sub[2].axis = tf.constant([[1.,0.,0.]],dtype=tf.float64)

        self.leg[1].sub[0].axis = tf.constant([[0.,-1.,0.]],dtype=tf.float64)
        self.leg[1].sub[1].axis = tf.constant([[1.,0.,0.]],dtype=tf.float64)
        self.leg[1].sub[2].axis = tf.constant([[1.,0.,0.]],dtype=tf.float64)

        self.leg[2].sub[0].axis = tf.constant([[0.,1.,0.]],dtype=tf.float64)
        self.leg[2].sub[1].axis = tf.constant([[-1.,0.,0.]],dtype=tf.float64)
        self.leg[2].sub[2].axis = tf.constant([[-1.,0.,0.]],dtype=tf.float64)

        self.leg[3].sub[0].axis = tf.constant([[0.,-1.,0.]],dtype=tf.float64)
        self.leg[3].sub[1].axis = tf.constant([[-1.,0.,0.]],dtype=tf.float64)
        self.leg[3].sub[2].axis = tf.constant([[-1.,0.,0.]],dtype=tf.float64)

        #Set lvectors
        self.body.lbtomot[0] = tf.constant([[0.065,0.065,0.02]],dtype=tf.float64)
        self.body.lbtomot[1] = tf.constant([[-0.065,0.065,0.02]],dtype=tf.float64)
        self.body.lbtomot[2] = tf.constant([[0.065,-0.065,0.02]],dtype=tf.float64)
        self.body.lbtomot[3] = tf.constant([[-0.065,-0.065,0.02]],dtype=tf.float64)

        self.leg[0].sub[0].l[0] = tf.constant([[0.,0.020,0.0]],dtype=tf.float64)
        self.leg[0].sub[0].l[1] = tf.constant([[0.,0.,-0.015]],dtype=tf.float64)
        self.leg[0].sub[1].l[0] = tf.constant([[0.,0.033,0.]],dtype=tf.float64)
        self.leg[0].sub[1].l[1] = tf.constant([[0.,0.033,0.]],dtype=tf.float64)
        self.leg[0].sub[2].l[0] = tf.constant([[0.,0.030,0.]],dtype=tf.float64)
        self.leg[0].sub[2].l[1] = tf.constant([[0.,0.070,0.]],dtype=tf.float64)

        self.leg[1].sub[0].l[0] = tf.constant([[0.,0.020,0.0]],dtype=tf.float64)
        self.leg[1].sub[0].l[1] = tf.constant([[0.,0.,-0.015]],dtype=tf.float64)
        self.leg[1].sub[1].l[0] = tf.constant([[0.,0.033,0.]],dtype=tf.float64)
        self.leg[1].sub[1].l[1] = tf.constant([[0.,0.033,0.]],dtype=tf.float64)
        self.leg[1].sub[2].l[0] = tf.constant([[0.,0.030,0.]],dtype=tf.float64)
        self.leg[1].sub[2].l[1] = tf.constant([[0.,0.070,0.]],dtype=tf.float64)

        self.leg[2].sub[0].l[0] = tf.constant([[0.,-0.020,0.0]],dtype=tf.float64)
        self.leg[2].sub[0].l[1] = tf.constant([[0.,0.,-0.015]],dtype=tf.float64)
        self.leg[2].sub[1].l[0] = tf.constant([[0.,-0.033,0.]],dtype=tf.float64)
        self.leg[2].sub[1].l[1] = tf.constant([[0.,-0.033,0.]],dtype=tf.float64)
        self.leg[2].sub[2].l[0] = tf.constant([[0.,-0.030,0.]],dtype=tf.float64)
        self.leg[2].sub[2].l[1] = tf.constant([[0.,-0.070,0.]],dtype=tf.float64)

        self.leg[3].sub[0].l[0] = tf.constant([[0.,-0.020,0.0]],dtype=tf.float64)
        self.leg[3].sub[0].l[1] = tf.constant([[0.,0.,-0.015]],dtype=tf.float64)
        self.leg[3].sub[1].l[0] = tf.constant([[0.,-0.033,0.]],dtype=tf.float64)
        self.leg[3].sub[1].l[1] = tf.constant([[0.,-0.033,0.]],dtype=tf.float64)
        self.leg[3].sub[2].l[0] = tf.constant([[0.,-0.030,0.]],dtype=tf.float64)
        self.leg[3].sub[2].l[1] = tf.constant([[0.,-0.070,0.]],dtype=tf.float64)

        #set Mass
        for p in range(numLeg):
            self.leg[p].sub[0].m = tf.constant(0.0550,dtype=tf.float64) #kg
            self.leg[p].sub[1].m = tf.constant(0.0294,dtype=tf.float64)
            self.leg[p].sub[2].m = tf.constant(0.0709,dtype=tf.float64)

            #set Inertia tensors
            self.leg[p].sub[0].Ib = tf.constant([[1.62e-5,0.,0.],
                                                 [0.,1.87e-5,0.],
                                                 [0.,0.,1.20e-5]],dtype=tf.float64)

            self.leg[p].sub[1].Ib = tf.constant([[0.60e-5,0.,0.],
                                                 [0.,0.30e-5,0.],
                                                 [0.,0.,0.60e-5]],dtype=tf.float64)

            self.leg[p].sub[2].Ib = tf.constant([[2.00e-5,0.,0.],
                                                 [0.,1.40e-5,0.],
                                                 [0.,0.,2.20e-5]],dtype=tf.float64)
            #set Initial theta conditions
            for i in range(numsubleg):
                self.leg[p].sub[i].theta = tf.constant(0., dtype=tf.float64)
            self.leg[p].sub[1].theta = tf.constant(np.pi / 2., dtype=tf.float64)

        self.body.m = (0.2676 + 0.1414) \
                      +self.leg[0].sub[0].m \
                      -self.leg[0].sub[1].m \
                      -self.leg[0].sub[2].m \
                      +(0.1583 + 0.2358)

        Mtot=self.body.m + 4 * (self.leg[0].sub[0].m+self.leg[0].sub[1].m+self.leg[0].sub[2].m)
        self.body.Ib = tf.constant([[75.0e-5,0.,0.],
                                    [0.,75.0e-5,0.],
                                    [0.,0.,50.0e-5]],dtype=tf.float64)


        
    def setalpha(self,t = 0.0):
        return None
    def timeflow(self,t = 0.0):
        self.setalpha(t)
        Feqc = tf.scalar_mul(Mtot,g)
        Feqa = tf.diag([Mtot,Mtot,Mtot])
        Crossvec = tf.zeros((1,3),dtype=tf.float64)
        Teqalpha = tf.zeros((3,3),dtype=tf.float64)
        Teqc = tf.zeros((1,3),dtype=tf.float64)
        mlsum = tf.zeros((1,3),dtype=tf.float64)
        sumDs = tf.zeros((3,3),dtype=tf.float64)
        wbs = tf.matmul(self.body.wb, self.body.Q) #[1,3] matrix

        for p in range(numLeg):
           Qs = [tf.matmul(self.leg[p].sub[0].Q , self.body.Q)]
           #List of rotation matrices of each sublegs in space frame
           #Type : list of [3,3] Tensor
           
           for i in range(1,numsubleg):
               Qs.append(tf.matmul(self.leg[p].sub[i].Q , Qs[i-1]))
           

           Is = [ tf.matmul( tf.matmul( Qs[i] , self.leg[p].sub[i].Ib , transpose_a = True) , Qs[i]) for i in range(numsubleg) ]\

           e = [tf.matmul(self.leg[p].sub[i].axis, Qs[i]) for i in range(numsubleg)]
           #List of axes of each sublegs in space frame
           #Type : list of [None,3] Tensor

           Qalpha = [tf.scalar_mul(self.leg[p].sub[i].alpha,e[i]) for i in range(numsubleg)]
           
           Qalphasum = [ Qalpha[0] ]
           for i in range(1,numsubleg): Qalphasum.append( Qalphasum[i-1] + Qalpha[i] )

           Qw = [tf.scalar_mul(self.leg[p].sub[i].omega,e[i]) for i in range(numsubleg)]

           ws = [wbs+Qw[0]]
           for i in range(1,numsubleg): 
               ws.append(ws[i-1]+Qw[i])

           w = [tf.matmul(ws[i],Qs[i], transpose_b = True) for i in range(numsubleg)]

           ls = [[tf.matmul(self.leg[p].sub[i].l[0],Qs[i]), 
                  tf.matmul(self.leg[p].sub[i].l[1],Qs[i])] for i in range(numsubleg)] #ls = 2Dtensor

           lbtomotbs = tf.matmul(self.body.lbtomot[p] , self.body.Q) # lbtomotbs = 2Dtensor
           lbtomots = [ lbtomotbs + ls[0][0] ] # lbtomots = 2Dtensor
           for i in range(1,numsubleg):
               lbtomots.append(lbtomots[i-1]+ls[i-1][1]+ls[i][0])
           for i in range(numsubleg):
               mlsum += tf.scalar_mul(self.leg[p].sub[i].m, lbtomots[i])

           #Calculating External Forces
           vs = self.body.vs
           for i in range(numsubleg):
               Collisiontemp = tf.cast(tf.less(lbtomots[i]+self.body.rs,tf.zeros((1,3),dtype=tf.float64)),tf.float64)
               Collisionz = tf.multiply(Collisiontemp, tf.constant([[0,0,1]], tf.float64))
               Collisionxy = tf.matmul(Collisionz, tf.constant([[0,0,0],[0,0,0],[1,1,0]], tf.float64))##더 연산량을 줄일 수 있을 듯 방법을 강구하라
               vs += tf.cross(ws[i], ls[i][0]+ls[i][1])
               vCollision = tf.cast(tf.less( vs , tf.zeros((1,3),dtype=tf.float64) ),tf.float64)
               Ftemp = tf.multiply(Collisionz, Fadded + tf.multiply( (vCollision - Offset) , Fsubed ))
               Feqc += Ftemp
               Teqc += tf.cross( lbtomots[i] , Ftemp )
               FrictionTemp = -tf.multiply(tf.scalar_mul( Fricscale , vs ), Collisionxy)##########하.. 힘이 너무 다 틀렸어
               Feqc += FrictionTemp
               Teqc += tf.cross( lbtomots[i], FrictionTemp )
           
           A = [tf.cross(wbs,tf.cross(wbs,lbtomotbs)) 
                + tf.cross(Qalpha[0],lbtomots[0]) 
                + tf.cross(ws[0], tf.cross(ws[0],lbtomots[0]))]

           for i in range(1,numsubleg):
               A.append(
                        tf.cross( Qalpha[i-1], ls[i-1][1] )
                        +tf.cross( ws[i], tf.cross(ws[i], ls[i][0]))
                        +tf.cross( Qalpha[i], ls[i][0])
                   )

           mlsquare = tf.zeros((1),dtype=tf.float64)
           for i in range(numsubleg):
               mlsquare += tf.scalar_mul(self.leg[p].sub[i].m, 
                                         tf.matmul(lbtomots[i],lbtomots[i],transpose_b=True))
           mlsquare = tf.reshape(mlsquare,[-1])

           for i in range(numsubleg):
               Dya = tf.scalar_mul(self.leg[p].sub[i].m,tf.matmul(lbtomots[i],lbtomots[i],transpose_a=True))
           
           Ds = tf.diag(tf.concat([mlsquare,mlsquare,mlsquare], axis=0)) - Dya
           Teqalpha += Ds 
           sumDs += Ds
           #Qb * Ib * Qb.transpose()


           for i in range(numsubleg):
               Feqc -= tf.scalar_mul(self.leg[p].sub[i].m,A[i])
               Crossvec += tf.scalar_mul(self.leg[p].sub[i].m,lbtomots[i])
               Teqc += tf.matmul( tf.cross( tf.matmul(w[i] , self.leg[p].sub[i].Ib) , w[i] ), Qs[i])
               Teqc -= tf.matmul( Qalphasum[i] , Is[i])
               Teqalpha += Is[i]
               #Qs_i * I_i * Qs_i^T

           #leg update
           #float32 -> float64 conversion : 171013 Fine
           for i in range(numsubleg):
               self.leg[p].sub[i].omega += self.leg[p].sub[i].alpha * dtime
               self.leg[p].sub[i].theta += self.leg[p].sub[i].omega * dtime
               self.leg[p].sub[i].Q = tf.scalar_mul(tf.cos(self.leg[p].sub[i].theta), tf.eye(3, dtype=tf.float64)) + \
               tf.scalar_mul(1.-tf.cos(self.leg[p].sub[i].theta), tf.matmul(self.leg[p].sub[i].axis,self.leg[p].sub[i].axis,transpose_a = True)) + \
               tf.scalar_mul(tf.cos(self.leg[p].sub[i].theta) , tf.cross(tf.concat([self.leg[p].sub[i].axis,
                                                                                    self.leg[p].sub[i].axis,
                                                                                    self.leg[p].sub[i].axis],axis=0)
                                                                                   ,tf.eye(3, dtype=tf.float64)))
               #update 'Q's of leg - 20171012 fine
        Teqalpha += tf.matmul( tf.matmul( self.body.Q , self.body.Ib , transpose_a = True) , self.body.Q)
        Teqc += tf.matmul( tf.cross( tf.matmul( self.body.wb , self.body.Ib ), self.body.wb) , self.body.Q)
        Teqc += tf.cross(mlsum, g)
        Teqanorm = tf.reshape(tf.matmul(mlsum, mlsum, transpose_b = True),[-1])
        alphabs = tf.matmul( 
        Teqc - tf.scalar_mul(1./Mtot, tf.cross(mlsum,Feqc)),
        tf.matrix_inverse(
            Teqalpha - tf.scalar_mul(1./Mtot , 
            tf.diag(tf.concat([Teqanorm,Teqanorm,Teqanorm], axis=0)) - tf.matmul(mlsum,mlsum,transpose_a = True))
        ))
        asb = tf.scalar_mul(1./Mtot, Feqc - tf.cross(mlsum,alphabs))
        self.body.wb += tf.scalar_mul(dtime, alphabs)
        self.body.Q += tf.scalar_mul(dtime,tf.cross(tf.concat([wbs, wbs,wbs], axis = 0),self.body.Q))
        self.body.vs+=tf.scalar_mul(dtime,asb)
        self.body.rs+=tf.scalar_mul(dtime,self.body.vs)
        MWT=asb
        return MWT
R = robot()
R.set_constants()
print("set constant")
print(1)
R.body.rs = tf.constant([0,0,0.3],dtype=tf.float64)
R.body.Q=tf.constant([[1,0,0],[0,1,0],[0,0,1]], dtype=tf.float64)

R.body.rs = prs
R.body.vs = pvs
R.body.wb = pwb

return_val = R.timeflow()

sess=tf.Session()
tf.global_variables_initializer()
nowrs = np.ones((1,3))
nowvs = np.zeros((1,3))
nowwb = np.zeros((1,3))
nowQb = np.zeros((3,3))
[nowrs, nowvs, nowwb, nowQb] = sess.run([R.body.rs,R.body.vs,R.body.wb ,R.body.Q] ,feed_dict={Destination:[[0,0,0]], prs: nowrs, pvs:nowvs, pwb: nowwb, pQb: nowQb})
for i in range(100000):
    [nowrs, nowvs, nowwb, nowQb] = sess.run([R.body.rs,R.body.vs,R.body.wb ,R.body.Q] , feed_dict={Destination:[[0,0,0]], prs: nowrs, pvs:nowvs, pwb: nowwb, pQb: nowQb})
    if(i%100==0):
        print("time = ", i*dtime)
        print( "body.rs = ", nowrs)
        print()