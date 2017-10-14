import tensorflow as tf
import numpy as np

numsubleg = 3
numLeg = 4
Mtot = 0.5
dtime = 0.01
Fupscale = 3.
Fdownscale = 1.
Fricscale = Mtot*9.81*100.
g = tf.constant([[0,0,-9.81]],dtype=tf.float64)
Fup = tf.constant([[0,0,Mtot*Fupscale*9.81]],dtype=tf.float64)
Fdown = tf.constant([[0,0,Mtot*Fdownscale*9.81]],dtype=tf.float64)
Fadded = tf.constant([[0,0,Mtot*(Fupscale+Fdownscale)*9.81/2.]],dtype=tf.float64)
Fsubed = tf.constant([[0,0,Mtot*(Fupscale-Fdownscale)*9.81]],dtype=tf.float64)  
Offset = tf.constant([[0,0,0.5]],dtype=tf.float64)
MxMyFilter = tf.constant([[-1.,-1.,0]],dtype=tf.float64)
global_step = tf.Variable(0,trainable = False, name = 'global_step')

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

            self.leg[p].sub[1].theta = np.pi / 2.

        self.body.m = (0.2676 + 0.1414) \
                      +self.leg[0].sub[0].m \
                      -self.leg[0].sub[1].m \
                      -self.leg[0].sub[2].m \
                      +(0.1583 + 0.2358)

        self.body.Ib = tf.constant([[75.0e-5,0.,0.],
                                    [0.,75.0e-5,0.],
                                    [0.,0.,50.0e-5]],dtype=tf.float64)

        self.body.rs += tf.constant([0,0,0.25],dtype=tf.float64)

        
    def setalpha(self,t = 0.0):
        return None
    def timeflow(self,t = 0.0):
        self.setalpha(t)
        Feqc = tf.scalar_mul(Mtot,g)
        #print("Feqc = mg = ",Feqc)
        Feqa = tf.diag([Mtot,Mtot,Mtot])
        Crossvec = tf.zeros((1,3),dtype=tf.float64)
        Teqalpha = tf.zeros((3,3),dtype=tf.float64)
        Teqc = tf.zeros((1,3),dtype=tf.float64)
        mlsum = tf.zeros((1,3),dtype=tf.float64)
        sumDs = tf.zeros((3,3),dtype=tf.float64)
        wbs = tf.matmul(self.body.wb, self.body.Q) #[1,3] matrix
        print(wbs)
        print("sssss")

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
           #print(mlsum)
           for i in range(numsubleg):
               mlsum += tf.scalar_mul(self.leg[p].sub[i].m, lbtomots[i])
           #print(lbtomots[0])
           #print("after adding lbtomots : ",mlsum)

           #Calculating External Forces
           for i in range(numsubleg):
               Collision = tf.cast(tf.less(lbtomots[i]-self.body.rs,tf.zeros((1,3),dtype=tf.float64)),tf.float64)
               vs = tf.cross(ws[i], lbtomots[i])
               vCollision = tf.cast(tf.less( vs , tf.zeros((1,3),dtype=tf.float64) ),tf.float64)
               Ftemp = tf.multiply(Collision, Fadded + tf.multiply( (vCollision - Offset) , Fsubed ))
               Feqc += Ftemp
               Teqc += tf.cross( lbtomots[i]-self.body.rs , Ftemp )
               print(lbtomots[0])
               FrictionTemp = tf.scalar_mul( Fricscale , tf.multiply( MxMyFilter, vs ) )
               Feqc += FrictionTemp
               Teqc += tf.cross( lbtomots[i]-self.body.rs , FrictionTemp )
           
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
           #print(Teqalpha)
           Teqalpha += Ds 
           sumDs += Ds
           #print(Teqalpha)
           #Qb * Ib * Qb.transpose()



           #print(Ds)
           #print("QIw^2 = ",tf.matmul( Qs[i], tf.cross( tf.matmul(self.leg[p].sub[i].Ib, w[i]) , w[i] )))
           #print("IQa = ",tf.matmul( Is[i], Qalphasum[i] ))
           for i in range(numsubleg):
               Feqc -= tf.scalar_mul(self.leg[p].sub[i].m,A[i])
               Crossvec += tf.scalar_mul(self.leg[p].sub[i].m,lbtomots[i])
               Teqc += tf.matmul( tf.cross( tf.matmul(w[i] , self.leg[p].sub[i].Ib) , w[i] ), Qs[i])
               Teqc -= tf.matmul( Qalphasum[i] , Is[i])
               print("cross = ", tf.cross( tf.matmul(w[i] , self.leg[p].sub[i].Ib) , w[i] ))
               Teqalpha += Is[i]
               #Qs_i * I_i * Qs_i^T

           #leg update
           #float32 -> float64 conversion : 171013 Fine
           for i in range(numsubleg):
               self.leg[p].sub[i].omega += self.leg[p].sub[i].alpha * dtime
               self.leg[p].sub[i].theta += self.leg[p].sub[i].omega * dtime
               self.leg[p].sub[i].Q = tf.scalar_mul(tf.cos(self.leg[p].sub[i].theta), tf.eye(3),dtype=tf.float64) + \
               tf.scalar_mul(1.-tf.cos(self.leg[p].sub[i].theta), tf.matmul(self.leg[p].sub[i].axis,self.leg[p].sub[i].axis,transpose_a = True),dtype=tf.float64) + \
               tf.scalar_mul(tf.cos(self.leg[p].sub[i].theta) , tf.cross(tf.concat([self.leg[p].sub[i].axis,
                                                                                    self.leg[p].sub[i].axis,
                                                                                    self.leg[p].sub[i].axis],axis=0)
                                                                                   ,tf.eye(3)),dtype=tf.float64)
               print(tf.concat([self.leg[p].sub[i].axis,self.leg[p].sub[i].axis,self.leg[p].sub[i].axis],axis=0))
               #print("Qmat = ",self.leg[p].sub[i].Q)
               #update 'Q's of leg - 20171012 fine
        #print("Teqc out loop",Teqc);
        print("asdfasdf",self.body.wb)
        #print("out of loop",Teqalpha)
        Teqalpha += tf.matmul( tf.matmul( self.body.Q , self.body.Ib , transpose_a = True) , self.body.Q)
        #print(Teqalpha)
        print("teqc2=",Teqc)
        Teqc += tf.matmul( tf.cross( tf.matmul( self.body.wb , self.body.Ib ), self.body.wb) , self.body.Q)
        print("teqc2=",Teqc)
        print(tf.cross(tf.matmul( self.body.wb, self.body.Ib ) ,self.body.wb, name = "mac"))
        #print("Teqc final ",Teqc);
        #print(g)
        #print(mlsum)
        #print("mg = ",tf.cross(mlsum, g))
        Teqc += tf.cross(mlsum, g)
        print("mlsum",mlsum)
        #print("Teqc = ",Teqc);
        Teqanorm = tf.reshape(tf.matmul(mlsum, mlsum, transpose_b = True),[-1])
        #print(mlsum)
        #print(Teqanorm)
        #print(tf.cross(mlsum,Feqc))
        print("teqc=",Teqc)
        alphabs = tf.matmul( 
        Teqc - tf.scalar_mul(1./Mtot, tf.cross(mlsum,Feqc)),
        tf.matrix_inverse(
            Teqalpha - tf.scalar_mul(1./Mtot , 
            tf.diag(tf.concat([Teqanorm,Teqanorm,Teqanorm], axis=0)) - tf.matmul(mlsum,mlsum,transpose_a = True))
        ))
        #print("alphabs = ",alphabs)
        #print("Feqc = ",Feqc)
        asb = tf.scalar_mul(1./Mtot, Feqc - tf.cross(mlsum,alphabs))
        #print("asb = ",asb)
        print("alphabs", alphabs)
        self.body.wb += tf.scalar_mul(dtime, alphabs)
        self.body.Q += tf.scalar_mul(dtime,tf.cross(tf.concat([wbs, wbs,wbs], axis = 0),self.body.Q))
        self.body.vs+=tf.scalar_mul(dtime,asb)
        self.body.rs+=tf.scalar_mul(dtime,self.body.vs)

        return sumDs
R = robot()
R.set_constants()
return_val = R.timeflow()
return_val = R.timeflow()
for _ in range(1000):
    sess=tf.Session()
    print( "Ds = ", sess.run(return_val, feed_dict={Destination:[[0,0,0]]}))
