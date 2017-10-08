import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.axes as yaxes
fp = open("motion1.txt","r")
def rv():
    L = fp.readline().split(',')
    L[0] = L[0][1:]
    L[-1] = L[-1][:len(L[-1])-2]
    return list(map(float,L))
rl = lambda : list(map(float,fp.readline().split()))
line = []
T=[]
X=[]
Y=[]
Z=[]
while True:
    line = rl()
    if not line : break
    T.append(line)
    for _ in range(17):
        line = rv()
        X.append(line[0])
        Y.append(line[1])
        Z.append(line[2])

fig = plt.figure()
c = 100
timen = 20
for k in range(c):
    if k*timen >= len(T): break
    #plt.add_subplot(111,projection='3d')
    ax = fig.add_subplot(111,projection='3d')
    ax.scatter(X[timen*17*k:timen*17*k+17],Y[timen*17*k:timen*17*k+17],Z[timen*17*k:timen*17*k+17])
    ax.set_zlim(0,0.5)
    ax.set_xlim(X[timen*17*k]-0.5,X[timen*17*k]+0.5)
    ax.set_ylim(Y[timen*17*k]-0.5,Y[timen*17*k]+0.5)
    plt.title(str(T[k*timen]))
    plt.draw()
    """
    plt.gca(projection = '3d')
    plt.set_zlim(0,0.5)
    plt.plot(X[timen*17*k:timen*17*(k+1)],Y[timen*17*k:timen*17*(k+1)],Z[timen*17*k:timen*17*(k+1)])
    plt.title(str(T[k]))
    plt.draw()
    """
    plt.pause(0.001)
    plt.clf()
plt.show(block = True)

fp.close()
