#include <bits/stdc++.h>
using namespace std;
using phys = double;

//for Cylinder
const double M=10;
const double h=0.5;
const double R=0.01;
const double g=9.81;
const double pi = 3.14159265358979323846264338;
const phys dtime = 5e-3;

struct Vector{
    phys x,y,z;
    Vector(phys x=0.L,phys y=0.L, phys z=0.L):x(x),y(y),z(z){}
    Vector operator+ (Vector w){
        return Vector(x+w.x,y+w.y,z+w.z);
    }
    Vector operator- (Vector w){
        return Vector(x-w.x,y-w.y,z-w.z);
    }
    Vector operator* (Vector w){
        return Vector(y * w.z - z * w.y, z * w.x - x * w.z, x * w.y - y * w.x);
    }
    Vector operator* (phys a){
        return Vector(x*a, y*a, z*a);
    }
    phys operator% (Vector w){
        return x*w.x+y*w.y+z*w.z;
    }
};
struct Quat{
    phys r,i,j,k;
    Quat(phys r=0, phys i=0, phys j=0, phys k=0):r(r),i(i),j(j),k(k){}
    Quat operator+ (Quat w){
        return Quat(r+w.r,i+w.i,j+w.j,k+w.k);
    }
    Quat operator- (Quat w){
        return Quat(r-w.r,i-w.i,j-w.j,k-w.k);
    }
    Quat operator* (Quat w){
        return Quat(r*w.r - i*w.i - j*w.j - k*w.k,
                    r*w.i + i*w.r + j*w.k - k*w.j,
                    r*w.j - i*w.k + j*w.r + k*w.i,
                    r*w.k + i*w.j - j*w.i + k*w.r);
    }
};
struct Mat33{
    phys mat[3][3];
    Mat33(phys a=0.,phys b=0.,phys c=0.,phys d=0.,phys e=0.,phys f=0.,phys g=0.,phys h=0.,phys i=0.){
        mat[0][0]=a;
        mat[0][1]=b;
        mat[0][2]=c;
        mat[1][0]=d;
        mat[1][1]=e;
        mat[1][2]=f;
        mat[2][0]=g;
        mat[2][1]=h;
        mat[2][2]=i;
    }
    Mat33 inv(){
        phys det = 0;
        Mat33 z;
        for(int i=0;i<3;i++) det += (mat[0][i]*
                                     (mat[1][(i+1)%3]*mat[2][(i+2)%3]-mat[1][(i+2)%3]*mat[2][(i+1)%3]));
        if(abs(det)<1e-10){
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++) printf("%.3f ",mat[i][j]);
                puts("");
            }
            puts("Determinant is 0!");
            exit(4444);
        }
        det = 1.L/det;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                z.mat[i][j]=(mat[(j+1)%3][(i+1)%3]*mat[(j+2)%3][(i+2)%3]-mat[(j+1)%3][(i+2)%3]*mat[(j+2)%3][(i+1)%3])*det;
        return z;
    }
    Mat33 operator+ (Mat33 z){
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                z.mat[i][j]+=mat[i][j];
        return z;
    }
    Mat33 operator- (Mat33 z){
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                z.mat[i][j]-=mat[i][j];
        return z;
    }
    Mat33 operator* (Mat33 z){
        Mat33 w(0.,0.,0.,0.,0.,0.,0.,0.,0.);
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                for(int k=0;k<3;k++){
                    w.mat[i][j]+=mat[i][k]*z.mat[k][j];
                }
            }
        }
        return w;
    }
    Vector operator* (Vector z){
        Vector w;
        return Vector(mat[0][0]*z.x+mat[0][1]*z.y+mat[0][2]*z.z,
                      mat[1][0]*z.x+mat[1][1]*z.y+mat[1][2]*z.z,
                      mat[2][0]*z.x+mat[2][1]*z.y+mat[2][2]*z.z);
    }
    Mat33 transpose(){
        return Mat33(mat[0][0],mat[1][0],mat[2][0],
                     mat[0][1],mat[1][1],mat[2][1],
                     mat[0][2],mat[1][2],mat[2][2]);
    }
};
struct Rigidbody{
    Rigidbody(){}
    phys m,speed;
    Mat33 I, invI;
    Vector r,v,vb,w,euler,F,tau;
    Quat qori;
};
Rigidbody stick;
void initstick(){
    stick.I.mat[0][0]=0.5L*M*R*R;
    stick.I.mat[1][1]=1.L/12.L*M*h*h+1.L/4.L*M*R*R;
    stick.I.mat[2][2]=1.L/12.L*M*h*h+1.L/4.L*M*R*R;
    stick.invI = stick.I.inv();
    puts("I info!");
    for(int i=0 ; i<3 ; i++){
        for(int j=0 ; j<3 ; j++){
            printf("%.3f\t", stick.I.mat[i][j]);
        }
        printf("\n");
    }
    puts("invI info!");
    for(int i=0 ; i<3 ; i++){
        for(int j=0 ; j<3 ; j++){
            printf("%.3f\t", stick.invI.mat[i][j]);
        }
        printf("\n");
    }
    Mat33 Iden = stick.I * stick.I.inv();
    puts("Iden info!");
    for(int i=0 ; i<3 ; i++){
        for(int j=0 ; j<3 ; j++){
            printf("%.3f\t", Iden.mat[i][j]);
        }
        printf("\n");
    }
    stick.tau = Vector();
    stick.r = Vector();
    stick.euler = Vector(0.,0.,0.);
    stick.w = Vector(0.5,0.1,0.3);
}
void flyingstick(){

    phys eux=stick.euler.x;
    phys euy=stick.euler.y;
    phys euz=stick.euler.z;
    //이 밑은 계산 훨씬 줄일수 있음. 계산하면 됨. 추후에 바꿔야할 것.
    Mat33 RollMat(cos(eux),sin(eux),0,
                  -sin(eux),cos(eux),0,
                  0,0,1);
    Mat33 PitchMat(1,0,0,
                   0,cos(euy),sin(euy),
                   0,-sin(euy),cos(euy));
    Mat33 YawMat(cos(euz),sin(euz),0,
                 -sin(euz),cos(euz),0,
                 0,0,1);
    Mat33 Rotatemat=YawMat*PitchMat*RollMat;//문법은 몰라요~~초기화는 알아서 빼서 하세요~~
    Vector wprime = Rotatemat*stick.w;//초기화는 알아서~
    Mat33 WprimeMat(0,-wprime.z, wprime.y,
                    wprime.z, 0, -wprime.x,
                    -wprime.y, wprime.x, 0);//문법은 몰라요~~초기화는 알아서 빼서 하세요~~

    Vector alphaprime = stick.invI * (stick.tau - (WprimeMat*stick.I + stick.I*WprimeMat.transpose())*wprime); //tau를 프라임좌표계에서 봐야~~~
    printf("alphaprime = (%.3f %.3f %.3f)\n",alphaprime.x,alphaprime.y,alphaprime.z);
    Vector alpha = Rotatemat.inv()*alphaprime;
    printf("alpha = (%.3f %.3f %.3f)\n",alpha.x,alpha.y,alpha.z);

    Mat33 I_labframe=Rotatemat.transpose()*stick.I*Rotatemat;
    Vector L = I_labframe*stick.w;
    printf("L = (%.3f %.3f %.3f), |L| = %.6f\n",L.x,L.y,L.z,sqrt(L.x*L.x+L.y*L.y+L.z*L.z));

    Mat33 WtoXYZ(1,sin(eux)*tan(euy),cos(eux)*tan(euy),
                 0,cos(eux),-sin(eux),
                 0,sin(eux)/cos(euy),cos(eux)/cos(euy));

    stick.w = stick.w + alpha*dtime;//스칼라곱 정의해놓을것
    stick.euler = stick.euler + WtoXYZ*stick.w*dtime;//마찬가지

    Vector a;
    stick.v = stick.v + a*dtime;//스칼라곱 정의해놓을것
    stick.r = stick.r + stick.v*dtime;

}
int main(){
    initstick();
    system("pause");
    for(int i=0 ; i<10000 ; i++){
            flyingstick();
            if(i%100) continue;
            printf("time = %.3f\n",dtime*(i+1));
            printf("r = (%.3f, %.3f, %.3f)\n",stick.r.x,stick.r.y,stick.r.z);
            printf("euler = (%.3f,%.3f,%.3f)\n",stick.euler.x,stick.euler.y,stick.euler.z);
            printf("w = (%.3f %.3f %.3f)\n",stick.w.x,stick.w.y,stick.w.z);
    }
}
