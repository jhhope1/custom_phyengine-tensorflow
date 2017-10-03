#include <bits/stdc++.h>
using namespace std;
using phys = double;

//for Cylinder
const double M=10;
const double h=0.5;
const double R=1;
const double g=9.81;


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
    phys operator% (Vector w){
        return x*w.x+y*w.y+z*w.z;
    }
};
struct Quat{
    phys r,i,j,k;
    Quat(phys r, phys i, phys j, phys k):r(r),i(i),j(j),k(k){}
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
    Mat33 inv(){
        phys det = 0;
        Mat33 z;
        for(int i=0;i<3;i++) det += (mat[0][i]*
                                     (mat[1][(i+1)%3]*mat[2][(i+2)%3]-mat[1][(i+2)%3]*mat[2][(i+1)%3]));
        if(abs(det)<1e-10){
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
        Mat33 w;
        for(int k=0;k<3;k++){
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    w.mat[i][j]+=mat[i][k]*mat[k][j];
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
};
struct Rigidbody{
    phys m,speed;
    Mat33 I, invI;
    Vector r,v,vb,w,euler,F,tau;
    Quat qori;
};

void flyingstick(){
    vector <Rigidbody> stick(1);

    stick[0].I.mat[0][0]=double(1/12)*M*h*h+1/4*M*R*R;
    stick[0].I.mat[1][1]=double(1/12)*M*h*h+1/4*M*R*R;
    stick[0].I.mat[2][2]=double(1/2)*M*R*R;
    for(int i=0 ; i<3 ; i++){
        for(int j=0 ; j<3 ; j++){
            printf("%.3f\t", stick[0].I.mat[i][j]);
        }
        printf("\n");
    }
    stick[0].tau=Vector(0,0,0)
    stick[0].r=Vector(0,0,0);
    phys eux=stick[0].euler.x;
    phys euy=stick[0].euler.y;
    phys euz=stick[0].euler.z;
    //이 밑은 계산 훨씬 줄일수 있음. 계산하면 됨. 추후에 바꿔야할 것.
    Mat33 RollMat;
    RollMat.mat[0][0]=1;
    RollMat.mat[1][1]=cos(eux);
    RollMat.mat[1][2]=sin(eux);
    RollMat.mat[2][1]=-sin(eux);
    RollMat.mat[2][2]=cos(eux);

    Mat33 PitchMat;
    PitchMat.mat[0][0]=cos(euy);
    PitchMat.mat[0][2]=-sin(euy);
    PitchMat.mat[1][1]=1;
    PitchMat.mat[2][0]=sin(euy);
    PitchMat.mat[2][2]=cos(euy);

    Mat33 YawMat;
    YawMat.mat[0][0]=cos(euz);
    YawMat.mat[0][1]=sin(euz);
    YawMat.mat[1][0]=-sin(euz);
    YawMat.mat[1][1]=cos(eux);
    YawMat.mat[2][2]=1;

    Mat33 Rotatemat;
    Rotatemat.mat=RollMat.mat*(PitchMat.mat*YawMat.mat);//문법은 몰라요~~초기화는 알아서 빼서 하세요~~

    Vector wprime ;
    wprime= Rotatemat.mat*stick[0].w;//초기화는 알아서~
    Mat33 WprimeMat;

    WprimeMat.mat={0,-wprime.z, wprime.y, wprime.z, 0, -wprime.z, -wprime.y, wprime.x, 0};//문법은 몰라요~~초기화는 알아서 빼서 하세요~~

    Vector alphaprime;
    alphaprime=stick[0].invI*(stick[0].tau-(WprimeMat.mat*stick[0].I+stick[0].I*WprimeMat.inv()*stick[0].w);//또 초기화는 알아서~

    Vector alpha;
    alpha=Rotatemat.inv()*alphaprime.Vector();
    stick[0].w+=alpha.Vector()*dtime;//스칼라곱 정의해놓을것
    stick[0].euler+=stick[0].w*dtime;//마찬가지

    Vector a;
    a=Vector(0,0,0);
    stick[0].v+=a*dtime;//스칼라곱 정의해놓을것
    stick[0].r+=stick[0].v*dtime;

}
int main(){
    flyingstick();
}
