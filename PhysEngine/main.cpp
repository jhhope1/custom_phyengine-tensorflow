#include <bits/stdc++.h>
using namespace std;
using phys = double;
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
struct Rigidbody2d{
    phys m,speed;
    Mat33 I,invI;
    Vector r,v,vb,w,euler,F,tau;
    Quat qori;
};
int main(){

}
