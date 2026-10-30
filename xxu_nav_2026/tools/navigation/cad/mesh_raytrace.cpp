// Offline, double-sided triangle ray tracing for CAD visibility analysis.
// Build: g++ -O3 -std=c++17 -shared -fPIC -fopenmp mesh_raytrace.cpp -o mesh_raytrace.so
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <vector>

struct Triangle { float v[3][3]; };
struct Node { float lo[3], hi[3]; int start, count, left, right; };
static std::vector<Triangle> triangles;
static std::vector<int> order;
static std::vector<Node> nodes;

static int build(int start, int end) {
    Node n{};
    n.start = start; n.count = end-start; n.left = n.right = -1;
    float clo[3], chi[3];
    for (int k=0;k<3;k++) { n.lo[k]=clo[k]=INFINITY; n.hi[k]=chi[k]=-INFINITY; }
    for (int i=start;i<end;i++) {
        const auto &t=triangles[order[i]];
        for (int k=0;k<3;k++) {
            float c=0;
            for (int j=0;j<3;j++) { n.lo[k]=std::min(n.lo[k],t.v[j][k]); n.hi[k]=std::max(n.hi[k],t.v[j][k]); c+=t.v[j][k]/3; }
            clo[k]=std::min(clo[k],c); chi[k]=std::max(chi[k],c);
        }
    }
    int id=nodes.size(); nodes.push_back(n);
    int axis=0; for (int k=1;k<3;k++) if (chi[k]-clo[k]>chi[axis]-clo[axis]) axis=k;
    if (end-start>8 && chi[axis]-clo[axis]>1e-9) {
        int mid=(start+end)/2;
        std::nth_element(order.begin()+start,order.begin()+mid,order.begin()+end,[axis](int a,int b) {
            const auto &x=triangles[a]; const auto &y=triangles[b];
            return x.v[0][axis]+x.v[1][axis]+x.v[2][axis] < y.v[0][axis]+y.v[1][axis]+y.v[2][axis];
        });
        int l=build(start,mid),r=build(mid,end);
        nodes[id].left=l; nodes[id].right=r; nodes[id].count=0;
    }
    return id;
}

static bool box(const Node &n,const double *o,const double *d,double best) {
    double lo=0,hi=best;
    for(int k=0;k<3;k++) {
        if(std::abs(d[k])<1e-15) { if(o[k]<n.lo[k]-1e-7 || o[k]>n.hi[k]+1e-7) return false; continue; }
        double a=(n.lo[k]-o[k]-1e-7)/d[k],b=(n.hi[k]-o[k]+1e-7)/d[k];
        if(a>b) std::swap(a,b);
        lo=std::max(lo,a); hi=std::min(hi,b); if(lo>hi) return false;
    }
    return true;
}
static double hit(const Triangle &t,const double *o,const double *d) {
    double e[3],f[3],p[3],q[3],s[3];
    for(int k=0;k<3;k++) {e[k]=t.v[1][k]-t.v[0][k];f[k]=t.v[2][k]-t.v[0][k];s[k]=o[k]-t.v[0][k];}
    p[0]=d[1]*f[2]-d[2]*f[1]; p[1]=d[2]*f[0]-d[0]*f[2]; p[2]=d[0]*f[1]-d[1]*f[0];
    double det=e[0]*p[0]+e[1]*p[1]+e[2]*p[2]; if(std::abs(det)<1e-14) return INFINITY;
    double u=(s[0]*p[0]+s[1]*p[1]+s[2]*p[2])/det; if(u < -1e-9 || u>1+1e-9) return INFINITY;
    q[0]=s[1]*e[2]-s[2]*e[1]; q[1]=s[2]*e[0]-s[0]*e[2]; q[2]=s[0]*e[1]-s[1]*e[0];
    double v=(d[0]*q[0]+d[1]*q[1]+d[2]*q[2])/det; if(v < -1e-9 || u+v>1+1e-9) return INFINITY;
    double x=(f[0]*q[0]+f[1]*q[1]+f[2]*q[2])/det;
    return x>1e-5?x:INFINITY;
}
extern "C" void initialize(const float *v,int count) {
    triangles.resize(count); for(int i=0;i<count;i++) for(int j=0;j<3;j++) for(int k=0;k<3;k++) triangles[i].v[j][k]=v[9*i+3*j+k];
    order.resize(count); std::iota(order.begin(),order.end(),0); nodes.clear(); nodes.reserve(count/2); build(0,count);
}
extern "C" void trace(const double *o,const double *dirs,int count,double *out) {
    #pragma omp parallel for num_threads(2)
    for(int i=0;i<count;i++) {
        const double *d=dirs+3*i; double best=INFINITY;
        std::vector<int> stack{0};
        while(!stack.empty()) {
            int id=stack.back(); stack.pop_back(); const auto &n=nodes[id];
            if(!box(n,o,d,best)) continue;
            if(n.count) {for(int j=n.start;j<n.start+n.count;j++) best=std::min(best,hit(triangles[order[j]],o,d));}
            else {stack.push_back(n.left);stack.push_back(n.right);}
        }
        out[i]=best;
    }
}
