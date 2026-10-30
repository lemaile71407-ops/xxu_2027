// Standalone probe: links upstream GPL sources from an external checkout.
// It is not built into Small Point-LIO and does not change the production map.
#include "small_point_lio/small_ivox.h"
#include "zlio/map.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <malloc.h>
#include <random>
#include <string>
#include <sys/resource.h>
#include <unistd.h>
using V3 = Eigen::Vector3d;
struct Sample { V3 measured, truth, normal; };
using Clock = std::chrono::steady_clock;
static double elapsed(Clock::time_point t) {
    return std::chrono::duration<double>(Clock::now()-t).count();
}
static std::size_t heap_bytes() {
    const auto m = mallinfo2(); return m.uordblks + m.hblkhd;
}
static long rss_bytes() {
    std::ifstream in("/proc/self/statm"); long pages=0, rss=0;
    in >> pages >> rss; return rss * sysconf(_SC_PAGESIZE);
}
static std::vector<Sample> generate(const std::string &scene, std::size_t count, unsigned seed) {
    std::mt19937 random(seed); std::uniform_real_distribution<double> u(0,1);
    std::normal_distribution<double> noise(0,.003);
    std::vector<Sample> result; result.reserve(count);
    for (std::size_t i=0;i<count;++i) {
        Sample s; const double a=u(random),b=u(random); const int face=int(u(random)*3);
        if (scene=="plane") { s.truth=V3(20*a-10,20*b-10,1.137);s.normal=V3::UnitZ(); }
        else if (scene=="dense_patch") { s.truth=V3(.8*a+.013,.8*b+.027,1.137);s.normal=V3::UnitZ(); }
        else if (scene=="corner") {
            if(face==0) {s.truth=V3(2.137,10*a-5,3*b);s.normal=V3::UnitX();}
            else if(face==1) {s.truth=V3(10*a-5,1.237,3*b);s.normal=V3::UnitY();}
            else {s.truth=V3(10*a-5,10*b-5,-.113);s.normal=V3::UnitZ();}
        } else if (scene=="steps") {
            const int step=int(u(random)*24);
            if(face!=0) {s.truth=V3(.3*(step+a)-3.6,8*b-4,.12*step+.017);s.normal=V3::UnitZ();}
            else {s.truth=V3(.3*(step+1)-3.6,8*b-4,.12*(step+a)+.017);s.normal=V3::UnitX();}
        } else { throw std::runtime_error("unknown scene"); }
        s.measured=s.truth+V3(noise(random),noise(random),noise(random));result.push_back(s);
    } return result;
}
static bool small_match(small_point_lio::SmallIVox &map, const V3 &point, V3 &normal, double &residual,
                        std::vector<Eigen::Vector3f> &near) {
    map.get_closest_point(point.cast<float>(),near,5);
    if(near.size()!=5) return false;
    Eigen::Vector3f center=Eigen::Vector3f::Zero();for(const auto &p:near)center+=p;center/=5;
    Eigen::Matrix3f scatter=Eigen::Matrix3f::Zero();
    for(const auto &p:near){const Eigen::Vector3f d=p-center;scatter+=d*d.transpose();}
    scatter/=4;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(scatter);
    const Eigen::Vector3f n=solver.eigenvectors().col(0);
    for(const auto &p:near) if(std::abs(n.dot(p-center))>.1f)return false;
    residual=n.cast<double>().dot(point-center.cast<double>());
    if(point.norm()<=81*residual*residual)return false;
    normal=n.cast<double>();return true;
}
int main(int argc,char **argv) {
    if(argc!=7){std::cerr<<"method scene points seed voxel passes\n";return 2;}
    const std::string method=argv[1],scene=argv[2];const std::size_t count=std::stoull(argv[3]);
    const unsigned seed=std::stoul(argv[4]);const double voxel=std::stod(argv[5]);const int passes=std::stoi(argv[6]);
    if(method!="ivox"&&method!="moments")return 2;
    std::vector<std::vector<Sample>> training;
    for(int pass=0;pass<passes;++pass)training.push_back(generate(scene,count,seed+1000*pass));
    auto queries=generate(scene,4096,seed+100000);
    // Allocate identical scratch storage before measuring map memory. Bootstrap
    // points carry independent covariance only; no pose uncertainty is injected.
    std::vector<zlio::MapPoint> batch;batch.reserve(1024);
    std::vector<Eigen::Vector3f> nearest;nearest.reserve(7);
    malloc_trim(0);const auto heap_before=heap_bytes();const auto rss_before=rss_bytes();
    std::unique_ptr<small_point_lio::SmallIVox> ivox;
    std::unique_ptr<zlio::VoxelMap> moments;
    if(method=="ivox")ivox=std::make_unique<small_point_lio::SmallIVox>(voxel,1000000);
    else {zlio::MapConfig config;config.voxel_size=voxel;moments=std::make_unique<zlio::VoxelMap>(config);}
    double first_update=0,later_update=0;std::size_t first_heap=0;
    for(int pass=0;pass<passes;++pass){
        auto start=Clock::now();
        const auto &points=training[pass];
        for(std::size_t i=0;i<points.size();){
            const auto end=std::min(i+1024,points.size());
            if(ivox){for(;i<end;++i)ivox->add_point(points[i].measured.cast<float>());}
            else {batch.clear();for(;i<end;++i)batch.push_back({points[i].measured,zlio::Mat3::Identity()*.000009,{}});moments->insert(batch);}
        }
        const double seconds=elapsed(start);if(!pass){first_update=seconds;first_heap=heap_bytes();}else later_update+=seconds;
    }
    malloc_trim(0);const auto heap_after=heap_bytes();const auto rss_after=rss_bytes();
    std::size_t matched=0,good=0;double error2=0,angle_sum=0;double checksum=0;
    auto query_start=Clock::now();zlio::VoxelMap::QueryCache cache;
    for(const auto &q:queries){
        V3 normal;double residual=0;bool ok=false;
        if(ivox)ok=small_match(*ivox,q.measured,normal,residual,nearest);
        else if(auto m=moments->match(q.measured,zlio::Mat3::Identity()*.000009,&cache)){normal=m->normal;residual=m->residual;ok=true;}
        if(!ok)continue;++matched;
        double dot=normal.dot(q.normal);if(dot<0){dot=-dot;residual=-residual;}
        const double angle=std::acos(std::clamp(dot,0.,1.))*180/std::acos(-1.);
        const double true_residual=q.normal.dot(q.measured-q.truth);
        const double e=residual-true_residual;error2+=e*e;angle_sum+=angle;checksum+=residual;
        if(angle<=5&&std::abs(e)<=.02)++good;
    }
    const double query_seconds=elapsed(query_start);rusage usage{};getrusage(RUSAGE_SELF,&usage);
    std::cout<<std::setprecision(12)<<method<<','<<scene<<','<<count<<','<<seed<<','<<voxel<<','<<passes<<','
      <<first_update<<','<<later_update<<','<<static_cast<long long>(first_heap)-static_cast<long long>(heap_before)<<','
      <<static_cast<long long>(heap_after)-static_cast<long long>(heap_before)<<','<<rss_after-rss_before<<','<<usage.ru_maxrss*1024L<<','
      <<queries.size()<<','<<matched<<','<<good<<','<<query_seconds<<','
      <<(matched?std::sqrt(error2/matched):NAN)<<','<<(matched?angle_sum/matched:NAN)<<','<<checksum<<'\n';
}
