#include <Frame.h>
#include <ORBextractor.h>
#include <CameraModels/Pinhole.h>
#include <opencv2/core.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <type_traits>
#include <new>
#include <cstring>
int main(int argc,char** argv) {
 if(argc!=3)return 2;int pattern=std::stoi(argv[1]);cv::setNumThreads(1);cv::setRNGSeed(0);
 cv::Mat left(240,320,CV_8U),right=cv::Mat::zeros(240,320,CV_8U);cv::RNG rng(1234);rng.fill(left,cv::RNG::UNIFORM,0,256);left.colRange(4,320).copyTo(right.colRange(0,316));
 ORB_SLAM3::ORBextractor a(2000,1.2,8,20,7),b(2000,1.2,8,20,7);ORB_SLAM3::Pinhole camera(std::vector<float>{128,128,160,120});cv::Mat K=(cv::Mat_<float>(3,3)<<128,0,160,0,128,120,0,0,1),dist=cv::Mat::zeros(4,1,CV_32F);
 typename std::aligned_storage<sizeof(ORB_SLAM3::Frame),alignof(ORB_SLAM3::Frame)>::type storage;std::memset(&storage,pattern,sizeof(storage));
 auto f=new(&storage) ORB_SLAM3::Frame(left,right,0,&a,&b,nullptr,K,dist,38.4f,10.5f,&camera);
 std::ofstream out(argv[2]);out<<std::setprecision(17)<<"{\"pattern\":"<<pattern<<",\"N\":"<<f->N<<",\"bf\":"<<f->mbf<<",\"baseline_m\":"<<f->mb<<",\"features\":[";
 for(int i=0;i<f->N;++i){if(i)out<<",";auto k=f->mvKeys[i];out<<"["<<k.pt.x<<","<<k.pt.y<<","<<k.octave<<","<<f->mvuRight[i]<<","<<f->mvDepth[i]<<"]";}out<<"]}\n";f->~Frame();return 0;
}
