// Initialized stereo-only native replay. No tracking, map, vocabulary or optimizer.
#include <Frame.h>
#include <ORBextractor.h>
#include <ORBmatcher.h>
#include <Settings.h>
#include <System.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>

void descriptor(std::ostream& o,const cv::Mat& d,int row) {
    o<<"[";for(int c=0;c<32;++c){if(c)o<<",";o<<int(d.at<unsigned char>(row,c));}o<<"]";
}
void patch(std::ostream& o,const cv::Mat& m) {
    o<<"[";bool first=true;for(int y=0;y<m.rows;++y)for(int x=0;x<m.cols;++x){if(!first)o<<",";first=false;o<<int(m.at<unsigned char>(y,x));}o<<"]";
}
int main(int argc,char** argv) {
 try {
  if(argc!=4)throw std::runtime_error("usage settings selected.tsv output.jsonl");
  int fd=open(argv[3],O_WRONLY|O_CREAT|O_EXCL,0600);if(fd<0)throw std::runtime_error("output exists");close(fd);std::ofstream out(argv[3]);out<<std::setprecision(17);
  cv::setNumThreads(1);cv::setRNGSeed(0);ORB_SLAM3::Settings cfg(argv[1],ORB_SLAM3::System::STEREO);
  ORB_SLAM3::ORBextractor extL(cfg.nFeatures(),cfg.scaleFactor(),cfg.nLevels(),cfg.initThFAST(),cfg.minThFAST());
  ORB_SLAM3::ORBextractor extR(cfg.nFeatures(),cfg.scaleFactor(),cfg.nLevels(),cfg.initThFAST(),cfg.minThFAST());
  std::ifstream file(argv[2]);std::string line;
  while(std::getline(file,line)) {
   std::vector<std::string> fields;std::istringstream ls(line);std::string v;while(std::getline(ls,v,'\t'))fields.push_back(v);
   if(fields.size()!=5)throw std::runtime_error("bad selected input row");
   int index=std::stoi(fields[0]);cv::Mat left=cv::imread(fields[1],-1),right=cv::imread(fields[2],-1);
   if(left.empty()||right.empty()||left.depth()!=CV_8U||right.depth()!=CV_8U)throw std::runtime_error("canonical input");
   if(left.channels()==3)cv::cvtColor(left,left,cv::COLOR_BGR2GRAY);if(right.channels()==3)cv::cvtColor(right,right,cv::COLOR_BGR2GRAY);
   ORB_SLAM3::Frame f;f.mpORBextractorLeft=&extL;f.mpORBextractorRight=&extR;f.mbf=cfg.bf();f.mb=f.mbf/cfg.camera1()->getParameter(0);
   f.mvScaleFactors=extL.GetScaleFactors();f.mvInvScaleFactors=extL.GetInverseScaleFactors();f.ExtractORB(0,left,0,0);f.ExtractORB(1,right,0,0);f.N=f.mvKeys.size();
   if(f.N==0)throw std::runtime_error("empty replay image");f.ComputeStereoMatches();
   out<<"{\"index\":"<<index<<",\"bf\":"<<f.mbf<<",\"baseline_m\":"<<f.mb<<",\"max_disparity\":"<<f.mbf/f.mb<<",\"scale_factors\":[";
   for(size_t j=0;j<f.mvScaleFactors.size();++j){if(j)out<<",";out<<f.mvScaleFactors[j];}out<<"],\"inverse_scales\":[";
   for(size_t j=0;j<f.mvInvScaleFactors.size();++j){if(j)out<<",";out<<f.mvInvScaleFactors[j];}out<<"],\"features\":[";
   for(int j=0;j<f.N;++j){if(j)out<<",";auto k=f.mvKeys[j];out<<"["<<k.pt.x<<","<<k.pt.y<<","<<k.octave<<","<<f.mvuRight[j]<<","<<f.mvDepth[j]<<"]";}
   out<<"],\"right_features\":[";
   for(size_t j=0;j<f.mvKeysRight.size();++j){if(j)out<<",";auto k=f.mvKeysRight[j];out<<"{\"key\":["<<k.pt.x<<","<<k.pt.y<<","<<k.octave<<"],\"descriptor\":";descriptor(out,f.mDescriptorsRight,j);out<<"}";}
   out<<"],\"points\":[";std::vector<int> ids;std::istringstream il(fields[4]);while(std::getline(il,v,','))if(!v.empty())ids.push_back(std::stoi(v));bool first=true;
   for(int id:ids) {
    if(id<0||id>=f.N)throw std::runtime_error("retained ID outside replay");if(!first)out<<",";first=false;auto k=f.mvKeys[id];
    int row=int(k.pt.y),level=k.octave;float minU=k.pt.x-f.mbf/f.mb;int best=100,bestId=-1;
    out<<"{\"feature_index\":"<<id<<",\"descriptor\":";descriptor(out,f.mDescriptors,id);out<<",\"candidates\":[";bool fc=true;
    for(size_t j=0;j<f.mvKeysRight.size();++j) {
     auto q=f.mvKeysRight[j];float radius=2.f*f.mvScaleFactors[q.octave];
     if(row<int(std::floor(q.pt.y-radius))||row>int(std::ceil(q.pt.y+radius))||q.octave<level-1||q.octave>level+1||q.pt.x<minU||q.pt.x>k.pt.x)continue;
     int dist=ORB_SLAM3::ORBmatcher::DescriptorDistance(f.mDescriptors.row(id),f.mDescriptorsRight.row(j));if(!fc)out<<",";fc=false;out<<"["<<j<<","<<dist<<"]";if(dist<best){best=dist;bestId=j;}
    }
    out<<"],\"selected_right_id\":"<<bestId<<",\"best_hamming\":"<<best;
    if(bestId<0||best>=75) {out<<",\"patch_status\":\"descriptor_rejected\",\"costs\":null}";continue;}
    float inv=f.mvInvScaleFactors[level];int ul=int(std::round(k.pt.x*inv)),vl=int(std::round(k.pt.y*inv)),ur=int(std::round(f.mvKeysRight[bestId].pt.x*inv));
    auto &lp=extL.mvImagePyramid[level];auto &rp=extR.mvImagePyramid[level];
    if(ul-5<0||ul+6>lp.cols||vl-5<0||vl+6>lp.rows||ur<0||ur+11>=rp.cols||ur-10<0){out<<",\"patch_status\":\"patch_outside\",\"costs\":null}";continue;}
    auto IL=lp.rowRange(vl-5,vl+6).colRange(ul-5,ul+6);std::vector<float> costs;
    out<<",\"patch_status\":\"ok\",\"scaled_left\":["<<ul<<","<<vl<<"],\"scaled_right_center\":"<<ur<<",\"left_patch\":";patch(out,IL);out<<",\"right_patches\":[";
    for(int shift=-5;shift<=5;++shift){if(shift!=-5)out<<",";auto IR=rp.rowRange(vl-5,vl+6).colRange(ur+shift-5,ur+shift+6);costs.push_back(cv::norm(IL,IR,cv::NORM_L1));patch(out,IR);}
    out<<"],\"costs\":[";for(size_t j=0;j<costs.size();++j){if(j)out<<",";out<<costs[j];}out<<"]}";
   }
   out<<"]}\n";out.flush();
  }
  return 0;
 } catch(const std::exception& e){std::cerr<<"replay failure: "<<e.what()<<"\n";return 1;}
}
