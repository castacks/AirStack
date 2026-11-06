"""Minimal stereo-only initialization order repair, or unchanged rebuild control."""
from pathlib import Path
import hashlib,json,difflib,sys
p=Path('/opt/ORB_SLAM3/src/Frame.cc');original=p.read_text()
assert hashlib.sha256(original.encode()).hexdigest()=='67e0c446d9da80db11aceee4ff2664487cfe1f2502bc50d843ff7471917eb42e'
apply=sys.argv[1]=='1';a=original.index('Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight,');b=original.index('Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth,',a);block=original[a:b]
assert block.count('    mb = mbf/fx;\n')==1 and block.count('    ComputeStereoMatches();\n')==1
fixed=block.replace('    ComputeStereoMatches();\n','    mb = mbf / mK.at<float>(0,0);\n    ComputeStereoMatches();\n').replace('    mb = mbf/fx;\n','')
result=original[:a]+fixed+original[b:] if apply else original
out=Path('/opt/ore-ms-baseline-init');out.mkdir(exist_ok=True);(out/'Frame.cc.original').write_text(original);(out/'Frame.cc.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),result.splitlines(True),fromfile='a/src/Frame.cc',tofile='b/src/Frame.cc')))
p.write_text(result);(out/'patch.json').write_text(json.dumps(dict(applied=apply,original_sha256=hashlib.sha256(original.encode()).hexdigest(),result_sha256=hashlib.sha256(result.encode()).hexdigest(),scope='stereo constructor assignment moved before matcher using current K'),indent=2)+'\n')
