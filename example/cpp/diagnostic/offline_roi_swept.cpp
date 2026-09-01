#include "offline_roi_swept.h"
#include <cmath>
namespace go2_diagnostic { RoiInput MakeOfflineRoi(std::string i,std::string f,std::string g,double t,std::uint64_t q){bool v=!i.empty()&&!f.empty()&&!g.empty()&&std::isfinite(t);return {std::move(i),"offline",std::move(f),std::move(g),t,q,v,v?PayloadRepresentation::kCompleteMessageValue:PayloadRepresentation::kAbsent};} SweptInput MakeOfflineSwept(std::string i,std::string f,std::string g,double t,std::uint64_t q){return {MakeOfflineRoi(std::move(i),std::move(f),std::move(g),t,q)};} }
