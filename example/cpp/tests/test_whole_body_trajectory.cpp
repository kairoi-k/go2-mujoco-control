#include "stage_c/whole_body_trajectory.h"
#include <fstream>
#include <iostream>
#include <sstream>
using namespace go2_terrain::stage_c::whole_body;
namespace {
int checks=0;
void Check(bool ok,const char* label){++checks;if(!ok)throw std::runtime_error(label);}
ParseResult Read(const std::string& s){std::istringstream in(s);return Parse(in);}
std::string Replace(std::string s,const std::string& from,const std::string& to){auto pos=s.find(from);Check(pos!=std::string::npos,"fixture replacement target");s.replace(pos,from.size(),to);return s;}
}
int main(int argc,char** argv){try{
    if(argc!=5)throw std::runtime_error("usage: test packet model actual_packet_sha256 actual_manifest_sha256 (compute hashes outside parser)");
    std::ifstream input;std::ostringstream buffer;
    if(std::string(argv[1])=="-")buffer<<std::cin.rdbuf();
    else {input.open(argv[1]);buffer<<input.rdbuf();}
    const auto bytes=buffer.str();
    const auto p=Read(bytes);Check(bool(p.parsed),p.error.c_str());const auto& data=*p.parsed;
    char error[1024]{};mjModel* raw=mj_loadXML(argv[2],nullptr,error,sizeof(error));
    std::unique_ptr<mjModel,void(*)(mjModel*)> model(raw,mj_deleteModel);Check(raw!=nullptr,error);
    // Test fixture expectations are deliberately assembled here; production
    // loaders must independently verify closure, model hash, and file digests.
    ExpectedContext context{data.identity,data.model_files,data.sources};
    ByteBinding binding{argv[3],argv[3],argv[4],argv[4]};
    const auto checked=Verify(p,binding,context,raw);Check(bool(checked.verified),checked.error.c_str());
    Check(!checked.verified->can_actuate,"verified research cannot actuate");
    const auto& t=*checked.verified;
    Check(bool(SampleAt(t,context.identity,data.start_ns).knot),"start included");
    Check(bool(SampleAt(t,context.identity,data.end_ns-data.dt_ns).knot),"last grid included");
    Check(!SampleAt(t,context.identity,data.end_ns).knot,"end excluded");
    Check(!SampleAt(t,context.identity,data.start_ns-1).knot,"future rejected");
    Check(!SampleAt(t,context.identity,data.start_ns+1).knot,"non-grid rejected");
    Check(!SampleAt(t,context.identity,data.end_ns+data.dt_ns).knot,"no looping");
    auto identity=context.identity;++identity.map_epoch;Check(!SampleAt(t,identity,data.start_ns).knot,"map mismatch rejected");
    identity=context.identity;++identity.schedule_epoch;Check(!SampleAt(t,identity,data.start_ns).knot,"schedule mismatch rejected");
    identity=context.identity;++identity.source_ns;Check(!SampleAt(t,identity,data.start_ns).knot,"source mismatch rejected");
    ActualState actual{data.start_ns,data.knots[0].q,data.knots[0].v};
    auto torque=FeedbackAt(t,context.identity,raw,data.start_ns,actual,{1,1});
    Check(torque.valid&&!torque.can_actuate,"nominal shadow candidate only");
    for(int i=0;i<12;++i)Check(std::abs(torque.candidate_tau[i]-data.knots[0].tau[i])<1e-12,"zero-error nominal torque");
    for(int i=3;i<7;++i)actual.q[i]=-actual.q[i];
    const auto sign=FeedbackAt(t,context.identity,raw,data.start_ns,actual,{1,1});Check(sign.valid,"quaternion sign equivalence");
    for(int i=0;i<12;++i)Check(std::abs(sign.candidate_tau[i]-torque.candidate_tau[i])<1e-10,"quaternion sign torque invariance");
    actual.q=data.knots[0].q;actual.q[0]+=1e-4;actual.v[0]+=2e-4;
    const auto perturb=FeedbackAt(t,context.identity,raw,data.start_ns,actual,{1,1});Check(perturb.valid,"small tracking perturbation");
    for(int i=0;i<12;++i){double expected=data.knots[0].tau[i]-data.knots[0].gain[i*36]*1e-4-data.knots[0].gain[i*36+18]*2e-4;Check(std::abs(perturb.candidate_tau[i]-expected)<1e-9,"gain row/tangent ordering");}
    Check(!FeedbackAt(t,context.identity,raw,data.start_ns,actual,{1e-5,1}).valid,"tracking limit rejection");
    ++actual.time_ns;Check(!FeedbackAt(t,context.identity,raw,data.start_ns,actual,{1,1}).valid,"actual time mismatch");--actual.time_ns;
    actual.q[3]=2;Check(!FeedbackAt(t,context.identity,raw,data.start_ns,actual,{1,1}).valid,"actual quaternion rejection");
    actual.q=data.knots[0].q;actual.v[0]=std::numeric_limits<double>::infinity();Check(!FeedbackAt(t,context.identity,raw,data.start_ns,actual,{1,1}).valid,"actual finite check");
    auto bad=binding;bad.actual_packet_sha256=std::string(64,'a');Check(!Verify(p,bad,context,raw).verified,"packet digest mismatch");
    bad=binding;bad.actual_manifest_sha256=std::string(64,'a');Check(!Verify(p,bad,context,raw).verified,"manifest digest mismatch");
    auto wrong=context;wrong.identity.model_sha256=std::string(64,'a');Check(!Verify(p,binding,wrong,raw).verified,"model identity mismatch");
    wrong=context;wrong.sources.at("nominal").revision=std::string(40,'a');Check(!Verify(p,binding,wrong,raw).verified,"source revision mismatch");
    wrong=context;wrong.model_files.begin()->second=std::string(64,'a');Check(!Verify(p,binding,wrong,raw).verified,"model closure mismatch");
    const int original=raw->jnt_dofadr[1];raw->jnt_dofadr[1]=17;Check(!Verify(p,binding,context,raw).verified,"model coordinate mismatch");raw->jnt_dofadr[1]=original;
    const double original_dt=raw->opt.timestep;raw->opt.timestep=.01;
    Check(!Verify(p,binding,context,raw).verified,"model timestep mismatch");raw->opt.timestep=original_dt;
    const double original_gear=raw->actuator_gear[0];raw->actuator_gear[0]=2;
    Check(!Verify(p,binding,context,raw).verified,"model gear mismatch");raw->actuator_gear[0]=original_gear;
    const double original_gain=raw->actuator_gainprm[0];raw->actuator_gainprm[0]=2;
    Check(!Verify(p,binding,context,raw).verified,"model gain mismatch");raw->actuator_gainprm[0]=original_gain;
    Check(!Read(Replace(bytes,"authority 1 0 0","authority 1 1 0")).parsed,"production authority rejected");
    Check(!Read(Replace(bytes,"authority 1 0 0","authority 1 0 1")).parsed,"observed authority rejected");
    Check(!Read(Replace(bytes," 19 18 12 36"," 18 18 12 36")).parsed,"dimension mismatch");
    Check(!Read(Replace(bytes,"dimensions 70","dimensions -1")).parsed,"negative count rejected");
    Check(!Read(Replace(bytes,"dimensions 70","dimensions 99999999")).parsed,"oversized count rejected");
    Check(!Read(bytes.substr(0,bytes.size()/2)).parsed,"truncation rejected");
    Check(!Read(bytes+"extra").parsed,"trailing data rejected");
    Check(!Read(Replace(bytes,"limit torque_limit_nm 35","limit torque_limit_nm nan")).parsed,"nonfinite metadata rejected");
    const auto first=bytes.find("sample 0 ");Check(first!=std::string::npos,"sample exists");
    const auto qstart=bytes.find(' ',bytes.find(' ',first+7)+1)+1;
    const auto qend=bytes.find(' ',qstart);
    auto invalid=bytes;invalid.replace(qstart,qend-qstart,"nan");Check(!Read(invalid).parsed,"nonfinite sample rejected");
    // Corrupt the fourth q coordinate (quaternion w), not the time/index.
    std::istringstream sample_line(bytes.substr(first));
    std::string tag;std::size_t index;std::int64_t stamp;sample_line>>tag>>index>>stamp;
    std::array<std::string,4> qt;for(auto& value:qt)sample_line>>value;
    std::size_t quat=first;for(int i=0;i<6;++i)quat=bytes.find(' ',quat)+1;
    auto bad_quat=bytes;bad_quat.replace(quat,bytes.find(' ',quat)-quat,"2");
    Check(!Read(bad_quat).parsed,"sample quaternion rejected");
    Check(!Read(Replace(bytes,"coverage 21020000000 21160000000 2000000","coverage 21020000000 21160000001 2000000")).parsed,"non-grid coverage rejected");
    Check(!Read(Replace(bytes,"sample 0 21020000000","sample 0 21020000001")).parsed,"sample timestamp rejected");
    Check(!Read(Replace(bytes,"command_authority 0 0","command_authority 1 0")).parsed,"command authority rejected");
    Check(!Read(Replace(bytes,"sources 2","sources 1")).parsed,"incomplete source provenance rejected");
    Check(!data.certificates.at("nominal").diagnostic_success&&!data.certificates.at("nominal").failflags.empty(),"original nominal failure retained");
    std::cout<<"whole_body_trajectory checks="<<checks<<" PASS; shadow only, no physics stepping\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<"\n";return 1;}}
