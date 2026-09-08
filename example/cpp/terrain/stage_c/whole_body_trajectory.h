#pragma once
// Research packet only. No owner, clock rebasing, contact inference, or motor writes.
// Hashes below are checked against caller-computed digests, not computed here.
#include <mujoco/mujoco.h>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <istream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
namespace go2_terrain { namespace stage_c { namespace whole_body {
struct Identity {
    std::string model_sha256, observation_sha256;
    std::uint64_t source_tick=0, map_epoch=0, schedule_epoch=0;
    std::int64_t source_ns=0;
    bool operator==(const Identity& b) const {
        return model_sha256==b.model_sha256 && observation_sha256==b.observation_sha256 &&
            source_tick==b.source_tick && source_ns==b.source_ns &&
            map_epoch==b.map_epoch && schedule_epoch==b.schedule_epoch;
    }
};
struct Source { std::string revision, result_sha256; };
struct Actuator { std::string name, joint; int qpos=0, qvel=0; };
struct Certificate {
    std::string sha256, result_sha256;
    bool diagnostic_success=false;
    std::vector<std::string> failflags;
};
struct Derivative { double epsilon=0, relative_error=0; };
struct Knot {
    std::array<double,19> q{};
    std::array<double,18> v{};
    std::array<double,12> tau{};
    std::array<double,432> gain{};
};
// Published only as shared_ptr<const Payload>; parser never exposes a mutable alias.
struct Payload {
    Identity identity;
    std::string manifest_sha256;
    std::int64_t start_ns=0, end_ns=0, dt_ns=0;
    bool research_only=true, production_ready=false, observed_terrain=false;
    std::map<std::string,Source> sources;
    std::map<std::string,std::string> model_files;
    std::map<std::string,double> limits;
    std::array<Actuator,12> ordering{};
    std::map<std::string,Certificate> certificates;
    std::vector<Derivative> derivatives;
    double command_vx=0, period_s=0, phase=0, duty=0;
    std::vector<Knot> knots;
    std::array<double,19> terminal_q{};
    std::array<double,18> terminal_v{};
};
namespace detail {
inline bool Hex(const std::string& s,std::size_t n) {
    if(s.size()!=n) return false;
    for(char c:s) if(!((c>='0'&&c<='9')||(c>='a'&&c<='f'))) return false;
    return true;
}
inline void Require(bool valid,const char* error) { if(!valid) throw std::runtime_error(error); }
class Reader {
    std::istream& in_;
public:
    explicit Reader(std::istream& in):in_(in){}
    std::string Word() { std::string s; Require(bool(in_>>s),"truncated packet"); Require(s.size()<=4096,"token too long"); return s; }
    void Tag(const char* s) { Require(Word()==s,"unexpected record"); }
    std::string Hash(std::size_t n=64) { auto s=Word(); Require(Hex(s,n),"invalid digest"); return s; }
    template<class T> T Integer() {
        const auto s=Word(); T v{}; const auto r=std::from_chars(s.data(),s.data()+s.size(),v);
        Require(r.ec==std::errc{} && r.ptr==s.data()+s.size(),"invalid integer"); return v;
    }
    double Number() { double v; Require(bool(in_>>v)&&std::isfinite(v),"invalid numeric value"); return v; }
    std::size_t Count(std::size_t max) { auto n=Integer<std::uint64_t>(); Require(n<=max,"record count too large"); return static_cast<std::size_t>(n); }
    std::string Quoted() { in_>>std::ws; Require(in_.peek()=='"',"quoted string required"); std::string s; Require(bool(in_>>std::quoted(s)),"invalid quoted string"); Require(!s.empty()&&s.size()<=4096,"invalid string"); return s; }
    template<std::size_t N> void Numbers(std::array<double,N>& a) { for(auto& v:a)v=Number(); }
    void End() { Tag("end"); in_>>std::ws; Require(in_.peek()==std::char_traits<char>::eof(),"trailing packet data"); }
};
template<std::size_t N> inline bool Finite(const std::array<double,N>& a) { for(double v:a)if(!std::isfinite(v))return false; return true; }
inline bool Unit(const std::array<double,19>& q) { double n=0;for(int i=3;i<7;++i)n+=q[i]*q[i];return std::isfinite(n)&&std::abs(n-1.0)<=2e-9; }
} // detail
class ParseResult;
ParseResult Parse(std::istream&);
class ParseResult final {
    ParseResult(std::shared_ptr<const Payload> p,std::string e):parsed(std::move(p)),error(std::move(e)){}
    friend ParseResult Parse(std::istream&);
public:
    const std::shared_ptr<const Payload> parsed;
    const std::string error;
};
inline ParseResult Parse(std::istream& input) {
    using namespace detail;
    try {
        Reader r(input); auto p=std::make_shared<Payload>();
        r.Tag("whole-body-trajectory-v1-research"); r.Tag("manifest_sha256"); p->manifest_sha256=r.Hash();
        r.Tag("dimensions"); const auto n=r.Count(4096); Require(n>0,"empty trajectory");
        Require(r.Integer<int>()==19&&r.Integer<int>()==18&&r.Integer<int>()==12&&r.Integer<int>()==36,"unsupported dimensions");
        r.Tag("coverage"); p->start_ns=r.Integer<std::int64_t>();p->end_ns=r.Integer<std::int64_t>();p->dt_ns=r.Integer<std::int64_t>();
        Require(p->start_ns>=0&&p->end_ns>p->start_ns&&p->dt_ns>0,"invalid coverage");
        const auto span=p->end_ns-p->start_ns;
        Require(span%p->dt_ns==0&&static_cast<std::uint64_t>(span/p->dt_ns)==n,"coverage count mismatch");
        r.Tag("authority");Require(r.Integer<int>()==1&&r.Integer<int>()==0&&r.Integer<int>()==0,"research-only authority required");
        r.Tag("identity");p->identity.model_sha256=r.Hash();p->identity.observation_sha256=r.Hash();
        p->identity.source_tick=r.Integer<std::uint64_t>();p->identity.source_ns=r.Integer<std::int64_t>();
        p->identity.map_epoch=r.Integer<std::uint64_t>();p->identity.schedule_epoch=r.Integer<std::uint64_t>();
        Require(p->identity.source_ns==p->start_ns,"source time mismatch");
        // V1 exported research data explicitly has no verified runtime epochs/tick.
        Require(p->identity.source_tick==0&&p->identity.map_epoch==0&&p->identity.schedule_epoch==0,"research identity must mark unavailable provenance");
        r.Tag("sources");Require(r.Count(2)==2,"source count mismatch");
        for(int i=0;i<2;++i){r.Tag("source");auto name=r.Word();Source s;s.revision=r.Hash(40);s.result_sha256=r.Hash();Require(p->sources.emplace(name,s).second,"duplicate source");}
        Require(p->sources.count("nominal")&&p->sources.count("gain_evidence"),"missing source");
        r.Tag("model_files");auto files=r.Count(4096);Require(files>0,"missing model closure");
        for(std::size_t i=0;i<files;++i){r.Tag("model_file");auto name=r.Quoted();auto hash=r.Hash();Require(p->model_files.emplace(name,hash).second,"duplicate model file");}
        r.Tag("observation");Require(r.Hash()==p->identity.observation_sha256,"observation digest mismatch");Require(r.Integer<std::int64_t>()==p->start_ns,"observation time mismatch");
        r.Tag("command_authority");Require(r.Integer<int>()==0&&r.Integer<int>()==0,"unexpected command authority");p->command_vx=r.Number();
        r.Tag("period");p->period_s=r.Number();p->phase=r.Number();p->duty=r.Number();Require(r.Integer<int>()==0,"periodic certification unsupported");
        Require(p->period_s>0&&std::abs(p->period_s-span*1e-9)<=1e-10&&p->phase>=0&&p->phase<1&&p->duty>0&&p->duty<1,"invalid period metadata");
        r.Tag("limits");Require(r.Count(5)==5,"limit count mismatch");
        for(int i=0;i<5;++i){r.Tag("limit");auto name=r.Word();auto value=r.Number();Require(value>0&&p->limits.emplace(name,value).second,"invalid limit");}
        for(auto key:{"joint_speed_limit_radps","min_base_height_m","normal_force_limit_n","roll_pitch_limit_rad","torque_limit_nm"})Require(p->limits.count(key),"missing physical limit");
        r.Tag("ordering");Require(r.Count(12)==12,"actuator count mismatch");std::set<int> qaddr,vaddr;std::set<std::string> names,joints;
        for(int i=0;i<12;++i){r.Tag("actuator");Require(r.Integer<int>()==i,"actuator index mismatch");auto& a=p->ordering[i];a.name=r.Quoted();a.joint=r.Quoted();a.qpos=r.Integer<int>();a.qvel=r.Integer<int>();Require(a.qpos>=7&&a.qpos<19&&a.qvel>=6&&a.qvel<18&&qaddr.insert(a.qpos).second&&vaddr.insert(a.qvel).second&&names.insert(a.name).second&&joints.insert(a.joint).second,"invalid actuator ordering");}
        r.Tag("certificates");Require(r.Count(2)==2,"certificate count mismatch");
        for(int i=0;i<2;++i){r.Tag("certificate");auto name=r.Word();Certificate c;c.sha256=r.Hash();c.result_sha256=r.Hash();auto success=r.Integer<int>();Require(success==0||success==1,"invalid certificate flag");c.diagnostic_success=success;auto flags=r.Count(128);std::set<std::string> unique;for(std::size_t f=0;f<flags;++f){r.Tag("failflag");auto s=r.Word();Require(unique.insert(s).second,"duplicate failflag");c.failflags.push_back(s);}Require(c.diagnostic_success==c.failflags.empty(),"certificate verdict mismatch");Require(p->certificates.emplace(name,c).second,"duplicate certificate");}
        Require(p->certificates.count("nominal")&&p->certificates.count("feedback"),"missing certificate");
        Require(p->certificates.at("nominal").result_sha256==p->sources.at("nominal").result_sha256&&p->certificates.at("feedback").result_sha256==p->sources.at("gain_evidence").result_sha256,"certificate source mismatch");
        r.Tag("derivatives");Require(r.Count(4096)==n,"derivative coverage mismatch");p->derivatives.resize(n);
        for(std::size_t i=0;i<n;++i){r.Tag("derivative");Require(r.Count(4096)==i,"derivative index mismatch");auto& d=p->derivatives[i];d.epsilon=r.Number();d.relative_error=r.Number();Require(d.epsilon>0&&d.relative_error>=0,"invalid derivative diagnostic");}
        p->knots.resize(n);
        for(std::size_t i=0;i<n;++i){r.Tag("sample");Require(r.Count(4096)==i,"sample index mismatch");Require(r.Integer<std::int64_t>()==p->start_ns+static_cast<std::int64_t>(i)*p->dt_ns,"sample time mismatch");auto& k=p->knots[i];r.Numbers(k.q);r.Numbers(k.v);r.Numbers(k.tau);r.Numbers(k.gain);Require(Unit(k.q),"nonunit sample quaternion");for(double tau:k.tau)Require(std::abs(tau)<=p->limits.at("torque_limit_nm"),"nominal torque exceeds packet limit");}
        r.Tag("terminal");Require(r.Integer<std::int64_t>()==p->end_ns,"terminal time mismatch");r.Numbers(p->terminal_q);r.Numbers(p->terminal_v);Require(Unit(p->terminal_q),"nonunit terminal quaternion");r.End();
        return {std::make_shared<const Payload>(std::move(*p)),{}};
    } catch(const std::exception& e) { return {nullptr,e.what()}; }
}
// Trusted loader computes actual digests from the exact bytes parsed and the
// corresponding manifest. This API compares them; it does not hash files.
// Binding those bytes and the loaded model to their digests remains an external
// trusted-loader assumption, not a sealed security boundary provided by Verify.
struct ByteBinding { std::string expected_packet_sha256, actual_packet_sha256, expected_manifest_sha256, actual_manifest_sha256; };
struct ExpectedContext {
    Identity identity;
    std::map<std::string,std::string> model_files;
    std::map<std::string,Source> sources;
};
class VerifiedTrajectory;
struct VerifyResult;
VerifyResult Verify(const ParseResult&,const ByteBinding&,const ExpectedContext&,const mjModel*);
class VerifiedTrajectory final {
    std::shared_ptr<const Payload> payload_;
    explicit VerifiedTrajectory(std::shared_ptr<const Payload> p):payload_(std::move(p)){}
    friend VerifyResult Verify(const ParseResult&,const ByteBinding&,const ExpectedContext&,const mjModel*);
public:
    const Payload& payload()const{return *payload_;}
    static constexpr bool can_actuate=false;
};
struct VerifyResult { std::shared_ptr<const VerifiedTrajectory> verified; std::string error; };
inline bool ModelOrderingMatches(const Payload& p,const mjModel* m) {
    if(!m||!std::isfinite(m->opt.timestep)||std::abs(m->opt.timestep-p.dt_ns*1e-9)>1e-12||m->nq!=19||m->nv!=18||m->nu!=12||m->na!=0||m->njnt!=13||m->jnt_type[0]!=mjJNT_FREE||m->jnt_qposadr[0]!=0||m->jnt_dofadr[0]!=0)return false;
    for(int i=0;i<12;++i){
        if(m->actuator_dyntype[i]!=mjDYN_NONE||m->actuator_gaintype[i]!=mjGAIN_FIXED||
           m->actuator_biastype[i]!=mjBIAS_NONE||m->actuator_gainprm[mjNGAIN*i]!=1.0)
            return false;
        for(int axis=0;axis<6;++axis)
            if(m->actuator_gear[6*i+axis]!=(axis==0?1.0:0.0))return false;
        const auto& a=p.ordering[i];const int j=mj_name2id(m,mjOBJ_JOINT,a.joint.c_str());
        if(j<1||m->jnt_type[j]!=mjJNT_HINGE||m->jnt_qposadr[j]!=a.qpos||m->jnt_dofadr[j]!=a.qvel||mj_name2id(m,mjOBJ_ACTUATOR,a.name.c_str())!=i||m->actuator_trntype[i]!=mjTRN_JOINT||m->actuator_trnid[2*i]!=j)return false;}
    return true;
}
inline VerifyResult Verify(const ParseResult& parsed,const ByteBinding& b,const ExpectedContext& c,const mjModel* m) {
    using namespace detail;
    if(!parsed.parsed)return {nullptr,"packet not parsed"};
    const auto& p=*parsed.parsed;
    if(!Hex(b.expected_packet_sha256,64)||!Hex(b.actual_packet_sha256,64)||!Hex(b.expected_manifest_sha256,64)||!Hex(b.actual_manifest_sha256,64)||b.expected_packet_sha256!=b.actual_packet_sha256||b.expected_manifest_sha256!=b.actual_manifest_sha256||p.manifest_sha256!=b.actual_manifest_sha256)return {nullptr,"caller byte binding mismatch"};
    if(!(p.identity==c.identity)||p.model_files!=c.model_files||p.sources.size()!=c.sources.size())return {nullptr,"context identity mismatch"};
    for(const auto& item:p.sources){const auto it=c.sources.find(item.first);if(it==c.sources.end()||it->second.revision!=item.second.revision||it->second.result_sha256!=item.second.result_sha256)return {nullptr,"source identity mismatch"};}
    if(!ModelOrderingMatches(p,m))return {nullptr,"model coordinate ordering mismatch"};
    return {std::shared_ptr<const VerifiedTrajectory>(new VerifiedTrajectory(parsed.parsed)),{}};
}
struct SampleResult { const Knot* knot=nullptr; std::size_t index=0; std::string error; };
inline SampleResult SampleAt(const VerifiedTrajectory& t,const Identity& current_identity,std::int64_t now_ns) {
    const auto& p=t.payload();
    if(!(p.identity==current_identity))return {nullptr,0,"context identity mismatch"};
    if(now_ns<p.start_ns)return {nullptr,0,"future trajectory"};
    if(now_ns>=p.end_ns)return {nullptr,0,"trajectory expired"};
    if((now_ns-p.start_ns)%p.dt_ns!=0)return {nullptr,0,"non-grid sample"};
    const auto i=static_cast<std::size_t>((now_ns-p.start_ns)/p.dt_ns);return {&p.knots[i],i,{}};
}
struct ActualState { std::int64_t time_ns=0;std::array<double,19> q{};std::array<double,18> v{}; };
// Explicit diagnostic tracking box; this is not a robust stability certificate.
struct TrackingLimits { double position_tangent_norm=0, velocity_error_norm=0; };
struct ShadowTorque {
    bool valid=false;
    static constexpr bool can_actuate=false;
    std::array<double,12> candidate_tau{};
    std::array<double,36> error{};
    std::string failure;
};
inline ShadowTorque FeedbackAt(const VerifiedTrajectory& t,const Identity& identity,const mjModel* model,
    std::int64_t now_ns,const ActualState& actual,const TrackingLimits& limits) {
    ShadowTorque out;const auto sample=SampleAt(t,identity,now_ns);
    if(!sample.knot){out.failure=sample.error;return out;}
    if(actual.time_ns!=now_ns||!detail::Finite(actual.q)||!detail::Finite(actual.v)||!detail::Unit(actual.q)){out.failure="invalid actual state";return out;}
    if(!ModelOrderingMatches(t.payload(),model)){out.failure="model coordinate ordering mismatch";return out;}
    if(!std::isfinite(limits.position_tangent_norm)||!std::isfinite(limits.velocity_error_norm)||limits.position_tangent_norm<0||limits.velocity_error_norm<0){out.failure="invalid tracking limits";return out;}
    const auto& k=*sample.knot;mj_differentiatePos(model,out.error.data(),1.0,k.q.data(),actual.q.data());
    double qnorm=0,vnorm=0;for(int i=0;i<18;++i){out.error[i+18]=actual.v[i]-k.v[i];qnorm=std::hypot(qnorm,out.error[i]);vnorm=std::hypot(vnorm,out.error[i+18]);}
    if(!detail::Finite(out.error)||qnorm>limits.position_tangent_norm||vnorm>limits.velocity_error_norm){out.failure="tracking box exceeded";return out;}
    for(int a=0;a<12;++a){out.candidate_tau[a]=k.tau[a];for(int j=0;j<36;++j)out.candidate_tau[a]-=k.gain[a*36+j]*out.error[j];}
    if(!detail::Finite(out.candidate_tau)){out.failure="nonfinite candidate torque";return out;}
    out.valid=true;return out;
}
}}} // namespace
