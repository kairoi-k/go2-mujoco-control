#include "../trot/joint_planning_shadow.h"
#include <fstream>
#include <stdexcept>
struct Reader {
 std::ifstream stream;
 explicit Reader(const char *p):stream(p){if(!stream)throw std::runtime_error("snapshot unavailable");}
 std::string word(){std::string s;if(!(stream>>s))throw std::runtime_error("truncated snapshot");return s;}
 double number(){auto s=word();std::size_t end=0;double x=std::stod(s,&end);if(end!=s.size())throw std::runtime_error("bad number");return x;}
 std::uint64_t integer(){auto s=word();std::size_t end=0;if(s.empty()||s.front()=='-')throw std::runtime_error("bad integer");auto x=std::stoull(s,&end);if(end!=s.size())throw std::runtime_error("bad integer");return x;}
 bool boolean(){auto x=integer();if(x>1)throw std::runtime_error("bad bool");return x!=0;}
 template<class V>void vector(V&v,int n){for(int j=0;j<n;++j)v[j]=number();}
};
int main(int argc,char**argv){try {
 const bool roundtrip=argc==3 && std::string(argv[2])=="--roundtrip";
 if(argc!=2 && !roundtrip)throw std::runtime_error("usage: replay_joint_shadow_snapshot EXTRACTED_SNAPSHOT [--roundtrip]");
 Reader r(argv[1]);if(r.word()!="joint-shadow-snapshot-v1")throw std::runtime_error("unsupported snapshot");
 const auto id=r.integer();const auto pattern=r.integer();if(pattern>static_cast<unsigned>(go2_control::GaitPattern::kRunningTrot))throw std::runtime_error("invalid pattern");
 go2_terrain::TerrainPlannerInput input;input.state_stamp_s=r.number();input.gait_phase=r.number();input.gait_period_s=r.number();input.duty_factor=r.number();input.commanded_vx_mps=r.number();input.base_yaw_rad=r.number();
 go2_control::RigidBodyState state;r.vector(state.position_world,3);std::array<double,4>q;r.vector(q,4);state.quat_world_from_body=Eigen::Quaterniond(q[0],q[1],q[2],q[3]);
 r.vector(state.linear_vel_world,3);r.vector(state.angular_vel_body,3);r.vector(state.q,12);r.vector(state.dq,12);
 input.contact_schedule.measured_valid=r.boolean();for(auto&b:input.contact_schedule.measured_contact)b=r.boolean();
 input.touchdown_target_feet_valid=r.boolean();for(auto&f:input.touchdown_target_feet_base){f.x=r.number();f.y=r.number();f.z=r.number();}
 go2_terrain::TerrainModel m;m.frame_id=r.word();m.source=static_cast<go2_terrain::TerrainSource>(r.integer());m.epoch=r.integer();m.registered=r.boolean();m.map_sequence=r.integer();
 m.state_stamp_s=r.number();m.map_stamp_s=r.number();m.age_s=r.number();m.width=r.integer();m.height=r.integer();m.resolution_m=r.number();
 if(!m.width||!m.height||m.width>1000000/m.height)throw std::runtime_error("invalid cell count");
 r.vector(m.origin_m,2);r.vector(m.registration_position_world,3);m.registration_yaw_rad=r.number();r.vector(m.capture_position_world,3);m.capture_yaw_rad=r.number();
 m.cells.resize(m.width*m.height);for(auto&c:m.cells){c.known=r.boolean();c.height_m=r.number();c.has_height_bounds=r.boolean();c.height_min_m=r.number();c.height_max_m=r.number();c.age_s=r.number();c.slope_rad=r.number();c.roughness_m=r.number();c.variance_m2=r.number();r.vector(c.normal,3);}
 std::string extra;if(r.stream>>extra)throw std::runtime_error("extra snapshot fields");input.terrain=&m;
 if(!state.position_world.allFinite() || !state.linear_vel_world.allFinite() || !state.angular_vel_body.allFinite() || !state.q.allFinite() || !state.dq.allFinite() || !state.quat_world_from_body.coeffs().allFinite() || state.quat_world_from_body.norm()<1e-12)throw std::runtime_error("invalid actual state");
 if(roundtrip){std::cout<<go2_trot::JointShadowSnapshotJson(state,input,id,static_cast<int>(pattern))<<"\n";return 0;}
 go2_trot::JointPlanningShadow shadow;if(!shadow.Load(GO2_MODEL_PATH))throw std::runtime_error("model load");
 shadow.Capture(state,input,id,static_cast<go2_control::GaitPattern>(pattern));return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
