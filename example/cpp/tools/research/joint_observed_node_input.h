#pragma once
// Complete joint-shadow-snapshot-v2 reader, aligned with replay_joint_shadow_snapshot.
#include "../../terrain/stage_c/world_terrain_snapshot.h"
#include <istream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
namespace joint_observed_node {
using namespace go2_terrain;using namespace go2_terrain::stage_c;
struct Reader {
 std::istream& stream;std::size_t tokens=0;explicit Reader(std::istream& in):stream(in){}
 std::string word(){std::string s;if(!(stream>>s))throw std::runtime_error("truncated snapshot");++tokens;return s;}
 double number(){auto s=word();std::size_t end=0;double v=std::stod(s,&end);if(end!=s.size())throw std::runtime_error("invalid numeric token");return v;}
 std::uint64_t integer(){auto s=word();std::size_t end=0;if(s.empty()||s[0]=='-'||s[0]=='+')throw std::runtime_error("invalid integer");auto v=std::stoull(s,&end);if(end!=s.size())throw std::runtime_error("invalid integer");return v;}
 bool boolean(){auto v=integer();if(v>1)throw std::runtime_error("invalid bool");return v!=0;}
 template<std::size_t N>void array(std::array<double,N>& values){for(double& v:values)v=number();}
};
inline TerrainModel ReadTerrain(Reader& r){
 TerrainModel m;m.frame_id=r.word();auto source=r.integer();if(source>255)throw std::runtime_error("terrain source overflow");m.source=static_cast<TerrainSource>(source);m.epoch=r.integer();m.registered=r.boolean();m.map_sequence=r.integer();
 m.state_stamp_s=r.number();m.map_stamp_s=r.number();m.age_s=r.number();auto width=r.integer(),height=r.integer();m.resolution_m=r.number();
 if(!width||!height||width>1000000/height)throw std::runtime_error("invalid terrain cell count");m.width=width;m.height=height;
 r.array(m.origin_m);r.array(m.registration_position_world);m.registration_yaw_rad=r.number();r.array(m.capture_position_world);m.capture_yaw_rad=r.number();
 m.cells.resize(m.width*m.height);for(auto& c:m.cells){c.known=r.boolean();c.height_m=r.number();c.has_height_bounds=r.boolean();c.height_min_m=r.number();c.height_max_m=r.number();c.age_s=r.number();c.slope_rad=r.number();c.roughness_m=r.number();c.variance_m2=r.number();r.array(c.normal);}return m;
}
inline std::string Canonical(const TerrainModel& m){
 // All model fields, matching the original replay's comparison; unavailable
 // numeric fields share its null representation without becoming known/zero.
 std::ostringstream o;o<<std::setprecision(17);auto n=[&](double v){if(std::isfinite(v))o<<v;else o<<"null";o<<' ';};auto a=[&](const auto& v){for(double x:v)n(x);};
 o<<std::quoted(m.frame_id)<<' '<<int(m.source)<<' '<<m.epoch<<' '<<m.registered<<' '<<m.map_sequence<<' ';n(m.state_stamp_s);n(m.map_stamp_s);n(m.age_s);o<<m.width<<' '<<m.height<<' ';n(m.resolution_m);a(m.origin_m);a(m.registration_position_world);n(m.registration_yaw_rad);a(m.capture_position_world);n(m.capture_yaw_rad);
 for(const auto& c:m.cells){o<<c.known<<' ';n(c.height_m);o<<c.has_height_bounds<<' ';n(c.height_min_m);n(c.height_max_m);n(c.age_s);n(c.slope_rad);n(c.roughness_m);n(c.variance_m2);a(c.normal);}return o.str();
}
struct Snapshot {
 std::uint64_t id=0,pattern=0;double time=0,phase=0,period=0,duty=0,vx=0,yaw=0;
 std::array<double,3> position{},linear_velocity{},angular_velocity{};std::array<double,4> quaternion{};std::array<double,12> joints{},joint_velocities{},target_feet{};
 bool measured_valid=false,target_feet_valid=false;std::array<bool,4> measured_contact{};TerrainModel latest;WorldTerrainSnapshot history;std::size_t tokens=0;
};
inline Snapshot Parse(std::istream& stream){
 Reader r(stream);if(r.word()!="joint-shadow-snapshot-v2")throw std::runtime_error("complete v2 snapshot required");Snapshot s;s.id=r.integer();s.pattern=r.integer();if(s.pattern>1)throw std::runtime_error("unsupported gait pattern");
 s.time=r.number();s.phase=r.number();s.period=r.number();s.duty=r.number();s.vx=r.number();s.yaw=r.number();r.array(s.position);r.array(s.quaternion);r.array(s.linear_velocity);r.array(s.angular_velocity);r.array(s.joints);r.array(s.joint_velocities);
 s.measured_valid=r.boolean();for(auto& b:s.measured_contact)b=r.boolean();s.target_feet_valid=r.boolean();r.array(s.target_feet);s.latest=ReadTerrain(r);
 auto epoch=r.integer();double stamp=r.number();WorldTerrainSnapshotOptions options;options.stationary_terrain_assumption=r.boolean();options.height_conflict_tolerance_m=r.number();options.minimum_normal_dot=r.number();auto count=r.integer();if(!count||count>kWorldTerrainSnapshotMaxCaptures)throw std::runtime_error("invalid history count");
 std::vector<CaptureTerrainViewResult> views;for(std::uint64_t i=0;i<count;++i){CaptureTerrainViewResult view;view.model=ReadTerrain(r);const auto& m=view.model;auto& p=view.provenance;p.source_sequence=m.map_sequence;p.map_epoch=m.epoch;p.source_map_stamp_s=m.map_stamp_s;p.state_stamp_s=m.state_stamp_s;p.capture_position_world=m.capture_position_world;p.capture_yaw_rad=m.capture_yaw_rad;p.source=m.source;for(const auto& c:m.cells)view.source_known_cells+=c.known;view.view_known_cells=view.source_known_cells;view.coverage_preserved=true;view.valid=true;view.error=CaptureTerrainViewError::kNone;views.push_back(std::move(view));}
 auto built=BuildWorldTerrainSnapshot(epoch,stamp,std::move(views),options);if(!built.ok())throw std::runtime_error(std::string("invalid history: ")+WorldTerrainSnapshotErrorName(built.error));s.history=std::move(built.snapshot);
 if(stamp!=s.time||!s.history.latest_model()||Canonical(s.latest)!=Canonical(*s.history.latest_model()))throw std::runtime_error("history latest/state mismatch");
 std::string extra;if(stream>>extra)throw std::runtime_error("extra snapshot fields");s.tokens=r.tokens;
 auto finite=[](const auto& v){return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});};
 if(!std::isfinite(s.time)||!std::isfinite(s.phase)||!std::isfinite(s.period)||!std::isfinite(s.duty)||!std::isfinite(s.vx)||!std::isfinite(s.yaw)||!finite(s.position)||!finite(s.quaternion)||!finite(s.linear_velocity)||!finite(s.angular_velocity)||!finite(s.joints)||!finite(s.joint_velocities))throw std::runtime_error("nonfinite observation");
 double norm=0;for(double x:s.quaternion)norm+=x*x;if(!std::isfinite(norm)||norm<1e-24)throw std::runtime_error("invalid observation quaternion");return s;
}
}
