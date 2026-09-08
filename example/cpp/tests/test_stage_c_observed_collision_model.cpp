#include "stage_c/observed_collision_model.h"
#include <iostream>
#ifdef GO2_OBSERVED_MODEL_XML
#include "stage_c/observed_collision_mujoco.h"
#endif
#include <stdexcept>
using namespace go2_terrain;
using namespace go2_terrain::stage_c;
namespace oc=observed_collision;
namespace {
void Check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
CaptureTerrainViewResult View(double rise,std::uint64_t sequence=1,int hole=-1,
                             TerrainSource kind=TerrainSource::kLidar) {
    TerrainMapEnvelope source;
    source.sequence=sequence;source.map_stamp_s=10.;source.frame_id="base_link";
    source.resolution_m=.1;source.width=12;source.height=12;source.origin_m={-.6,-.6};
    source.capture_position_world={2.,-3.,.4};source.capture_yaw_rad=1.5707963267948966;
    source.heights_m.resize(144);source.observation_stamp_s.assign(144,10.);
    for(int y=0;y<12;++y)for(int x=0;x<12;++x)
        source.heights_m[y*12+x]=(x>=6?rise:0.)-.4;
    if(hole>=0){source.heights_m[hole]=kTerrainMapUnknown;source.observation_stamp_s[hole]=kTerrainMapUnknown;}
    auto out=BuildCaptureHeadingTerrainView(source,10.04,7,kind,.2);
    Check(out.ok(),"capture fixture rejected");return out;
}
WorldTerrainSnapshot Snapshot(std::vector<CaptureTerrainViewResult> entries) {
    WorldTerrainSnapshotOptions options;options.stationary_terrain_assumption=true;
    auto out=BuildWorldTerrainSnapshot(7,10.04,entries,options);
    Check(out.ok(),"snapshot fixture rejected");return out.snapshot;
}
}
int main(){try{
    const oc::Query query{2.,-3.,.21};
    const oc::Reconstruction parameters{true,.3,.2};
    for(double rise:{.05,.10}) {
        auto snapshot=Snapshot({View(rise)});
        const auto result=oc::Build(snapshot,query,parameters);
        Check(result.ok(),result.failure.c_str());const auto& d=*result.descriptor;
        Check(!d.can_actuate&&!d.continuous_surface_certified&&!d.support_certified&&
              !d.continuous_sweep_certified&&!d.vertical_walls_observed,"research permissions changed");
#ifdef GO2_OBSERVED_MODEL_XML
        const auto compiled=oc::CompileMujoco(result.descriptor,GO2_OBSERVED_MODEL_XML);
        const auto m=compiled->model();
        char error[2048]{};
        std::unique_ptr<mjModel,oc::ModelDeleter> robot(mj_loadXML(GO2_OBSERVED_MODEL_XML,nullptr,error,sizeof(error)));
        Check(bool(robot),error);
        Check(!compiled->can_actuate&&!compiled->continuous_surface_certified,"model authority changed");
        Check(m->nbody==robot->nbody&&m->nq==robot->nq&&m->nv==robot->nv&&m->nu==robot->nu,"robot dimensions changed");
        for(int i=0;i<m->nbody;++i){
            Check(m->body_mass[i]==robot->body_mass[i],"robot mass changed");
            for(int j=0;j<3;++j)Check(m->body_ipos[3*i+j]==robot->body_ipos[3*i+j],"robot inertial center changed");
            for(int j=0;j<4;++j)Check(m->body_iquat[4*i+j]==robot->body_iquat[4*i+j],"robot inertia frame changed");
            for(int j=0;j<3;++j)Check(m->body_inertia[3*i+j]==robot->body_inertia[3*i+j],"robot inertia changed");
        }
        for(int i=0;i<m->nv;++i)Check(m->dof_damping[i]==robot->dof_damping[i]&&m->dof_armature[i]==robot->dof_armature[i]&&m->dof_frictionloss[i]==robot->dof_frictionloss[i],"passive dynamics changed");
        for(int i=0;i<m->nu*6;++i)Check(m->actuator_gear[i]==robot->actuator_gear[i],"actuator transmission changed");
        for(int i=0;i<m->nu*2;++i)Check(m->actuator_ctrlrange[i]==robot->actuator_ctrlrange[i],"actuator range changed");
        Check(m->opt.timestep==robot->opt.timestep&&m->opt.solver==robot->opt.solver&&m->opt.cone==robot->opt.cone&&m->opt.tolerance==robot->opt.tolerance,"solver options changed");
        std::vector<int> robot_geoms;
        for(int g=0;g<m->ngeom;++g)if(m->geom_bodyid[g])robot_geoms.push_back(g);
        Check(robot_geoms.size()==static_cast<std::size_t>(robot->ngeom),"robot geometry lost");
        for(int g=0;g<robot->ngeom;++g){const int h=robot_geoms[g];
            Check(m->geom_type[h]==robot->geom_type[g]&&m->geom_contype[h]==robot->geom_contype[g]&&m->geom_conaffinity[h]==robot->geom_conaffinity[g]&&m->geom_condim[h]==robot->geom_condim[g]&&m->geom_priority[h]==robot->geom_priority[g]&&m->geom_margin[h]==robot->geom_margin[g],"robot collision contract changed");
            for(int j=0;j<3;++j)Check(m->geom_size[3*h+j]==robot->geom_size[3*g+j]&&m->geom_friction[3*h+j]==robot->geom_friction[3*g+j]&&m->geom_pos[3*h+j]==robot->geom_pos[3*g+j],"robot collision geometry/friction changed");
            for(int j=0;j<4;++j)Check(m->geom_quat[4*h+j]==robot->geom_quat[4*g+j],"robot geom frame changed");
        }
        Check(mj_name2id(m,mjOBJ_GEOM,"phase2_floor")==-1,"scene floor leaked");
        for(const auto& cell:d.prisms()){
            auto name="observed_cell_"+std::to_string(cell.ix)+"_"+std::to_string(cell.iy);
            int geom=mj_name2id(m,mjOBJ_GEOM,name.c_str());
            Check(geom>=0&&m->geom_bodyid[geom]==0&&m->geom_type[geom]==mjGEOM_BOX,"observed box missing");
            Check(std::abs(m->geom_quat[4*geom]-std::cos(cell.yaw_rad*.5))<1e-15&&
                  std::abs(m->geom_quat[4*geom+3]-std::sin(cell.yaw_rad*.5))<1e-15&&
                  m->geom_quat[4*geom+1]==0&&m->geom_quat[4*geom+2]==0,"compiled cell yaw mismatch");
            for(int j=0;j<3;++j)Check(m->geom_pos[3*geom+j]==cell.center_world[j]&&m->geom_size[3*geom+j]==cell.half_size[j],"compiled cell geometry mismatch");
        }
        bool rejected=false;
        try{oc::CompileMujoco(result.descriptor,GO2_OBSERVED_SCENE_XML);}catch(const std::invalid_argument&){rejected=true;}
        Check(rejected,"scene with ground-truth floor accepted");
#endif
        bool low=false,high=false;
        for(const auto& prism:d.prisms()) {
            // Registration serializes grid origin/resolution through float fields
            // (terrain_map_envelope.h:497-502). Test the actual immutable grid,
            // not an ideal decimal grid that the transport cannot represent.
            const auto& grid=snapshot.captures.front().model;
            const double x=grid.origin_m[0]+(prism.ix+.5)*grid.resolution_m;
            const double y=grid.origin_m[1]+(prism.iy+.5)*grid.resolution_m;
            const double top=prism.ix>=6?rise:0.;
            Check(std::abs(prism.center_world[0]-(2.-y))<1e-12&&
                  std::abs(prism.center_world[1]-(-3.+x))<1e-12,"capture yaw/translation mismatch");
            Check(std::abs(prism.representative_top_world_m-top)<1e-6&&
                  std::abs(prism.center_world[2]+prism.half_size[2]-top)<1e-6,"prism top/depth mismatch");
            Check(std::abs(prism.source_min_world_m-top)<1e-12&&
                  std::abs(prism.source_max_world_m-top)<1e-12,"source bounds changed");
            low|=prism.ix<6;high|=prism.ix>=6;
        }
        Check(low&&high,"step sides missing");
        Check(oc::QueryCoverage(d,query,10.04).valid,"original query unavailable");
        Check(!oc::QueryCoverage(d,{2.,-3.,.41},10.04).valid,"geometry coverage silently extended");
        Check(!oc::QueryCoverage(d,{20.,-3.,.01},10.04).valid,"outside map accepted");
        const auto later=oc::QueryCoverage(d,query,10.10);
        Check(later.valid&&later.evaluated_at_s==10.10&&
              std::abs(later.source_query.diagnostic.cell_age_max_s-.10)<1e-12,
              "later query did not advance source age");
        Check(!oc::QueryCoverage(d,query,10.201).valid,"frozen snapshot kept stale source fresh");
        Check(!oc::QueryCoverage(d,query,10.03).valid,"coverage clock moved before construction");
        Check(!oc::QueryCoverage(d,query,kTerrainMapUnknown).valid,"nonfinite coverage clock accepted");
        // Descriptor owns a frozen snapshot; caller changes cannot alter it.
        snapshot.captures[0].model.cells.clear();
        Check(oc::QueryCoverage(d,query,10.04).valid,"mutable snapshot alias escaped");
    }
    const auto flat=Snapshot({View(0.)});
    Check(!oc::Build(flat,query,{}).ok(),"implicit reconstruction accepted");
    Check(!oc::Build(flat,query,{true,0.,.2}).ok(),"zero thickness accepted");
    Check(!oc::Build(flat,query,{true,.3,.001}).ok(),"stale cells accepted");
    Check(!oc::Build(Snapshot({View(.05,1,6*12+6)}),query,parameters).ok(),"unknown hole filled");
    // Complementary missing cells must not be merged across captures.
    Check(!oc::Build(Snapshot({View(0.,1,6*12+6),View(0.,2,5*12+5)}),query,parameters).ok(),
          "partial captures merged into complete geometry");
    const auto fallback=oc::Build(Snapshot({View(0.,1),View(0.,2,6*12+6)}),query,parameters);
    Check(fallback.ok()&&fallback.coverage.history_used&&fallback.coverage.selected_source_sequence==1,
          "complete historical capture not selected");
    const auto latest=oc::Build(Snapshot({View(0.,1),View(0.,2)}),query,parameters);
    Check(latest.ok()&&latest.coverage.selected_source_sequence==2,"latest complete capture not preferred");
    const auto conflict=oc::Build(Snapshot({View(0.,1),View(.10,2)}),query,parameters);
    Check(!conflict.ok()&&conflict.coverage.error==WorldTerrainSnapshotError::kConflictingHistory,
          "conflicting history accepted");
    Check(!oc::Build(Snapshot({View(0.,1,-1,TerrainSource::kTestFixture)}),query,parameters).ok(),
          "fixture source promoted to lidar");
    std::cout<<"observed collision descriptor PASS; discrete coverage only, no physics or authority\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
