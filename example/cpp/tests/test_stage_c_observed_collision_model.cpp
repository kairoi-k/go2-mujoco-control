#include "stage_c/observed_collision_model.h"
#include <iostream>
#ifdef GO2_OBSERVED_MODEL_XML
#include "stage_c/observed_collision_mujoco.h"
#endif
#include <memory>
#include <string>
#include <stdexcept>
using namespace go2_terrain;
using namespace go2_terrain::stage_c;
namespace oc=observed_collision;
namespace {
void Check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
CaptureTerrainViewResult View(double rise,std::uint64_t sequence=1,int hole=-1,
                             TerrainSource kind=TerrainSource::kLidar,
                             double x_slope=0.) {
    TerrainMapEnvelope source;
    source.sequence=sequence;source.map_stamp_s=10.;source.frame_id="base_link";
    source.resolution_m=.1;source.width=12;source.height=12;source.origin_m={-.6,-.6};
    source.capture_position_world={2.,-3.,.4};source.capture_yaw_rad=1.5707963267948966;
    source.heights_m.resize(144);source.observation_stamp_s.assign(144,10.);
    for(int y=0;y<12;++y)for(int x=0;x<12;++x)
        source.heights_m[y*12+x]=(x>=6?rise:0.)-.4+
            x_slope*static_cast<double>(x);
    if(hole>=0){source.heights_m[hole]=kTerrainMapUnknown;source.observation_stamp_s[hole]=kTerrainMapUnknown;}
    auto out=BuildCaptureHeadingTerrainView(source,10.04,7,kind,.2);
    Check(out.ok(),"capture fixture rejected");return out;
}
WorldTerrainSnapshot Snapshot(std::vector<CaptureTerrainViewResult> entries) {
    WorldTerrainSnapshotOptions options;options.stationary_terrain_assumption=true;
    auto out=BuildWorldTerrainSnapshot(7,10.04,entries,options);
    Check(out.ok(),"snapshot fixture rejected");return out.snapshot;
}
#ifdef GO2_OBSERVED_MODEL_XML
std::unique_ptr<mjModel,oc::ModelDeleter> CompileGroundReference(
    const std::string& robot_xml,bool plane,const oc::CellPrism* box=nullptr) {
    char error[2048]{};
    std::unique_ptr<mjSpec,decltype(&mj_deleteSpec)> spec(
        mj_parseXML(robot_xml.c_str(),nullptr,error,sizeof(error)),mj_deleteSpec);
    Check(bool(spec),error);
    auto base=mjs_findBody(spec.get(),"base_link");
    auto world=base?mjs_getParent(base->element):nullptr;
    Check(world!=nullptr,"robot world missing in ground reference");
    auto geom=mjs_addGeom(world,nullptr);
    Check(geom!=nullptr,"ground reference geometry creation failed");
    Check(mjs_setName(geom->element,plane?"reference_plane":"reference_box")==0,
          "ground reference naming failed");
    geom->type=plane?mjGEOM_PLANE:mjGEOM_BOX;
    if(plane) {
        geom->size[0]=100.;geom->size[1]=100.;geom->size[2]=.1;
        if(box)geom->pos[2]=box->center_world[2]+box->half_size[2];
    } else {
        Check(box!=nullptr,"single-box reference missing geometry");
        for(int i=0;i<3;++i){geom->pos[i]=box->center_world[i];geom->size[i]=box->half_size[i];}
        geom->quat[0]=std::cos(box->yaw_rad*.5);geom->quat[1]=0;
        geom->quat[2]=0;geom->quat[3]=std::sin(box->yaw_rad*.5);
    }
    geom->contype=1;geom->conaffinity=1;geom->condim=3;geom->priority=0;
    geom->friction[0]=1.;geom->friction[1]=.005;geom->friction[2]=.0001;
    geom->margin=0.;geom->gap=0.;
    mjModel* compiled=mj_compile(spec.get(),nullptr);
    Check(compiled!=nullptr,mjs_getError(spec.get()));
    return std::unique_ptr<mjModel,oc::ModelDeleter>(compiled);
}
struct ForwardContactSummary { int contacts=0; double normal_force=0.; };
ForwardContactSummary ForwardContacts(const mjModel* model) {
    std::unique_ptr<mjData,decltype(&mj_deleteData)> data(
        mj_makeData(model),mj_deleteData);
    Check(bool(data),"MuJoCo data allocation failed");
    data->qpos[0]=2.;data->qpos[1]=-3.;
    mj_forward(model,data.get());
    ForwardContactSummary out;out.contacts=data->ncon;
    for(int i=0;i<data->ncon;++i) {
        const auto& contact=data->contact[i];
        if(model->geom_bodyid[contact.geom1]!=0&&
           model->geom_bodyid[contact.geom2]!=0)continue;
        double force[6]{};mj_contactForce(model,data.get(),i,force);
        out.normal_force+=std::abs(force[0]);
    }
    return out;
}
std::unique_ptr<mjModel,oc::ModelDeleter> CompileSphereGround(
    bool plane,const oc::CellPrism* box=nullptr) {
    constexpr const char* kProbeXml=R"xml(
<mujoco model="stage_c_probe">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="probe" pos="0 0 0.2">
      <freejoint name="probe_free"/>
      <geom name="probe_geom" type="sphere" size="0.05" mass="1"
            contype="1" conaffinity="1" condim="3" friction="1 .005 .0001"/>
    </body>
  </worldbody>
</mujoco>)xml";
    char error[2048]{};
    std::unique_ptr<mjSpec,decltype(&mj_deleteSpec)> spec(
        mj_parseXMLString(kProbeXml,nullptr,error,sizeof(error)),mj_deleteSpec);
    Check(bool(spec),error);
    auto probe=mjs_findBody(spec.get(),"probe");
    auto world=probe?mjs_getParent(probe->element):nullptr;
    Check(world!=nullptr,"probe world missing in ground reference");
    auto geom=mjs_addGeom(world,nullptr);
    Check(geom!=nullptr,"probe ground geometry creation failed");
    Check(mjs_setName(geom->element,plane?"probe_plane":"probe_box")==0,
          "probe ground naming failed");
    geom->type=plane?mjGEOM_PLANE:mjGEOM_BOX;
    if(plane) {
        geom->size[0]=100.;geom->size[1]=100.;geom->size[2]=.1;
        if(box)geom->pos[2]=box->center_world[2]+box->half_size[2];
    } else {
        Check(box!=nullptr,"probe box reference missing geometry");
        for(int i=0;i<3;++i){geom->pos[i]=box->center_world[i];geom->size[i]=box->half_size[i];}
        geom->quat[0]=std::cos(box->yaw_rad*.5);geom->quat[1]=0;
        geom->quat[2]=0;geom->quat[3]=std::sin(box->yaw_rad*.5);
    }
    geom->contype=1;geom->conaffinity=1;geom->condim=3;geom->priority=0;
    geom->friction[0]=1.;geom->friction[1]=.005;geom->friction[2]=.0001;
    geom->margin=0.;geom->gap=0.;
    mjModel* compiled=mj_compile(spec.get(),nullptr);
    Check(compiled!=nullptr,mjs_getError(spec.get()));
    return std::unique_ptr<mjModel,oc::ModelDeleter>(compiled);
}
struct ProbeSummary {
    int contacts=0;
    double normal_force=0.;
    double qacc_x=0.;
    double qacc_y=0.;
    double qacc_z=0.;
    double qpos_x=0.;
    double qpos_y=0.;
    double qpos_z=0.;
    double qvel_x=0.;
    double qvel_y=0.;
    double qvel_z=0.;
};
ProbeSummary ProbeForward(const mjModel* model,double x,double y,double z,
                          bool step) {
    std::unique_ptr<mjData,decltype(&mj_deleteData)> data(
        mj_makeData(model),mj_deleteData);
    Check(bool(data),"probe MuJoCo data allocation failed");
    const int joint=mj_name2id(model,mjOBJ_JOINT,"probe_free");
    Check(joint>=0,"probe free joint missing");
    const int qposadr=model->jnt_qposadr[joint];
    const int dofadr=model->jnt_dofadr[joint];
    data->qpos[qposadr]=x;data->qpos[qposadr+1]=y;data->qpos[qposadr+2]=z;
    data->qpos[qposadr+3]=1.;data->qpos[qposadr+4]=0.;
    data->qpos[qposadr+5]=0.;data->qpos[qposadr+6]=0.;
    mj_forward(model,data.get());
    if(step) {
        mj_step(model,data.get());
        // mj_step integrates qpos/qvel; refresh qacc/contact for the new state.
        mj_forward(model,data.get());
    }
    ProbeSummary out;out.contacts=data->ncon;
    out.qacc_x=data->qacc[dofadr];out.qacc_y=data->qacc[dofadr+1];
    out.qacc_z=data->qacc[dofadr+2];
    out.qpos_x=data->qpos[qposadr];out.qpos_y=data->qpos[qposadr+1];
    out.qpos_z=data->qpos[qposadr+2];
    out.qvel_x=data->qvel[dofadr];out.qvel_y=data->qvel[dofadr+1];
    out.qvel_z=data->qvel[dofadr+2];
    for(int i=0;i<data->ncon;++i) {
        const auto& contact=data->contact[i];
        if(model->geom_bodyid[contact.geom1]!=0&&
           model->geom_bodyid[contact.geom2]!=0)continue;
        double force[6]{};mj_contactForce(model,data.get(),i,force);
        out.normal_force+=std::abs(force[0]);
    }
    return out;
}
#endif
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
        const auto merged=oc::detail::MergeAdjacentPrisms(d.prisms());
        Check(!merged.empty(),"observed merge unexpectedly empty");
        Check(m->ngeom==robot->ngeom+static_cast<int>(merged.size()),
              "merged geometry count mismatch");
        for(const auto& region:merged){
            const auto& cell=region.prism;
            auto name="observed_cell_"+std::to_string(cell.ix)+"_"+std::to_string(cell.iy);
            if(region.width_cells>1||region.height_cells>1)
                name+="_merged_"+std::to_string(region.width_cells)+"x"+
                    std::to_string(region.height_cells);
            int geom=mj_name2id(m,mjOBJ_GEOM,name.c_str());
            Check(geom>=0&&m->geom_bodyid[geom]==0&&m->geom_type[geom]==mjGEOM_BOX,"observed box missing");
            Check(std::abs(m->geom_quat[4*geom]-std::cos(cell.yaw_rad*.5))<1e-15&&
                  std::abs(m->geom_quat[4*geom+3]-std::sin(cell.yaw_rad*.5))<1e-15&&
                  m->geom_quat[4*geom+1]==0&&m->geom_quat[4*geom+2]==0,"compiled cell yaw mismatch");
            for(int j=0;j<3;++j)Check(m->geom_pos[3*geom+j]==cell.center_world[j]&&m->geom_size[3*geom+j]==cell.half_size[j],"compiled cell geometry mismatch");
        }
        if(rise==.05||rise==.10) {
            bool found_low=false,found_high=false;
            for(const auto& region:merged) {
                const auto& cell=region.prism;
                if(cell.ix<6)found_low=true;
                if(cell.ix>=6)found_high=true;
                Check((cell.ix<6&&std::abs(cell.representative_top_world_m)<1e-6)||
                      (cell.ix>=6&&std::abs(cell.representative_top_world_m-rise)<1e-6),
                      "step top was averaged across an edge");
            }
            Check(found_low&&found_high,"step sides were merged together");
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
#ifdef GO2_OBSERVED_MODEL_XML
    const auto flat_for_physics=Snapshot({View(0.)});
    const auto flat_build=oc::Build(flat_for_physics,query,parameters);
    Check(flat_build.ok(),"flat physics descriptor rejected");
    const auto flat_regions=oc::detail::MergeAdjacentPrisms(
        flat_build.descriptor->prisms());
    Check(flat_regions.size()==1&&
              flat_regions.front().width_cells*flat_regions.front().height_cells==
                  flat_build.descriptor->prisms().size(),
          "complete flat rectangle was not merged");
    const auto flat_model=oc::CompileMujoco(flat_build.descriptor,
                                             GO2_OBSERVED_MODEL_XML);
    char robot_error[2048]{};
    std::unique_ptr<mjModel,oc::ModelDeleter> flat_robot(
        mj_loadXML(GO2_OBSERVED_MODEL_XML,nullptr,robot_error,sizeof(robot_error)));
    Check(bool(flat_robot),robot_error);
    const auto single_box=CompileGroundReference(
        GO2_OBSERVED_MODEL_XML,false,&flat_regions.front().prism);
    const auto plane=CompileGroundReference(GO2_OBSERVED_MODEL_XML,true,&flat_regions.front().prism);
    const auto flat_contacts=ForwardContacts(flat_model->model());
    const auto box_contacts=ForwardContacts(single_box.get());
    const auto plane_contacts=ForwardContacts(plane.get());
    Check(flat_contacts.contacts>0&&flat_contacts.normal_force>0.,
          "flat mj_forward produced no observed contact");
    Check(flat_contacts.contacts==box_contacts.contacts&&
              std::abs(flat_contacts.normal_force-box_contacts.normal_force)<1e-9,
          "merged flat box disagrees with single-box mj_forward");
    std::cerr.precision(17);std::cerr<<"plane force "<<plane_contacts.normal_force<<" box "<<flat_contacts.normal_force<<"\n";
    Check(plane_contacts.contacts>0&&
              std::abs(flat_contacts.normal_force-plane_contacts.normal_force)<1e-3,
          "merged flat box disagrees with plane mj_forward");
    Check(flat_regions.front().width_cells%2==0&&
              flat_regions.front().height_cells%2==0,
          "flat probe fixture has no centered cell seam");
    // Translate both fixtures away from the observed near-zero plane offset discrepancy.
    // Gravity and relative contact geometry are invariant under this translation.
    auto flat_cell=flat_regions.front().prism;
    flat_cell.center_world[2]+=1.;
    const auto probe_box=CompileSphereGround(false,&flat_cell);
    const auto probe_plane=CompileSphereGround(true,&flat_cell);
    const double probe_z=1.+.05-.002;
    const double seam_x=flat_cell.center_world[0];
    const double seam_y=flat_cell.center_world[1];
    const double c=std::cos(flat_cell.yaw_rad),s=std::sin(flat_cell.yaw_rad);
    for(double offset:{0.,.002}) {
        const double x=seam_x+c*offset;
        const double y=seam_y+s*offset;
        const auto merged_probe=ProbeForward(probe_box.get(),x,y,probe_z,false);
        const auto plane_probe=ProbeForward(probe_plane.get(),x,y,probe_z,false);
        std::cerr<<"sphere "<<offset<<" az "<<merged_probe.qacc_z<<" vs "<<plane_probe.qacc_z<<" normal "<<merged_probe.normal_force<<" vs "<<plane_probe.normal_force<<"\n";
        Check(merged_probe.contacts>0&&plane_probe.contacts>0,
              "probe seam mj_forward produced no contact");
        Check(std::hypot(merged_probe.qacc_x,merged_probe.qacc_y)<1e-6&&
                  std::hypot(plane_probe.qacc_x,plane_probe.qacc_y)<1e-6&&
                  std::abs(merged_probe.qacc_z-plane_probe.qacc_z)<1e-9,
              "merged box introduced seam qacc");
        Check(std::abs(merged_probe.normal_force-plane_probe.normal_force)<1e-9,
              "merged box introduced seam normal-force difference");
    }
    const auto merged_step=ProbeForward(probe_box.get(),seam_x,seam_y,probe_z,true);
    const auto plane_step=ProbeForward(probe_plane.get(),seam_x,seam_y,probe_z,true);
    Check(std::isfinite(merged_step.qacc_x)&&std::isfinite(merged_step.qacc_y)&&
              std::isfinite(merged_step.qacc_z)&&
              std::isfinite(plane_step.qacc_x)&&std::isfinite(plane_step.qacc_y)&&
              std::isfinite(plane_step.qacc_z),
          "probe mj_step produced nonfinite acceleration");
    Check(std::hypot(merged_step.qacc_x,merged_step.qacc_y)<1e-3&&
              std::hypot(plane_step.qacc_x,plane_step.qacc_y)<1e-3,
          "probe mj_step introduced lateral seam acceleration");
    Check(std::abs(merged_step.qpos_x-plane_step.qpos_x)<1e-6&&
              std::abs(merged_step.qpos_y-plane_step.qpos_y)<1e-6&&
              std::abs(merged_step.qpos_z-plane_step.qpos_z)<1e-6&&
              std::abs(merged_step.qvel_x-plane_step.qvel_x)<1e-6&&
              std::abs(merged_step.qvel_y-plane_step.qvel_y)<1e-6&&
              std::abs(merged_step.qvel_z-plane_step.qvel_z)<1e-6,
          "probe mj_step state differs at seam");
    auto synthetic_cell=[](std::size_t ix,std::size_t iy) {
        oc::CellPrism cell;cell.ix=ix;cell.iy=iy;
        cell.center_world={.1*static_cast<double>(ix)+.05,
                           .1*static_cast<double>(iy)+.05,0.};
        cell.half_size={.05,.05,.1};cell.yaw_rad=0.;
        cell.representative_top_world_m=0.;cell.source_min_world_m=0.;
        cell.source_max_world_m=0.;cell.source_normal_world={0.,0.,1.};
        return cell;
    };
    const std::vector<oc::CellPrism> l_shape{
        synthetic_cell(0,0),synthetic_cell(1,0),synthetic_cell(0,1)};
    const auto l_regions=oc::detail::MergeAdjacentPrisms(l_shape);
    Check(oc::detail::HasCoplanarInternalSeam(l_regions),
          "T-junction seam was silently accepted");
    auto metadata_left=synthetic_cell(0,0),metadata_right=synthetic_cell(1,0);
    metadata_right.source_min_world_m=-.001;
    const auto metadata_regions=oc::detail::MergeAdjacentPrisms({metadata_left,metadata_right});
    Check(metadata_regions.size()==2&&oc::detail::HasCoplanarInternalSeam(metadata_regions),
          "metadata difference hid a physical coplanar seam");
    const auto sloped=oc::Build(Snapshot({View(0.,1,-1,TerrainSource::kLidar,.01)}),
                                query,parameters);
    Check(sloped.ok(),"sloped source descriptor rejected");
    const auto sloped_regions=oc::detail::MergeAdjacentPrisms(
        sloped.descriptor->prisms());
    Check(sloped_regions.size()>1,"sloped cells were merged into one box");
    for(const auto& region:sloped_regions)
        Check(region.width_cells==1,"sloped cells merged across unequal heights");
    bool unknown_rejected=false;
    try {
        const auto unknown=oc::Build(Snapshot({View(0.,1,6*12+6)}),query,parameters);
        unknown_rejected=!unknown.ok();
    } catch(const std::exception&) { unknown_rejected=true; }
    Check(unknown_rejected,"unknown hole was filled for collision compilation");
#endif
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
    std::cout<<"observed collision descriptor PASS; merged mj_forward regression PASS; research-only, no authority\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
