#pragma once
// Worker-local research model. Reconstructed terrain is not a surface certificate.
// robot_xml is a trusted caller input: RobotOnly validates topology, not its
// recursive source identity. Caller must independently bind XML/asset hashes
// before compiling; this API does not certify arbitrary same-shaped robot files.
#include "observed_collision_model.h"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
namespace go2_terrain { namespace stage_c { namespace observed_collision {
struct ModelDeleter { void operator()(const mjModel* m)const { mj_deleteModel(const_cast<mjModel*>(m)); } };
class MujocoModel;
std::shared_ptr<const MujocoModel> CompileMujoco(std::shared_ptr<const Descriptor>,const std::string&);
class MujocoModel final {
    const std::shared_ptr<const Descriptor> descriptor_;
    const std::unique_ptr<const mjModel,ModelDeleter> model_;
    MujocoModel(std::shared_ptr<const Descriptor> d,mjModel* m):descriptor_(std::move(d)),model_(m){}
    friend std::shared_ptr<const MujocoModel> CompileMujoco(std::shared_ptr<const Descriptor>,const std::string&);
public:
    static constexpr bool can_actuate=false;
    static constexpr bool continuous_surface_certified=false;
    const mjModel* model()const{return model_.get();}
    const Descriptor& descriptor()const{return *descriptor_;}
};
inline bool RobotOnly(const mjModel* m) {
    if(!m||m->nq!=19||m->nv!=18||m->nu!=12||m->njnt!=13||m->na!=0||
       m->nplugin!=0||m->jnt_type[0]!=mjJNT_FREE||m->opt.timestep!=.002)return false;
    const int root=mj_name2id(m,mjOBJ_BODY,"base_link");
    if(root<=0||m->jnt_bodyid[0]!=root||m->body_parentid[root]!=0)return false;
    std::vector<bool> robot(m->nbody,false);robot[root]=true;
    for(int i=root+1;i<m->nbody;++i)robot[i]=robot[m->body_parentid[i]];
    for(int i=1;i<m->nbody;++i)if(!robot[i])return false;
    // Reject imported world geometry outright, including invisible scene terrain.
    for(int g=0;g<m->ngeom;++g)if(!robot[m->geom_bodyid[g]])return false;
    return true;
}
namespace detail {
struct MergedPrism {
    CellPrism prism{};
    std::size_t width_cells=1;
    std::size_t height_cells=1;
};
inline bool SameValue(double a,double b) {
    // Numerical coordinate identity only; this tolerance is not a terrain
    // height acceptance margin.
    return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=1e-12;
}
inline bool SameMergeClass(const CellPrism& a,const CellPrism& b) {
    // CellPrism is a horizontal box, so a sloped source plane is not
    // substituted for equal-height geometry.
    // The selected descriptor comes from one capture/source. Preserve its
    // source bounds in addition to equal world height; source normals remain
    // per-cell evidence and never rotate or replace this horizontal box.
    if(!SameValue(a.yaw_rad,b.yaw_rad))return false;
    if(!SameValue(a.representative_top_world_m,b.representative_top_world_m)||
       !SameValue(a.center_world[2],b.center_world[2]))return false;
    if(!SameValue(a.half_size[0],b.half_size[0])||
       !SameValue(a.half_size[1],b.half_size[1])||
       !SameValue(a.half_size[2],b.half_size[2]))return false;
    return SameValue(a.source_min_world_m-a.representative_top_world_m,
                     b.source_min_world_m-b.representative_top_world_m)&&
        SameValue(a.source_max_world_m-a.representative_top_world_m,
                  b.source_max_world_m-b.representative_top_world_m);
}
inline std::vector<MergedPrism> MergeAdjacentPrisms(
    const std::vector<CellPrism>& input) {
    std::vector<MergedPrism> output;
    if(input.empty())return output;
    const std::size_t npos=std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> order;
    order.reserve(input.size());
    for(std::size_t i=0;i<input.size();++i)order.push_back(i);
    std::sort(order.begin(),order.end(),[&](std::size_t a,std::size_t b) {
        return input[a].iy==input[b].iy ? input[a].ix<input[b].ix :
            input[a].iy<input[b].iy;
    });
    std::vector<bool> visited(input.size(),false);
    const auto index_at=[&](std::size_t ix,std::size_t iy) {
        for(std::size_t i=0;i<input.size();++i)
            if(input[i].ix==ix&&input[i].iy==iy)return i;
        return npos;
    };
    const auto increment=[](std::size_t value,std::size_t delta,
                            std::size_t& result) {
        if(delta>std::numeric_limits<std::size_t>::max()-value)return false;
        result=value+delta;return true;
    };
    for(const std::size_t seed:order) {
        if(visited[seed])continue;
        const CellPrism& base=input[seed];
        std::size_t width=1;
        for(;;) {
            std::size_t ix=0;
            if(!increment(base.ix,width,ix))break;
            const std::size_t candidate=index_at(ix,base.iy);
            if(candidate==npos||visited[candidate]||
               !SameMergeClass(base,input[candidate]))break;
            ++width;
        }
        std::size_t height=1;
        for(;;) {
            std::size_t iy=0;
            if(!increment(base.iy,height,iy))break;
            bool complete=true;
            for(std::size_t dx=0;dx<width;++dx) {
                std::size_t ix=0;
                if(!increment(base.ix,dx,ix)) {complete=false;break;}
                const std::size_t candidate=index_at(ix,iy);
                if(candidate==npos||visited[candidate]||
                   !SameMergeClass(base,input[candidate])) {
                    complete=false;break;
                }
            }
            if(!complete)break;
            ++height;
        }
        MergedPrism merged;
        merged.prism=base;
        merged.width_cells=width;
        merged.height_cells=height;
        std::array<double,3> center_sum{0.,0.,0.};
        std::size_t count=0;
        for(std::size_t dy=0;dy<height;++dy) {
            std::size_t iy=0;
            if(!increment(base.iy,dy,iy))continue;
            for(std::size_t dx=0;dx<width;++dx) {
                std::size_t ix=0;
                if(!increment(base.ix,dx,ix))continue;
                const std::size_t candidate=index_at(ix,iy);
                if(candidate==npos)continue;
                visited[candidate]=true;
                for(int axis=0;axis<3;++axis)
                    center_sum[axis]+=input[candidate].center_world[axis];
                ++count;
            }
        }
        if(count==0)continue;
        for(int axis=0;axis<3;++axis)
            merged.prism.center_world[axis]=center_sum[axis]/
                static_cast<double>(count);
        merged.prism.half_size[0]*=static_cast<double>(width);
        merged.prism.half_size[1]*=static_cast<double>(height);
        output.push_back(merged);
    }
    return output;
}
inline bool SameCoplanarRegionGeometry(const MergedPrism& a,
                                       const MergedPrism& b) {
    if(a.width_cells==0||a.height_cells==0||b.width_cells==0||b.height_cells==0)
        return false;
    // This is a residual-seam diagnostic, so source bounds and normals do not
    // suppress a report: the compiled representation is a horizontal box.
    return SameValue(a.prism.yaw_rad,b.prism.yaw_rad) &&
        SameValue(a.prism.representative_top_world_m,
                  b.prism.representative_top_world_m) &&
        SameValue(a.prism.center_world[2],b.prism.center_world[2]) &&
        SameValue(a.prism.half_size[2],b.prism.half_size[2]);
}
inline bool HasCoplanarInternalSeam(const std::vector<MergedPrism>& regions) {
    // Greedy rectangles are complete only when no same-class rectangles still
    // share a grid edge. A T-junction is retained as a diagnostic boundary.
    for(std::size_t i=0;i<regions.size();++i) {
        for(std::size_t j=i+1;j<regions.size();++j) {
            const auto& a=regions[i];
            const auto& b=regions[j];
            if(!SameCoplanarRegionGeometry(a,b))continue;
            if(a.prism.ix>std::numeric_limits<std::size_t>::max()-a.width_cells||
               a.prism.iy>std::numeric_limits<std::size_t>::max()-a.height_cells||
               b.prism.ix>std::numeric_limits<std::size_t>::max()-b.width_cells||
               b.prism.iy>std::numeric_limits<std::size_t>::max()-b.height_cells)
                continue;
            const bool x_overlap=a.prism.ix<b.prism.ix+b.width_cells&&
                b.prism.ix<a.prism.ix+a.width_cells;
            const bool y_overlap=a.prism.iy<b.prism.iy+b.height_cells&&
                b.prism.iy<a.prism.iy+a.height_cells;
            const bool x_adj=a.prism.ix+a.width_cells==b.prism.ix||
                b.prism.ix+b.width_cells==a.prism.ix;
            const bool y_adj=a.prism.iy+a.height_cells==b.prism.iy||
                b.prism.iy+b.height_cells==a.prism.iy;
            if((x_adj&&y_overlap)||(y_adj&&x_overlap))return true;
        }
    }
    return false;
}
}
inline std::shared_ptr<const MujocoModel> CompileMujoco(
    std::shared_ptr<const Descriptor> descriptor,const std::string& robot_xml) {
    if(!descriptor||descriptor->prisms().empty())throw std::invalid_argument("missing observed descriptor");
    char error[2048]{};
    std::unique_ptr<mjSpec,decltype(&mj_deleteSpec)> spec(mj_parseXML(robot_xml.c_str(),nullptr,error,sizeof(error)),mj_deleteSpec);
    if(!spec)throw std::runtime_error(error);
    std::unique_ptr<mjModel,ModelDeleter> robot(mj_compile(spec.get(),nullptr));
    if(!robot)throw std::runtime_error(mjs_getError(spec.get()));
    if(!RobotOnly(robot.get()))throw std::invalid_argument("requires robot-only Go2 model without scene geometry");
    auto base=mjs_findBody(spec.get(),"base_link");
    auto world=base?mjs_getParent(base->element):nullptr;
    if(!world)throw std::runtime_error("missing robot world parent");
    const auto merged=detail::MergeAdjacentPrisms(descriptor->prisms());
    if(merged.empty())throw std::invalid_argument("missing observed geometry");
    if(detail::HasCoplanarInternalSeam(merged))
        throw std::invalid_argument("unsupported coplanar internal seam in observed terrain");
    for(const auto& region:merged) {
        const auto& cell=region.prism;
        auto geom=mjs_addGeom(world,nullptr);
        if(!geom)throw std::runtime_error("cannot add observed cell");
        auto name="observed_cell_"+std::to_string(cell.ix)+"_"+std::to_string(cell.iy);
        if(region.width_cells>1||region.height_cells>1)
            name+="_merged_"+std::to_string(region.width_cells)+"x"+
                std::to_string(region.height_cells);
        if(mjs_setName(geom->element,name.c_str()))throw std::runtime_error("cannot name observed cell");
        geom->type=mjGEOM_BOX;
        for(int i=0;i<3;++i){geom->pos[i]=cell.center_world[i];geom->size[i]=cell.half_size[i];}
        geom->quat[0]=std::cos(cell.yaw_rad*.5);geom->quat[1]=0;geom->quat[2]=0;geom->quat[3]=std::sin(cell.yaw_rad*.5);
        // Explicit MuJoCo ground defaults; robot's higher-priority foot settings
        // remain untouched. These are modeling parameters, not sensed friction.
        geom->contype=1;geom->conaffinity=1;geom->condim=3;geom->priority=0;
        geom->friction[0]=1.;geom->friction[1]=.005;geom->friction[2]=.0001;
        geom->margin=0.;geom->gap=0.;
    }
    std::unique_ptr<mjModel,ModelDeleter> compiled(mj_compile(spec.get(),nullptr));
    if(!compiled)throw std::runtime_error(mjs_getError(spec.get()));
    if(compiled->nq!=robot->nq||compiled->nv!=robot->nv||compiled->nu!=robot->nu||
       compiled->nbody!=robot->nbody||compiled->ngeom!=robot->ngeom+static_cast<int>(merged.size()))
        throw std::runtime_error("observed geometry changed robot topology");
    return std::shared_ptr<const MujocoModel>(new const MujocoModel(std::move(descriptor),compiled.release()));
}
}}}
