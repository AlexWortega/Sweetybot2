// Native MuJoCo servo stepping. No floating-base support or privileged control.
#include <mujoco/mujoco.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif
namespace py = pybind11;
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
using Indices = py::array_t<int64_t, py::array::c_style | py::array::forcecast>;
struct ModelDelete { void operator()(mjModel* m) const { mj_deleteModel(m); } };
struct DataDelete { void operator()(mjData* d) const { mj_deleteData(d); } };
struct Environment {
  std::unique_ptr<mjModel, ModelDelete> model;
  std::unique_ptr<mjData, DataDelete> data;
  std::array<double, 8> params{1., .9, 9.9, 1., 1., .0174533, 1., 1.};
  std::array<std::array<double,16>,4> queue{};
  std::array<double,16> torque{}, rms{}, peak{}, max_speed{};
  std::array<double,16> heat{},continuous{},caps{};
  std::array<double,30> imu_gyro{},imu_accel{};
  std::array<double,40> foot_force{};
  std::array<double,64> depth{};
  double depth_clock=0.,depth_age=0.;
  int head=0;
};
class Batch {
  int count_, threads_, nq_, nv_;
  std::vector<std::unique_ptr<Environment>> envs_;
  std::array<int,16> joints_{}, qids_{}, dids_{}, motors_{};
  std::array<int,4> feet_sites_{}, feet_geoms_{},force_sensors_{};
  int gyro_adr_,accel_adr_,quat_adr_,tof_site_;double tof_half_tangent_=.45;
  std::array<std::array<double,8>,16> specs_{};
  void range_scan(Environment& e) const {
    const auto* m=e.model.get();const auto* d=e.data.get();
    const double* origin=d->site_xpos+3*tof_site_;const double* R=d->site_xmat+9*tof_site_;
    const mjtByte group[6]{1,0,0,1,0,0};
    for(int row=0;row<8;++row) for(int col=0;col<8;++col) {
      double local[3]{((col+.5)/8.*2-1)*tof_half_tangent_,(1-(row+.5)/8.*2)*tof_half_tangent_,-1.};
      double ray[3]{};for(int a=0;a<3;++a)for(int b=0;b<3;++b)ray[a]+=R[3*a+b]*local[b];
      mju_normalize3(ray);int geom=-1;e.depth[row*8+col]=mj_ray(m,d,origin,ray,group,1,-1,&geom);
    }
    e.depth_age=0.;
  }
  std::array<double,16> lower_{}, upper_{};
  std::vector<double> mass_, inertia_;
  static int lookup(const mjModel* m, mjtObj kind, const std::string& name) {
    int id=mj_name2id(m,kind,name.c_str());
    if(id<0) throw std::runtime_error("Missing MuJoCo name: "+name);
    return id;
  }
  void validate_shape(const py::buffer_info& b, ssize_t rows, ssize_t cols, const char* name) const {
    if(b.ndim!=2 || b.shape[0]!=rows || b.shape[1]!=cols)
      throw std::invalid_argument(std::string(name)+" has incorrect shape");
    const double* p=static_cast<const double*>(b.ptr);
    for(ssize_t i=0;i<rows*cols;++i) if(!std::isfinite(p[i]))
      throw std::invalid_argument(std::string(name)+" contains a nonfinite value");
  }
 public:
  Batch(const std::string& path, int n, int threads):count_(n),threads_(threads) {
    if(n<1 || threads<1) throw std::invalid_argument("num_envs and threads must be positive");
    char error[2048]{};
    std::unique_ptr<mjModel,ModelDelete> base(mj_loadXML(path.c_str(),nullptr,error,sizeof(error)));
    if(!base) throw std::runtime_error(error);
    nq_=base->nq;nv_=base->nv;
    if(base->nu!=16 || nq_!=23 || nv_!=22) throw std::invalid_argument("Expected floating SweetieMini with 16 joints");
    if(std::abs(base->opt.timestep-.002)>1e-12) throw std::invalid_argument("Expected 500 Hz MuJoCo timestep");
    const std::array<std::string,4> legs{"fl","fr","rl","rr"};
    std::vector<std::string> names;
    for(const auto& leg:legs) for(const auto& joint:{"hip_roll","hip_pitch","knee"}) names.push_back(leg+"_"+joint);
    for(const auto& joint:{"head_yaw","head_pitch","ear_left","ear_right"}) names.push_back(joint);
    for(int j=0;j<16;++j) {
      joints_[j]=lookup(base.get(),mjOBJ_JOINT,names[j]);
      qids_[j]=base->jnt_qposadr[joints_[j]];dids_[j]=base->jnt_dofadr[joints_[j]];
      motors_[j]=lookup(base.get(),mjOBJ_ACTUATOR,names[j]);
      lower_[j]=base->jnt_range[2*joints_[j]];upper_[j]=base->jnt_range[2*joints_[j]+1];
    }
    for(int i=0;i<4;++i) {
      feet_sites_[i]=lookup(base.get(),mjOBJ_SITE,legs[i]+"_foot");
      feet_geoms_[i]=lookup(base.get(),mjOBJ_GEOM,legs[i]+"_sole_contact");
      force_sensors_[i]=base->sensor_adr[lookup(base.get(),mjOBJ_SENSOR,legs[i]+"_contact_force")];
    }
    gyro_adr_=base->sensor_adr[lookup(base.get(),mjOBJ_SENSOR,"imu_gyro")];
    accel_adr_=base->sensor_adr[lookup(base.get(),mjOBJ_SENSOR,"imu_accel")];
    quat_adr_=base->sensor_adr[lookup(base.get(),mjOBJ_SENSOR,"imu_quat")];
    tof_site_=lookup(base.get(),mjOBJ_SITE,"chest_tof");
    int spec=lookup(base.get(),mjOBJ_NUMERIC,"motor_specs");
    if(base->numeric_size[spec]!=128)throw std::invalid_argument("motor_specs must contain 16x8 values");
    for(int j=0;j<16;++j)for(int k=0;k<8;++k)specs_[j][k]=base->numeric_data[base->numeric_adr[spec]+j*8+k];
    int tangent=lookup(base.get(),mjOBJ_NUMERIC,"tof_half_tangent");tof_half_tangent_=base->numeric_data[base->numeric_adr[tangent]];
    mass_.assign(base->body_mass,base->body_mass+base->nbody);
    inertia_.assign(base->body_inertia,base->body_inertia+3*base->nbody);
    envs_.reserve(n);
    for(int i=0;i<n;++i) {
      auto e=std::make_unique<Environment>();e->model.reset(mj_copyModel(nullptr,base.get()));
      e->data.reset(mj_makeData(e->model.get()));
      if(!e->data) throw std::runtime_error("MuJoCo data allocation failed");
      if(base->nkey) mj_resetDataKeyframe(e->model.get(),e->data.get(),0);
      else mj_resetData(e->model.get(),e->data.get());
      mj_forward(e->model.get(),e->data.get());
      for(auto& row:e->queue) for(int j=0;j<16;++j) row[j]=e->data->qpos[qids_[j]];
      envs_.push_back(std::move(e));
    }
  }
  int nq() const { return nq_; } int nv() const { return nv_; }
  int njoints() const { return 16; } int num_envs() const { return count_; }
  int threads() const {
#ifdef _OPENMP
    return threads_;
#else
    return 1;
#endif
  }
  void reset(Indices indices, Array qpos, Array qvel, Array params) {
    auto ib=indices.request();if(ib.ndim!=1) throw std::invalid_argument("indices must be one-dimensional");
    int k=static_cast<int>(ib.shape[0]);auto qb=qpos.request(),vb=qvel.request(),pb=params.request();
    validate_shape(qb,k,nq_,"qpos");validate_shape(vb,k,nv_,"qvel");validate_shape(pb,k,8,"params");
    const auto* ix=static_cast<const int64_t*>(ib.ptr);const auto* q=static_cast<const double*>(qb.ptr);
    const auto* v=static_cast<const double*>(vb.ptr);const auto* p=static_cast<const double*>(pb.ptr);
    std::vector<bool> used(count_,false);
    for(int i=0;i<k;++i) {
      if(ix[i]<0 || ix[i]>=count_ || used[ix[i]]) throw std::invalid_argument("Invalid or duplicate reset index");
      used[ix[i]]=true;const double* z=p+8*i;
      if(z[0]<=0 || z[1]<0 || z[2]<9.3 || z[2]>10.8 || z[3]<=0 || z[4]<=0 || z[5]<0 || z[6]<0 || z[6]>3 || z[6]!=std::floor(z[6]) || z[7]<=0)
        throw std::invalid_argument("Invalid domain randomization parameters");
      double norm=0;for(int j=3;j<7;++j) norm+=q[i*nq_+j]*q[i*nq_+j];
      if(norm<1e-12) throw std::invalid_argument("Root quaternion must be nonzero");
    }
    py::gil_scoped_release release;
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_) schedule(static)
#endif
    for(int i=0;i<k;++i) {
      auto& e=*envs_[ix[i]];auto* m=e.model.get();auto* d=e.data.get();
      std::copy(p+8*i,p+8*(i+1),e.params.begin());
      for(int b=1;b<m->nbody;++b) m->body_mass[b]=mass_[b]*e.params[0];
      for(int b=3;b<3*m->nbody;++b) m->body_inertia[b]=inertia_[b]*e.params[0];
      for(int g=0;g<m->ngeom;++g) m->geom_friction[3*g]=e.params[1];
      mj_resetData(m,d);mj_setConst(m,d);
      std::copy(q+i*nq_,q+(i+1)*nq_,d->qpos);std::copy(v+i*nv_,v+(i+1)*nv_,d->qvel);
      mj_normalizeQuat(m,d->qpos);mj_forward(m,d);e.head=0;
      for(auto& row:e.queue) for(int j=0;j<16;++j) row[j]=d->qpos[qids_[j]];
      e.torque.fill(0);e.rms.fill(0);e.peak.fill(0);e.max_speed.fill(0);e.heat.fill(0);
      e.depth_clock=0.;range_scan(e);e.imu_gyro.fill(0);e.imu_accel.fill(0);e.foot_force.fill(0);
    }
  }
  void step(Array targets) {
    auto b=targets.request();validate_shape(b,count_,16,"targets");const auto* targets_ptr=static_cast<const double*>(b.ptr);
    py::gil_scoped_release release;
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_) schedule(static)
#endif
    for(int i=0;i<count_;++i) {
      auto& e=*envs_[i];auto* m=e.model.get();auto* d=e.data.get();
      for(int j=0;j<16;++j) e.queue[e.head][j]=std::clamp(targets_ptr[i*16+j],lower_[j],upper_[j]);
      const auto& target=e.queue[(e.head-static_cast<int>(e.params[6])+4)%4];
      e.rms.fill(0);e.peak.fill(0);e.max_speed.fill(0);

      for(int sub=0;sub<10;++sub) {
        for(int j=0;j<16;++j) {
          const auto& spec=specs_[j];const double voltage=spec[6]>0?spec[6]:e.params[2];
          const double vf=std::min(1.,voltage/spec[3]);
          const double stall=spec[0]*vf*e.params[3],omega=spec[2]*vf*e.params[4];
          const double continuous=spec[1]*vf*e.params[3]*std::max(.05,1.-std::abs(d->qvel[dids_[j]])/omega);
          const double burst=.55*stall;
          const double cap=std::min(burst,continuous+(burst-continuous)*std::clamp(1.-e.heat[j],0.,1.));
          e.continuous[j]=continuous;e.caps[j]=cap;
          const double kp=spec[4]*e.params[7],kd=spec[5]*e.params[7];
          double velocity=d->qvel[dids_[j]],error=target[j]-d->qpos[qids_[j]];
          error=std::copysign(std::max(0.,std::abs(error)-e.params[5]),error);
          // Above no-load, allow bounded braking but never accelerating torque.
          // Saturating each directional bound also keeps lo<=hi beyond 1.55*omega.
          const double lo=std::clamp(-stall*(1.+velocity/omega),-cap,cap);
          const double hi=std::clamp(stall*(1.-velocity/omega),-cap,cap);
          const double tau=std::clamp(kp*error-kd*velocity,lo,hi);
          e.heat[j]=std::max(0.,e.heat[j]+m->opt.timestep*(std::pow(tau/std::max(.01,continuous),2)-e.heat[j])/spec[7]);
          d->ctrl[motors_[j]]=tau;e.torque[j]=tau;e.rms[j]+=tau*tau;e.peak[j]=std::max(e.peak[j],std::abs(tau));
        }
        mj_step(m,d);
        for(int a=0;a<3;++a) {e.imu_gyro[sub*3+a]=d->sensordata[gyro_adr_+a];e.imu_accel[sub*3+a]=d->sensordata[accel_adr_+a];}
        for(int f=0;f<4;++f)e.foot_force[sub*4+f]=d->sensordata[force_sensors_[f]];
        e.depth_clock+=m->opt.timestep;e.depth_age+=m->opt.timestep;
        if(e.depth_clock>=1./15.) {e.depth_clock-=1./15.;range_scan(e);}
        for(int j=0;j<16;++j) e.max_speed[j]=std::max(e.max_speed[j],std::abs(d->qvel[dids_[j]]));
      }
      for(double& value:e.rms) value=std::sqrt(value/10.);
      // mj_step integrates qpos after computing contacts/sites; refresh derived
      // outputs once at the policy boundary rather than returning stale feet.
      mj_forward(m,d);
      e.head=(e.head+1)%4;
    }
  }
  void push(Array impulses) {
    auto b=impulses.request();validate_shape(b,count_,6,"dv");const auto* p=static_cast<const double*>(b.ptr);
    py::gil_scoped_release release;
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_) schedule(static)
#endif
    for(int i=0;i<count_;++i) for(int j=0;j<6;++j) envs_[i]->data->qvel[j]+=p[6*i+j];
  }
  py::dict state() const {
    py::array_t<double> qp({count_,nq_}),qv({count_,nv_}),feet({count_,4,3}),torque({count_,16}),rms({count_,16}),peak({count_,16}),speed({count_,16});
    py::array_t<double> gyro({count_,10,3}),accel({count_,10,3}),forces({count_,10,4}),depth({count_,64}),age(count_),quat({count_,4}),heat({count_,16}),continuous({count_,16}),caps({count_,16});
    auto gy=gyro.mutable_unchecked<3>(),ac=accel.mutable_unchecked<3>(),fo=forces.mutable_unchecked<3>();
    auto dep=depth.mutable_unchecked<2>(),qu=quat.mutable_unchecked<2>(),he=heat.mutable_unchecked<2>(),ct=continuous.mutable_unchecked<2>(),ca=caps.mutable_unchecked<2>();auto ag=age.mutable_unchecked<1>();
    py::array_t<bool> contacts({count_,4});py::array_t<int> bad(count_);
    auto qp_=qp.mutable_unchecked<2>(),qv_=qv.mutable_unchecked<2>();
    auto ft=feet.mutable_unchecked<3>();
    auto tq=torque.mutable_unchecked<2>(),rm=rms.mutable_unchecked<2>(),sp=speed.mutable_unchecked<2>();
    auto pk=peak.mutable_unchecked<2>();
    auto co=contacts.mutable_unchecked<2>();auto bc=bad.mutable_unchecked<1>();
    for(int i=0;i<count_;++i) {
      const auto& e=*envs_[i];const auto* m=e.model.get();const auto* d=e.data.get();
      for(int tick=0;tick<10;++tick) {for(int a=0;a<3;++a){gy(i,tick,a)=e.imu_gyro[tick*3+a];ac(i,tick,a)=e.imu_accel[tick*3+a];}for(int f=0;f<4;++f)fo(i,tick,f)=e.foot_force[tick*4+f];}
      for(int f=0;f<4;++f)qu(i,f)=d->sensordata[quat_adr_+f];
      for(int p=0;p<64;++p)dep(i,p)=e.depth[p];ag(i)=e.depth_age;
      for(int j=0;j<16;++j){he(i,j)=e.heat[j];ct(i,j)=e.continuous[j];ca(i,j)=e.caps[j];}
      for(int j=0;j<nq_;++j) qp_(i,j)=d->qpos[j];for(int j=0;j<nv_;++j) qv_(i,j)=d->qvel[j];
      for(int f=0;f<4;++f) {co(i,f)=false;for(int j=0;j<3;++j) ft(i,f,j)=d->site_xpos[3*feet_sites_[f]+j];}
      bc(i)=0;
      for(int c=0;c<d->ncon;++c) {
        const auto& contact=d->contact[c];int g1=contact.geom1,g2=contact.geom2;
        if(contact.dist>0) continue;
        int body_geom=-1;
        if(m->geom_bodyid[g1]==0 && m->geom_bodyid[g2]!=0) body_geom=g2;
        else if(m->geom_bodyid[g2]==0 && m->geom_bodyid[g1]!=0) body_geom=g1;
        if(body_geom<0) continue;bool foot=false;
        for(int f=0;f<4;++f) if(body_geom==feet_geoms_[f]) {co(i,f)=true;foot=true;}
        if(!foot) ++bc(i);
      }
      for(int j=0;j<16;++j) {tq(i,j)=e.torque[j];rm(i,j)=e.rms[j];pk(i,j)=e.peak[j];sp(i,j)=e.max_speed[j];}
    }
    py::dict out;out["qpos"]=qp;out["qvel"]=qv;out["feet"]=feet;out["contacts"]=contacts;
    out["imu_gyro_samples"]=gyro;out["imu_accel_samples"]=accel;out["imu_quat"]=quat;
    out["foot_force_samples"]=forces;out["depth_ranges"]=depth;out["depth_age"]=age;out["motor_heat"]=heat;out["motor_continuous"]=continuous;out["motor_cap"]=caps;
    out["bad_contacts"]=bad;out["torque"]=torque;out["torque_rms"]=rms;out["torque_peak"]=peak;out["max_speed"]=speed;return out;
  }
};
PYBIND11_MODULE(_sweetie_native,m) {
  py::class_<Batch>(m,"Batch")
    .def(py::init<const std::string&,int,int>(),py::arg("xml_path"),py::arg("num_envs"),py::arg("threads")=1)
    .def("reset",&Batch::reset,py::arg("indices"),py::arg("qpos"),py::arg("qvel"),py::arg("params"))
    .def("step",&Batch::step,py::arg("targets")).def("state",&Batch::state).def("push",&Batch::push,py::arg("dv"))
    .def_property_readonly("nq",&Batch::nq).def_property_readonly("nv",&Batch::nv)
    .def_property_readonly("njoints",&Batch::njoints).def_property_readonly("num_envs",&Batch::num_envs)
    .def_property_readonly("threads",&Batch::threads);
}
