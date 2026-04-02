# aircraft.py
#
# A Python class containing aircraft characteristics
#
# Written by First Last
# Other contributors: Bradley Denby, Darshan Sarojini, Dylan Hogge, John Riris, Khoa Nguyen
#
# See the LICENSE file for the license

# import Python modules
import copy # deepcopy
import json # json parsing
import math # log10, pi
import sys  # not needed when using as a package

# path to directory with other classes; use before deploying as package
sys.path.append('../evtol')
from environ import Environ
from mission import Mission
from power import Power
from propulsion import Propulsion

# comment above and uncomment below when ready to deploy as package
#from .environ import Environ
#from .mission import Mission
#from .power import Power
#from .propulsion import Propulsion

# constants
W_P_KW = 1000.0
S_P_HR = 3600.0
KG_2_LB = 2.20462
M_2_FT = 3.28084
M_P_S_2_KTS = 1.9438
N_P_M2_2_LB_P_FT2 = 0.0209

# Aircraft class
class Aircraft:
  # class constructor
  def __init__(self, path_to_json: str):
    # open and load JSON specification
    ifile = open(path_to_json, 'r')
    ijson = json.load(ifile)
    # aircraft properties
    self._max_takeoff_mass_kg = ijson['aircraft']['max_takeoff_mass_kg']
    self._payload_kg = ijson['aircraft']['payload_kg']
    self._personelle_payload_kg = ijson['aircraft']['personelle_payload_kg']
    self._vehicle_cl_max = ijson['aircraft']['vehicle_cl_max']
    self._wing_taper_ratio = ijson['aircraft']['wing_taper_ratio']
    self._wingspan_m = ijson['aircraft']['wingspan_m']
    self._d_value_m = ijson['aircraft']['d_value_m']
    self._stall_speed_m_p_s = ijson['aircraft']['stall_speed_m_p_s']
    self._fuselage_l_m = ijson['aircraft']['fuselage_l_m']
    self._fuselage_w_m = ijson['aircraft']['fuselage_w_m']
    self._fuselage_h_m = ijson['aircraft']['fuselage_h_m']
    self._wing_airfoil_cd_at_cruise_cl = \
     ijson['aircraft']['wing_airfoil_cd_at_cruise_cl']
    self._empennage_airfoil_cd0 = ijson['aircraft']['empennage_airfoil_cd0']
    self._span_effic_factor = ijson['aircraft']['span_effic_factor']
    self._trim_drag_factor = ijson['aircraft']['trim_drag_factor']
    self._landing_gear_drag_area_m2 = \
     ijson['aircraft']['landing_gear_drag_area_m2']
    self._excres_protub_factor = ijson['aircraft']['excres_protub_factor']
    self._horiz_tail_vol_coeff = ijson['aircraft']['horiz_tail_vol_coeff']
    self._vert_tail_vol_coeff = ijson['aircraft']['vert_tail_vol_coeff']
    self._ratio_disk_to_stopped_rotor_area = \
     ijson['aircraft']['ratio_disk_to_stopped_rotor_area']
    self._wing_t_p_c = ijson['aircraft']['wing_t_p_c']
    self._actuator_mass_kg = ijson['aircraft']['actuator_mass_kg']
    self._furnishings_mass_kg = ijson['aircraft']['furnishings_mass_kg']
    self._environmental_control_system_mass_kg = \
     ijson['aircraft']['environmental_control_system_mass_kg']
    self._avionics_mass_kg = ijson['aircraft']['avionics_mass_kg']
    self._hivolt_power_dist_mass_kg = \
     ijson['aircraft']['hivolt_power_dist_mass_kg']
    self._lovolt_power_coms_mass_kg = \
     ijson['aircraft']['lovolt_power_coms_mass_kg']
    self._mass_margin_factor = ijson['aircraft']['mass_margin_factor']
    # has-a classes: add classes if they exist in JSON
    self._environ = None
    if 'environ' in ijson:
      self._environ = Environ(path_to_json)
    self._mission = None
    if 'mission' in ijson:
      self._mission = Mission(path_to_json)
    self._power = None
    if 'power' in ijson:
      self._power = Power(path_to_json)
    self._propulsion = None
    if 'propulsion' in ijson:
      self._propulsion = Propulsion(path_to_json)

    # close JSON file
    ifile.close()

  # ratio of payload mass to max takeoff mass
  def _calc_payload_mass_frac(self):
    return (self.payload_kg+self.personelle_payload_kg)/self.max_takeoff_mass_kg

  # requires disk_area_m2 from propulsion
  # use MTOM to calculate kg per disk area m2
  # return None if propulsion object not populated
  def _calc_disk_loading_kg_p_m2(self):
    if self.propulsion != None:
      return self.max_takeoff_mass_kg/self.propulsion.disk_area_m2
    else:
      return None

  # requires environ g_m_p_s2, air_density_sea_lvl_kg_p_m3
  # requires propulsion disk_area_m2, rotor_effic
  # prop thrust momentum theory:_calc_hover_shaft_power_kw
  #   F = change in pressure * disk area
  #   change in pressure = 0.5 * air density * (v_e^2 - v_0^2); hover means v_0=0
  #   so F = 0.5 * air density * v_e^2 * disk area
  #   note: v_e is far-wake velocity, disk induced velocity is v_i = v_e/2
  #   for hover: T = m*g = 2 * air density * disk area * v_i^2
  #   induced power = T * v_i = (m*g)^(3/2) / sqrt(2 * air density * disk area)
  #   therefore hover shaft power in watts is:
  #   ((m*g)^1.5 / (2*air density*disk area)^0.5) / hover_power_effic
  # return None if environ or power object not populated
  def _calc_hover_shaft_power_kw(self):
    if self.environ != None and self.power != None:
      return \
       ((
        (self.environ.g_m_p_s2*self.max_takeoff_mass_kg)**1.5/
        (2.0*self.environ.air_density_sea_lvl_kg_p_m3*\
         self.propulsion.disk_area_m2)**0.5
       )/self.power.hover_power_effic)/W_P_KW
    else:
      return None

  # requires aircraft hover_shaft_power_kw
  # return None if aircraft field or power object not populated
  def _calc_hover_fuel_consumption(self):
    if self.hover_shaft_power_kw != None and self.power != None:
      return self.hover_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

  # requires environ air_density_max_alt_kg_p_m3
  # stall speed equation solved for wing area
  # return None if environ object not populated
  def _calc_wing_area_m2(self):
    if self.environ != None:
      return \
       (2.0*self.max_takeoff_mass_kg*self.environ.g_m_p_s2)/\
       (self.environ.air_density_max_alt_kg_p_m3*(self.stall_speed_m_p_s**2.0)*\
        self.vehicle_cl_max)
    else:
      return None

  # requires mission cruise_h_m_p_s 
  # both cruise speeds initial and return should be same so same 
  # with or without payload since we are simulating a tank to start this cl will stay same
  # this calculation replaces stall speed with cruise speed
  # see stall speed equation
  # return None if mission object not populated
  def _calc_cruise_cl(self):
    if self.mission != None:
      return \
       (self.stall_speed_m_p_s**2.0)*self.vehicle_cl_max/\
       (self.mission.cruise_h_m_p_s**2.0)
    else:
      return None

  # Hoerner Eq 13.1 (p 238)
  def _calc_fuselage_fineness_ratio(self):
    return 2.0*self.fuselage_l_m/(self.fuselage_w_m+self.fuselage_h_m)

  # Hoerner Eq 6.31 (p 111)
  def _calc_fuselage_cd0_p_cf(self):
    return \
     3.0*self.fuselage_fineness_ratio+4.5/self.fuselage_fineness_ratio**0.5+\
     21.0/self.fuselage_fineness_ratio**2.0

  # requires environ kinematic_viscosity_max_alt_m2_p_s
  # requires mission cruise_h_m_p_s again speeds should be same
  # reynolds number = velocity*length/kinematic viscosity
  # return None if environ or mission object not populated
  def _calc_fuselage_cruise_reynolds(self):
    if self.environ != None and self.mission != None:
      return \
       self.mission.cruise_h_m_p_s*self.fuselage_l_m/\
       self.environ.kinematic_viscosity_max_alt_m2_p_s
    else:
      return None

  # requires aircraft fuselage_cruise_reynolds
  # Prandtl-Schlichting skin friction formula
  # where other additive terms are negligible in this regime
  # return None if aircraft field not populated
  def _calc_fuselage_cf(self):
    if self.fuselage_cruise_reynolds != None:
      return 0.455/math.log10(self.fuselage_cruise_reynolds)**2.58
      # negligible: 
      # +0.0016*self.fuselage_fineness_ratio/self.fuselage_cruise_reynolds**0.4
    else:
      return None

  # requires aircraft fuselage_cf and aircraft wing_area_m2
  # dimensional analysis
  # fuselage_cd0 = fuselage_cda/wing_area_m2
  # where fuselage_cda = fuselage_cd0_p_cf*fuselage_cf*fuselage_reference_area
  # where fuselage_reference_area = pi*((fuselage_w+fuselage_h)/4)**2
  # return None if aircraft field not populated
  def _calc_fuselage_cd0(self):
    if self.fuselage_cf != None and self.wing_area_m2 != None:
      fuselage_reference_area = math.pi*((self.fuselage_w_m+self.fuselage_h_m)/4.0)**2.0
      return (
        self.fuselage_cd0_p_cf*self.fuselage_cf*fuselage_reference_area/self.wing_area_m2
      )
    else:
      return None

  # requires aircraft wing_area_m2
  # wing aspect ratio = wingspan^2/wing area
  # return None if aircraft field not populated
  def _calc_wing_aspect_ratio(self):
    if self.wing_area_m2 != None:
      return self.wingspan_m**2.0/self.wing_area_m2
    else:
      return None

  # requires aircraft cruise_cl, wing_aspect_ratio
  # induced drag coefficient equation
  # return None if aircraft field not populated
  def _calc_induced_drag_cdi(self):
    if self.cruise_cl != None and self.wing_aspect_ratio != None:
      return \
       self.cruise_cl**2.0/\
       (math.pi*self.wing_aspect_ratio*self.span_effic_factor)
    else:
      return None

  # requires aircraft wing_area_m2
  # recall wing aspect ratio = wingspan^2/wing area
  # return None if aircraft field not populated
  def _calc_wing_root_chord_m(self):
    if self.wing_area_m2 != None:
      return 2.0*self.wing_area_m2/(self.wingspan_m*(1.0+self.wing_taper_ratio))
    else:
      return None

  # requires aircraft wing_root_chord_m
  # wing Mean Aerodynamic Chord formula
  # return None if aircraft field not populated
  def _calc_wing_mac_m(self):
    if self.wing_root_chord_m != None:
      return \
       (2.0/3.0)*self.wing_root_chord_m*\
       (1.0+self.wing_taper_ratio**2.0/(1.0+self.wing_taper_ratio))
    else:
      return None

  # requires aircraft wing_area_m2 and wing_mac_m
  # return None if aircraft field not populated
  def _calc_horiz_tail_area_m2(self):
    if self.wing_area_m2 != None and self.wing_mac_m != None:
      return \
       (self.horiz_tail_vol_coeff*self.wing_area_m2*self.wing_mac_m)/\
       (0.5*self.fuselage_l_m)
    else:
      return None

  # requires aircraft wing_area_m2
  # return None if aircraft field not populated
  def _calc_vert_tail_area_m2(self):
    if self.wing_area_m2 != None :
      return \
       (self.vert_tail_vol_coeff*self.wingspan_m*self.wing_area_m2)/\
       (0.5*self.fuselage_l_m)
    else:
      return None

  # requires aircraft horiz_tail_area_m2
  # return None if aircraft field not populated
  def _calc_horiz_tail_cd0(self):
    if self.horiz_tail_area_m2 != None and self.vert_tail_area_m2 != None:
      return (\
       self.horiz_tail_area_m2/(self.wing_area_m2)\
       )*self.empennage_airfoil_cd0
    else:
      return None

  # requires aircraft vert_tail_area_m2
  # return None if aircraft field not populated
  def _calc_vert_tail_cd0(self):
    if self.horiz_tail_area_m2 != None and self.vert_tail_area_m2 != None:
      return (\
       self.vert_tail_area_m2/(self.wing_area_m2)\
       )*self.empennage_airfoil_cd0
    else:
      return None

  # requires aircraft wing_area_m2
  # return None if aircraft field not populated
  def _calc_landing_gear_cd0(self):
    if self.wing_area_m2 != None:
      return self.landing_gear_drag_area_m2/self.wing_area_m2
    else:
      return None

  # requires aircraft wing_area_m2
  # requires propulsion disk_area_m2
  # return None if aircraft field or propulsion object not populated
  def _calc_stopped_rotor_cd0(self):
    if self.wing_area_m2 != None and self.propulsion.disk_area_m2 != None:
      return \
       (self.propulsion.disk_area_m2/self.ratio_disk_to_stopped_rotor_area)/\
       self.wing_area_m2
    else:
      return None

  # requires aircraft fuselage_cd0, induced_drag_cdi, horiz_tail_cd0,
  # vert_tail_cd0, landing_gear_cd0, stopped_rotor_cd0
  # per-component drag buildup
  # return None if aircraft field(s) not populated
  def _calc_cruise_cd(self):
    if self.fuselage_cd0 != None and self.induced_drag_cdi != None and \
       self.horiz_tail_cd0 != None and self.vert_tail_cd0 != None and \
       self.landing_gear_cd0 != None and self.stopped_rotor_cd0 != None:
      return (\
       self.fuselage_cd0+self.wing_airfoil_cd_at_cruise_cl+\
       self.induced_drag_cdi+self.horiz_tail_cd0+self.vert_tail_cd0+\
       self.landing_gear_cd0+self.stopped_rotor_cd0)*self.trim_drag_factor*\
       self.excres_protub_factor
    else:
      return None

  # requires aircraft cruise_cl and cruise_cd
  # dimensional analysis
  # return None if aircraft field not populated
  def _calc_cruise_l_p_d(self):
    if self.cruise_cl != None and self.cruise_cd != None:
      return self.cruise_cl/self.cruise_cd
    else:
      return None
  
  # requires aircraft fields for fuselage, empennage, landing gear, etc.
  # returns total drag coefficient
  def _calc_total_drag_coef(self):
    if self.environ == None or self.wing_area_m2 == None:
      return None
    cd0_sum = 0.0
    if self.fuselage_cd0 != None:
      cd0_sum += self.fuselage_cd0
    if self.horiz_tail_cd0 != None:
      cd0_sum += self.horiz_tail_cd0
    if self.vert_tail_cd0 != None:
      cd0_sum += self.vert_tail_cd0
    if self.landing_gear_cd0 != None:
      cd0_sum += self.landing_gear_cd0
    return cd0_sum

  # Hoerner Eq 6.30 (p 111)
  def _calc_fuselage_wetted_area_m2(self):
    if self.fuselage_cf != None and self.wing_area_m2 != None:
      fuselage_reference_area = math.pi*((self.fuselage_w_m+self.fuselage_h_m)/4.0)**2.0
      return 3*self.fuselage_fineness_ratio*fuselage_reference_area
    else:
      return None
  
  # calculates the over-torque factor for the propulsion system.
  def _calc_over_torque_factor(self):
    if self.propulsion == None:
      return None
    else:
      return self.propulsion.rotor_count/(self.propulsion.rotor_count-1)+0.3

  # Parametric EPU mass estimation model (FHE / Magicall datasheet based)
  # Uses hover torque and over-torque scaling to compute motor torque at max thrust
  # Scales rotor RPM to account for sea-level vs. minimum air density conditions
  # Computes maximum motor power and applies empirical regression to estimate single EPU mass
  def _calc_single_epu_mass_kg(self):
    if self.propulsion == None and self.environ == None:
      return None
    else:
      # Hover torque
      rpm_hover_rpm = (self.environ.sound_speed_m_p_s*self.propulsion.tip_mach/(self.propulsion.rotor_diameter_m/2.0))*60.0/(2.0*math.pi)
      omega_hover_rad_s = 2.0*math.pi*rpm_hover_rpm/60.0
      torque_hover_nm = (self.hover_shaft_power_kw*1000.0/self.propulsion.rotor_count)/omega_hover_rad_s
      torque_max_nm = self.over_torque_factor * torque_hover_nm

      # Max RPM (min density)
      rpm_hover_sl_rpm = rpm_hover_rpm  # assuming hover calc is at sea-level
      rpm_max_rpm = rpm_hover_sl_rpm*math.sqrt(self.environ.air_density_sea_lvl_kg_p_m3/self.environ.air_density_max_alt_kg_p_m3)*math.sqrt(self.over_torque_factor)
      omega_max_rad_s = 2.0*math.pi*rpm_max_rpm/60.0
      power_max_kw = (torque_max_nm*omega_max_rad_s)/1000.0

      # Empirical mass model
      return 1.15*((power_max_kw/12.67) + (torque_max_nm/52.2) + 2.55)
  
  # estimates rotor solidity from thrust coefficient at hover
  # based on MTOW, air density, rotor geometry, and tip Mach hover RPM
  def _calc_rotor_solidity(self):
    if self.propulsion is None or self.environ is None:
      return None
    else:
      # Hover RPM at sea level 
      rpm_hover_rpm = (self.environ.sound_speed_m_p_s*self.propulsion.tip_mach/(self.propulsion.rotor_diameter_m/2.0))*60.0/(2.0*math.pi)
      omega_hover_sl_rad_s = rpm_hover_rpm*math.pi/30.0 # Convert to rad/s

      # Rotor thrust coefficient at hover 
      ct_hover = (
        (self.max_takeoff_mass_kg*self.environ.g_m_p_s2/self.propulsion.rotor_count)
        /(self.environ.air_density_sea_lvl_kg_p_m3
          *math.pi*(self.propulsion.rotor_diameter_m/2.0)**4
          *(omega_hover_sl_rad_s**2.0))
      )
      # Rotor solidity 
      rotor_solidity = ct_hover*6.0/self.propulsion.rotor_avg_cl
      return rotor_solidity
  
# ----- Depart Taxi (Segment A) -----
  # requires mission depart_taxi_avg_h_m_p_s, depart_taxi_s
  # horizontal power component only, assumes drag effects are negligible
  # initial horizontal velocity = 0, accelerates to final velocity
  # average velocity provided → used to find displacement, acceleration, and final velocity
  # then use MTOM, acceleration, and average velocity to find average shaft power
  # return None if mission object not populated
  def _calc_depart_taxi_avg_shaft_power_kw(self):
    if self.mission != None:
      d_h_m = self.mission.depart_taxi_avg_h_m_p_s*self.mission.depart_taxi_s
      vf_h_m_p_s = (2.0*d_h_m)/self.mission.depart_taxi_s
      a_h_m_p_s2 = vf_h_m_p_s**2.0/(2.0*d_h_m)
      return \
       (self.max_takeoff_mass_kg*a_h_m_p_s2*\
        self.mission.depart_taxi_avg_h_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft depart_taxi_avg_shaft_power_kw
  # requires power BSFC to get m_f = BSFC * P_shaft_avg * time and this same formula applies at each segment
  # return None if aircraft field or power object not populated
  def _calc_depart_taxi_fuel_consumption(self):
    if self.depart_taxi_avg_shaft_power_kw != None and self.power != None:
      return self.depart_taxi_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.depart_taxi_s
    else:
      return None


  # for the simulation inputs
  # return None if aircraft field or power object not populated
  def _calc_depart_taxi_kg_per_s(self):
    if self.depart_taxi_fuel_consumption != None and self.mission != None:
      return (self.depart_taxi_avg_shaft_power_kw*self.power.BSFC_general_kWs)
    else:
      return None

# ----- Hover Climb (Segment B) -----
  # requires mission hover_climb_avg_v_m_p_s, hover_climb_s
  # vertical power component only, assumes drag effects are negligible
  # initial vertical velocity = 0, accelerates to final velocity based on average climb rate
  # average velocity provided → used to find displacement, acceleration, and final velocity
  # includes both the induced hover power (to balance weight) and the additional power 
  # required for vertical acceleration during climb
  # return None if mission or propulsion object not populated
  def _calc_hover_climb_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None:
        
        # vertical kinematics (upward positive)
        d_v_m = self.mission.hover_climb_avg_v_m_p_s*self.mission.hover_climb_s
        vf_v_m_p_s = (2.0*d_v_m)/self.mission.hover_climb_s
        a_v_m_p_s2 = vf_v_m_p_s**2.0/(2.0*d_v_m)
    
        # induced velocity in hover (prop thrust momentum theory)
        v_i_hover = math.sqrt((self.max_takeoff_mass_kg*self.environ.g_m_p_s2)/\
                              (2.0*self.environ.air_density_sea_lvl_kg_p_m3*self.propulsion.disk_area_m2))

        # induced power (hover)
        P_hover_W = (self.max_takeoff_mass_kg*self.environ.g_m_p_s2)*v_i_hover

        return \
          (P_hover_W+self.max_takeoff_mass_kg*a_v_m_p_s2*\
            self.mission.hover_climb_avg_v_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
        return None

  # requires aircraft hover_climb_avg_shaft_power_kw
  # same method as previous segment
    # requires mission hover_climb_s and BSFC
  # return None if aircraft field or power object not populated
  def _calc_hover_climb_fuel_consumption(self):
    if self.hover_climb_avg_shaft_power_kw != None and self.power != None:
      return self.hover_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.hover_climb_s
    else:
      return None


  # for simulation
  # return None if aircraft field or power object not populated
  def _calc_hover_climb_kg_per_s(self):
    if self.hover_climb_fuel_consumption != None and self.mission != None:
      return self.hover_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Transition Climb (Segment C) -----
  # requires mission trans_climb_avg_h_m_p_s, trans_climb_v_m_p_s, trans_climb_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, hover-induced power, and climb forces
  # horizontal velocity: initial = 0, accelerates to final velocity
  # average horizontal velocity provided → used to find displacement, acceleration, and final velocity
  # vertical velocity: constant throughout (no vertical acceleration)
  # return None if mission, propulsion, or environment object not populated
  def _calc_trans_climb_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.trans_climb_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.trans_climb_v_m_p_s, self.mission.trans_climb_avg_h_m_p_s)

      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # horizontal acceleration
      v0_h_m_p_s = 0.0
      vf_h_m_p_s = 2.0*self.mission.trans_climb_avg_h_m_p_s
      d_h_m = self.mission.trans_climb_avg_h_m_p_s*self.mission.trans_climb_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0-v0_h_m_p_s**2.0)/(2.0*d_h_m)
        
      # vertical component (constant velocity, no acceleration)
      a_v_m_p_s2 = 0.0

      # force components 
      force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2
      force_v_n = self.max_takeoff_mass_kg*a_v_m_p_s2

      # induced velocity & power based on thrust deficit
      T_req_n = max(0.0, (weight_n - lift_n) + self.max_takeoff_mass_kg*a_v_m_p_s2)
      if T_req_n > 0.0:
        v_i_hover = math.sqrt(T_req_n/(2.0*self.environ.air_density_sea_lvl_kg_p_m3*self.propulsion.disk_area_m2))
      else:
        v_i_hover = 0.0

      # hover-induced power for unsupported weight only (no efficiency here yet)
      P_hover_W = T_req_n*v_i_hover

      # total shaft power (apply rotor efficiency once)
      return (P_hover_W+force_h_n*self.mission.trans_climb_avg_h_m_p_s+\
              force_v_n*self.mission.trans_climb_v_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None
    
  # requires aircraft trans_climb_avg_shaft_power_kw
  # need BSFC and time
  # return None if aircraft field or power object not populated
  def _calc_trans_climb_fuel_consumption(self):
    if self.trans_climb_avg_shaft_power_kw != None and self.power != None:
      return self.trans_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.trans_climb_s
    else:
      return None

  # for sim
  # return None if aircraft field or power object not populated
  def _calc_trans_climb_kg_per_s(self):
    if self.trans_climb_avg_shaft_power_kw != None and self.mission != None:
      return self.trans_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Depart Procedures (Segment D) -----
  # requires mission depart_proc_h_m_p_s, depart_proc_s
  # horizontal power component only, assumes constant velocity
  # vertical motion neglected (lift = weight)
  # includes aerodynamic lift, induced drag, parasite drag, and horizontal drag
  # return None if mission, propulsion, or environment object not populated
  def _calc_depart_proc_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.depart_proc_h_m_p_s**2.0
      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      lift_n = weight_n
      
      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # force components 
      force_h_n = total_drag_n

      return (force_h_n*self.mission.depart_proc_h_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft depart_proc_avg_shaft_power_kw also BSFC and time
  # return None if aircraft field or power object not populated
  def _calc_depart_proc_fuel_consumption(self):
    if self.depart_proc_avg_shaft_power_kw != None and self.power != None:
      return self.depart_proc_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.depart_proc_s
    else:
      return None

  # simulation
  # return None if aircraft field or power object not populated
  def _calc_depart_proc_kg_per_s(self):
    if self.depart_proc_avg_shaft_power_kw != None and self.mission != None:
      return self.depart_proc_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Accelerate Climb (Segment E) -----
  # requires mission accel_climb_avg_h_m_p_s, accel_climb_v_m_p_s, accel_climb_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, horizontal and vertical accelerations
  # horizontal velocity: initial = depart_proc_h_m_p_s, average velocity provided → used to compute final velocity
  # vertical velocity: initial = 0, accelerates to accel_climb_v_m_p_s
  # return None if mission, propulsion, or environment object not populated
  def _calc_accel_climb_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.accel_climb_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.accel_climb_v_m_p_s, self.mission.accel_climb_avg_h_m_p_s)

      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # horizontal accelerations
      v0_h_m_p_s = self.mission.depart_proc_h_m_p_s
      vf_h_m_p_s = 2.0*self.mission.accel_climb_avg_h_m_p_s-v0_h_m_p_s
      d_h_m = self.mission.accel_climb_avg_h_m_p_s*self.mission.accel_climb_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0-v0_h_m_p_s**2.0)/(2.0*d_h_m)

      # vertical accelerations
      v0_v_m_p_s = 0.0
      vf_v_m_p_s = self.mission.accel_climb_v_m_p_s
      d_v_m = 0.5*(v0_v_m_p_s+vf_v_m_p_s)*self.mission.accel_climb_s
      a_v_m_p_s2 = (vf_v_m_p_s**2.0-v0_v_m_p_s**2.0)/(2.0*d_v_m)

      # force components
      force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2
      force_v_n = (weight_n-lift_n)+self.max_takeoff_mass_kg*a_v_m_p_s2

      return (force_h_n*self.mission.accel_climb_avg_h_m_p_s+force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft accel_climb_avg_shaft_power_kw
  # return None if aircraft field or power object not populated
  def _calc_accel_climb_fuel_consumption(self):
    if self.accel_climb_avg_shaft_power_kw != None and self.power != None:
      return self.accel_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.accel_climb_s
    else:
      return None

  # simulation input
  # return None if aircraft field or power object not populated
  def _calc_accel_climb_kg_per_s(self):
    if self.accel_climb_avg_shaft_power_kw != None and self.mission != None:
      return self.accel_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

#begin modifications from original code
# ----- Cruise Initial (Segment F) -----
  # requires mission cruise_h_m_p_s, cruise_initial_s
  # horizontal power component only, assumes constant velocity
  # vertical motion neglected (lift = weight)
  # includes aerodynamic lift, induced drag, parasite drag, and horizontal drag
  # return None if mission, propulsion, or environment object not populated
  def _calc_cruise_initial_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_max_alt_kg_p_m3*self.mission.cruise_h_m_p_s**2.0
      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      lift_n = weight_n
      
      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      if self.wing_airfoil_cd_at_cruise_cl != None and self.stopped_rotor_cd0 != None:
        cd0_cruise = cd0+self.wing_airfoil_cd_at_cruise_cl+self.stopped_rotor_cd0
      else:
        cd0_cruise = cd0
      dp_n = q*self.wing_area_m2*cd0_cruise
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      return (total_drag_n*self.mission.cruise_h_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft cruise_shaft_power_kw
  # return None if aircraft field or power object not populated
  def _calc_cruise_initial_fuel_consumption(self):
    if self.cruise_initial_avg_shaft_power_kw != None and self.power != None:
      return self.cruise_initial_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.cruise_initial_s
    else:
      return None\
      
  # return None if aircraft field or power object not populated
  def _calc_cruise_initial_kg_per_s(self):
    if self.cruise_initial_avg_shaft_power_kw != None and self.mission != None:
      return self.cruise_initial_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

#new segments
# ----- Decelerate Descend to Drop Point (Segment F-A) -----
  # requires mission decel_descend_drop_point_avg_h_m_p_s, decel_descend_drop_point_v_m_p_s, decel_descend_drop_point_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, horizontal deceleration, vertical thrust assist if gravity is insufficient, and spoiler drag if power is negative
  # horizontal velocity: initial = cruise_h_m_p_s, average velocity provided → used to compute final velocity
  # note for continuity need the average to be the average of the hmps of the segement before and after
  # vertical velocity: initial = 0, accelerates to decel_descend_drop_point_v_m_p_s (downwards)
  # provide vertical thrust assist and spoiler drag (if needed)
  # return None if mission, propulsion, or environment object not populated
  def _calc_decel_descend_drop_point_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:     
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.decel_descend_drop_point_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.decel_descend_drop_point_v_m_p_s, self.mission.decel_descend_drop_point_avg_h_m_p_s)

      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      #this weight is correct because we have not dropped the payload yet
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # horizontal accelerations
      v0_h_m_p_s = self.mission.cruise_h_m_p_s
      vf_h_m_p_s = 2.0*self.mission.decel_descend_drop_point_avg_h_m_p_s-v0_h_m_p_s
      d_h_m = self.mission.decel_descend_drop_point_avg_h_m_p_s*self.mission.decel_descend_drop_point_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0-v0_h_m_p_s**2.0)/(2.0*d_h_m) 

      # vertical accelerations
      v0_v_m_p_s = 0.0
      vf_v_m_p_s = self.mission.decel_descend_drop_point_v_m_p_s
      d_v_m = 0.5*(v0_v_m_p_s+vf_v_m_p_s)*self.mission.decel_descend_drop_point_s
      a_v_m_p_s2 = (vf_v_m_p_s**2.0-v0_v_m_p_s**2.0)/(2.0*d_v_m)

      # force components
      force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2
      force_v_n = (weight_n-lift_n)-self.max_takeoff_mass_kg*a_v_m_p_s2 # physical: downward, speeding up

      # compute shaft power baseline
      shaft_power_kw = (force_h_n*self.mission.decel_descend_drop_point_avg_h_m_p_s+force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)

      # check vertical deficit: if gravity cannot provide enough, add vertical thrust assist shaft power
      vertical_deficit_n = self.max_takeoff_mass_kg*a_v_m_p_s2-(weight_n-lift_n)
      shaft_power_deficit_kw = 0.0
      if vertical_deficit_n > 0.0:
        shaft_power_deficit_kw = (vertical_deficit_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)

      # total shaft power (baseline + vertical assist)
      shaft_power_kw += shaft_power_deficit_kw

      # check for negative power to add spoiler drag
      if shaft_power_kw < 0.0:
        # required additional horizontal force to neutralize negative power
        required_extra_force_n = -force_h_n
        # compute equivalent delta Cd
        delta_cd_spoiler = required_extra_force_n/(q*self.wing_area_m2)
        if delta_cd_spoiler < 0.0:
          delta_cd_spoiler = 0.0
        # recompute with spoilers
        dp_spoiler_n = q*self.wing_area_m2*delta_cd_spoiler
        total_drag_n = (di_n+dp_n+dp_spoiler_n)*self.trim_drag_factor*self.excres_protub_factor
        force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2

        # total shaft power (with spoiler drag and vertical assist)
        shaft_power_kw = (force_h_n*self.mission.decel_descend_drop_point_avg_h_m_p_s+force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW) + shaft_power_deficit_kw

      return shaft_power_kw
    else:
      return None

  # requires aircraft decel_descend_drop_point_avg_shaft_power_kw
  #
  # return None if aircraft field or power object not populated
  def _calc_decel_descend_drop_point_fuel_consumption(self):
    if self.decel_descend_drop_point_avg_shaft_power_kw != None and self.power != None:
      return self.decel_descend_drop_point_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.decel_descend_drop_point_s
    else:
      return None


  # return None if aircraft field or power object not populated
  def _calc_decel_descend_drop_point_kg_per_s(self):
    if self.decel_descend_drop_point_avg_shaft_power_kw != None and self.mission != None:
      return self.decel_descend_drop_point_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None


# ----- Drop Payload (Segment F-B) -----
# requires mission drop_payload_h_m_p_s, drop_payload_s
#used the same code as the depart procedures because moving slow at a low altitude for a short time
  # horizontal power component only, assumes constant velocity
  # vertical motion neglected (lift = weight)
  # includes aerodynamic lift, induced drag, parasite drag, and horizontal drag
  # return None if mission, propulsion, or environment object not populated
  def _calc_drop_payload_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.drop_payload_h_m_p_s**2.0
      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      #will assume the payload is dropped at the end of this segment so the weight is correct
      lift_n = weight_n
      
      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # force components 
      force_h_n = total_drag_n

      return (force_h_n*self.mission.drop_payload_h_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft drop_payload_avg_shaft_power_kw
  #
  # return None if aircraft field or power object not populated
  def _calc_drop_payload_fuel_consumption(self):
    if self.drop_payload_avg_shaft_power_kw != None and self.power != None:
      return self.drop_payload_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.drop_payload_s
    else:
      return None
# sim
  # return None if aircraft field or power object not populated
  def _calc_drop_payload_kg_per_s(self):
    if self.drop_payload_avg_shaft_power_kw != None and self.mission != None:
      return self.drop_payload_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None




# ----- Accelerate Climb from Drop Point (Segment F-C) -----
# requires mission accel_climb_drop_point_avg_h_m_p_s, accel_climb_drop_point_v_m_p_s, accel_climb_drop_point_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, horizontal and vertical accelerations
  # horizontal velocity: initial = drop_payload_h_m_p_s, average velocity provided → used to compute final velocity
  # vertical velocity: initial = 0, accelerates to accel_climb_v_m_p_s
  # return None if mission, propulsion, or environment object not populated
  def _calc_accel_climb_drop_point_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.accel_climb_drop_point_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.accel_climb_drop_point_v_m_p_s, self.mission.accel_climb_drop_point_avg_h_m_p_s)

      new_mass = self.max_takeoff_mass_kg-self._payload_kg
      weight_n = new_mass*self.environ.g_m_p_s2
      # we have dropped the supressant payload
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # horizontal accelerations
      v0_h_m_p_s = self.mission.drop_payload_h_m_p_s
      vf_h_m_p_s = 2.0*self.mission.accel_climb_drop_point_avg_h_m_p_s-v0_h_m_p_s
      d_h_m = self.mission.accel_climb_drop_point_avg_h_m_p_s*self.mission.accel_climb_drop_point_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0-v0_h_m_p_s**2.0)/(2.0*d_h_m)

      # vertical accelerations
      v0_v_m_p_s = 0.0
      vf_v_m_p_s = self.mission.accel_climb_drop_point_v_m_p_s
      d_v_m = 0.5*(v0_v_m_p_s+vf_v_m_p_s)*self.mission.accel_climb_drop_point_s
      a_v_m_p_s2 = (vf_v_m_p_s**2.0-v0_v_m_p_s**2.0)/(2.0*d_v_m)

      # force components
      #changed all the max takeoff mass to the new mass where the supressant payload is dropped
      force_h_n = total_drag_n+new_mass*a_h_m_p_s2
      force_v_n = (weight_n-lift_n)+new_mass*a_v_m_p_s2

      return (force_h_n*self.mission.accel_climb_drop_point_avg_h_m_p_s+force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft accel_climb_drop_point_avg_shaft_power_kw
# return None if aircraft field or power object not populated
  def _calc_accel_climb_drop_point_fuel_consumption(self):
    if self.accel_climb_drop_point_avg_shaft_power_kw != None and self.power != None:
      return self.accel_climb_drop_point_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.accel_climb_drop_point_s
    else:
      return None

  # sim
  # return None if aircraft field or power object not populated
  def _calc_accel_climb_drop_point_kg_per_s(self):
    if self.accel_climb_drop_point_avg_shaft_power_kw != None and self.mission != None:
      return self.accel_climb_drop_point_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Cruise Return (Segment F-D) -----
# requires mission cruise_h_m_p_s, cruise_return_s
  # horizontal power component only, assumes constant velocity
  # vertical motion neglected (lift = weight)
  # includes aerodynamic lift, induced drag, parasite drag, and horizontal drag
  # return None if mission, propulsion, or environment object not populated
  def _calc_cruise_return_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_max_alt_kg_p_m3*self.mission.cruise_h_m_p_s**2.0
      new_mass = self.max_takeoff_mass_kg-self._payload_kg
      weight_n = new_mass*self.environ.g_m_p_s2
      lift_n = weight_n
      
      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      if self.wing_airfoil_cd_at_cruise_cl != None and self.stopped_rotor_cd0 != None:
        cd0_cruise = cd0+self.wing_airfoil_cd_at_cruise_cl+self.stopped_rotor_cd0
      else:
        cd0_cruise = cd0
      dp_n = q*self.wing_area_m2*cd0_cruise
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      return (total_drag_n*self.mission.cruise_h_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft cruise_shaft_power_kw
# return None if aircraft field or power object not populated
  def _calc_cruise_return_fuel_consumption(self):
    if self.cruise_return_avg_shaft_power_kw != None and self.power != None:
      return self.cruise_return_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.cruise_return_s
    else:
      return None
    
  # sim
  # return None if aircraft field or power object not populated
  def _calc_cruise_return_kg_per_s(self):
    if self.cruise_return_avg_shaft_power_kw != None and self.mission != None:
      return self.cruise_return_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Decelerate Descend (Segment G) -----
  # requires mission decel_descend_avg_h_m_p_s, decel_descend_v_m_p_s, decel_descend_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, horizontal deceleration, vertical thrust assist if gravity is insufficient, and spoiler drag if power is negative
  # horizontal velocity: initial = cruise_h_m_p_s, average velocity provided → used to compute final velocity
  # vertical velocity: initial = 0, accelerates to decel_descend_v_m_p_s (downwards)
  # provide vertical thrust assist and spoiler drag (if needed)
  # return None if mission, propulsion, or environment object not populated
  def _calc_decel_descend_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:     
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.decel_descend_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.decel_descend_v_m_p_s, self.mission.decel_descend_avg_h_m_p_s)

      new_mass = self.max_takeoff_mass_kg-self._payload_kg
      weight_n = new_mass*self.environ.g_m_p_s2
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # horizontal accelerations
      v0_h_m_p_s = self.mission.cruise_h_m_p_s
      vf_h_m_p_s = 2.0*self.mission.decel_descend_avg_h_m_p_s-v0_h_m_p_s
      d_h_m = self.mission.decel_descend_avg_h_m_p_s*self.mission.decel_descend_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0-v0_h_m_p_s**2.0)/(2.0*d_h_m) 

      # vertical accelerations
      v0_v_m_p_s = 0.0
      vf_v_m_p_s = self.mission.decel_descend_v_m_p_s
      d_v_m = 0.5*(v0_v_m_p_s+vf_v_m_p_s)*self.mission.decel_descend_s
      a_v_m_p_s2 = (vf_v_m_p_s**2.0-v0_v_m_p_s**2.0)/(2.0*d_v_m)

      # force components
      force_h_n = total_drag_n+new_mass*a_h_m_p_s2
      force_v_n = (weight_n-lift_n)-new_mass*a_v_m_p_s2 # physical: downward, speeding up

      # compute shaft power baseline
      shaft_power_kw = (force_h_n*self.mission.decel_descend_avg_h_m_p_s+force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)

      # check vertical deficit: if gravity cannot provide enough, add vertical thrust assist shaft power
      vertical_deficit_n = new_mass*a_v_m_p_s2-(weight_n-lift_n)
      shaft_power_deficit_kw = 0.0
      if vertical_deficit_n > 0.0:
        shaft_power_deficit_kw = (vertical_deficit_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)

      # total shaft power (baseline + vertical assist)
      shaft_power_kw += shaft_power_deficit_kw

      # check for negative power to add spoiler drag
      if shaft_power_kw < 0.0:
        # required additional horizontal force to neutralize negative power
        required_extra_force_n = -force_h_n
        # compute equivalent delta Cd
        delta_cd_spoiler = required_extra_force_n/(q*self.wing_area_m2)
        if delta_cd_spoiler < 0.0:
          delta_cd_spoiler = 0.0
        # recompute with spoilers
        dp_spoiler_n = q*self.wing_area_m2*delta_cd_spoiler
        total_drag_n = (di_n+dp_n+dp_spoiler_n)*self.trim_drag_factor*self.excres_protub_factor
        force_h_n = total_drag_n+new_mass*a_h_m_p_s2

        # total shaft power (with spoiler drag and vertical assist)
        shaft_power_kw = (force_h_n*self.mission.decel_descend_avg_h_m_p_s+force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW) + shaft_power_deficit_kw

      return shaft_power_kw
    else:
      return None

  # requires aircraft decel_descend_avg_shaft_power_kw
  # 
  # return None if aircraft field or power object not populated
  def _calc_decel_descend_fuel_consumption(self):
    if self.decel_descend_avg_shaft_power_kw != None and self.power != None:
      return self.decel_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.decel_descend_s
    else:
      return None

  # sim
  # return None if aircraft field or power object not populated
  def _calc_decel_descend_kg_per_s(self):
    if self.decel_descend_avg_shaft_power_kw != None and self.mission != None:
      return self.decel_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Arrive Procedures (Segment H) -----
  # requires mission arrive_proc_h_m_p_s, arrive_proc_s
  # horizontal power component only, assumes constant velocity
  # vertical motion neglected (lift = weight)
  # includes aerodynamic lift, induced drag, parasite drag, and horizontal drag
  # return None if mission, propulsion, or environment object not populated
  def _calc_arrive_proc_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.arrive_proc_h_m_p_s**2.0
      new_mass = self.max_takeoff_mass_kg-self._payload_kg
      weight_n = new_mass*self.environ.g_m_p_s2
      # horizontal component
      lift_n = weight_n
      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # force components
      force_h_n = total_drag_n

      return (force_h_n*self.mission.arrive_proc_h_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft arrive_proc_avg_shaft_power_kw
  # im about to claw my eyes out
  # return None if aircraft field or power object not populated
  def _calc_arrive_proc_fuel_consumption(self):
    if self.arrive_proc_avg_shaft_power_kw != None and self.power != None:
      return self.arrive_proc_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.arrive_proc_s
    else:
      return None

  # sim (shocker!)
  # return None if aircraft field or power object not populated
  def _calc_arrive_proc_kg_per_s(self):
    if self.arrive_proc_avg_shaft_power_kw != None and self.mission != None:
      return self.arrive_proc_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Transition Descend (Segment I) -----  
  # requires mission trans_descend_avg_h_m_p_s, trans_descend_v_m_p_s, trans_descend_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, descent forces,
  # hover-induced thrust assist if gravity is insufficient, and spoiler drag if power is negative
  # horizontal velocity: initial estimated from average, final = 0 (vehicle decelerates to stop)
  # vertical velocity: initial = decel_descend_v_m_p_s, final = trans_descend_v_m_p_s
  # return None if mission, propulsion, or environment object not populated
  def _calc_trans_descend_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.trans_descend_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.trans_descend_v_m_p_s, self.mission.trans_descend_avg_h_m_p_s)

      new_mass = self.max_takeoff_mass_kg-self._payload_kg
      weight_n = new_mass*self.environ.g_m_p_s2
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # horizontal acceleration (vehicle decelerates to stop)
      v0_h_m_p_s = 2.0*self.mission.trans_descend_avg_h_m_p_s
      vf_h_m_p_s = 0.0
      d_h_m = self.mission.trans_descend_avg_h_m_p_s*self.mission.trans_descend_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0 - v0_h_m_p_s**2.0)/(2.0*d_h_m)

      # vertical acceleration
      v0_v_m_p_s = self.mission.decel_descend_v_m_p_s
      vf_v_m_p_s = self.mission.trans_descend_v_m_p_s
      d_v_m = 0.5*(abs(v0_v_m_p_s)+abs(vf_v_m_p_s))*self.mission.trans_descend_s
      a_v_m_p_s2 = (vf_v_m_p_s**2.0 - v0_v_m_p_s**2.0)/(2.0*d_v_m)

      # force components
      force_h_n = total_drag_n+new_mass*a_h_m_p_s2
      force_v_n = new_mass*a_v_m_p_s2

      # compute thrust deficit if gravity + lift are insufficient
      T_req_n = max(0.0, (weight_n - lift_n) + new_mass*a_v_m_p_s2)
      if T_req_n > 0.0:
        v_i_hover = math.sqrt(T_req_n/(2.0*self.environ.air_density_sea_lvl_kg_p_m3*self.propulsion.disk_area_m2))
      else:
        v_i_hover = 0.0

      # hover-induced (assist) power for unsupported weight only
      P_hover_W = T_req_n*v_i_hover

      # baseline shaft power (sum of aerodynamic and vertical terms)
      shaft_power_kw = (P_hover_W+force_h_n*self.mission.trans_descend_avg_h_m_p_s+\
                        force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)

      # check for negative power → apply spoiler drag to dissipate excess
      if shaft_power_kw < 0.0:
        required_extra_force_n = -force_h_n
        delta_cd_spoiler = required_extra_force_n/(q*self.wing_area_m2)
        if delta_cd_spoiler < 0.0:
          delta_cd_spoiler = 0.0
        dp_spoiler_n = q*self.wing_area_m2*delta_cd_spoiler
        total_drag_n = (di_n+dp_n+dp_spoiler_n)*self.trim_drag_factor*self.excres_protub_factor
        force_h_n = total_drag_n+new_mass*a_h_m_p_s2

        # recompute total shaft power with spoiler drag
        shaft_power_kw = (P_hover_W+force_h_n*self.mission.trans_descend_avg_h_m_p_s+\
                          force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)

      return shaft_power_kw
    else:
      return None

  # requires aircraft trans_descend_avg_shaft_power_kw
  # i stg if i get another error from deleting COMMENTS im gonna break something
  # return None if aircraft field or power object not populated
  def _calc_trans_descend_fuel_consumption(self):
    if self.trans_descend_avg_shaft_power_kw != None and self.power != None:
      return self.trans_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.trans_descend_s
    else:
      return None

  # sim
  # return None if aircraft field or power object not populated
  def _calc_trans_descend_kg_per_s(self):
    if self.trans_descend_avg_shaft_power_kw != None and self.mission != None:
      return self.trans_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Hover Descend (Segment J) -----
  # requires mission hover_descend_avg_v_m_p_s, hover_descend_s
  # vertical power component only, assumes drag effects are negligible
  # initial vertical velocity = 2*avg (downward), final = 0.0
  # upward positive convention → acceleration is negative
  # compute induced power from actual thrust
  # return None if mission, propulsion, or environment object not populated
  def _calc_hover_descend_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
        new_mass = self.max_takeoff_mass_kg-self._payload_kg
        # vertical kinematics (upward positive)
        v0_v_m_p_s = 2.0*self.mission.hover_descend_avg_v_m_p_s
        vf_v_m_p_s = 0.0
        d_v_m = self.mission.hover_descend_avg_v_m_p_s*self.mission.hover_descend_s
        a_v_m_p_s2 = (vf_v_m_p_s**2.0 - v0_v_m_p_s**2.0) / (2.0*d_v_m)

        # vertical thrust required (upward positive)
        force_v_n =  (new_mass*a_v_m_p_s2)

        # induced velocity in hover (prop thrust momentum theory)
        v_i_hover = math.sqrt((new_mass*self.environ.g_m_p_s2)/\
                              (2.0*self.environ.air_density_sea_lvl_kg_p_m3*self.propulsion.disk_area_m2))

        # induced hover power
        P_hover_W = (new_mass*self.environ.g_m_p_s2)*v_i_hover

        # total shaft power (hover & vertical component)
        return \
          (P_hover_W + force_v_n * self.mission.hover_descend_avg_v_m_p_s) / \
            (self.propulsion.rotor_effic * W_P_KW)
    else:
        return None

  # requires aircraft hover_descend_avg_shaft_power_kw
  # AND THERE IT IS ANOTHER ERROR MESSAGE FOR DELETING COMMENTS
  # return None if aircraft field or power object not populated
  def _calc_hover_descend_fuel_consumption(self):
    if self.hover_descend_avg_shaft_power_kw != None and self.power != None:
      return self.hover_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.hover_descend_s
    else:
      return None

  # sim urghhhhhhhhhhhhh
  # return None if aircraft field or power object not populated
  def _calc_hover_descend_kg_per_s(self):
    if self.hover_descend_avg_shaft_power_kw != None and self.mission != None:
      return self.hover_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Arrive Taxi (Segment K) -----
  # requires mission arrive_taxi_avg_h_m_p_s, arrive_taxi_s
  # horizontal motion only: initial velocity = 0.0, final = 2*avg
  # includes horizontal acceleration effects (drag neglected)
  # return None if mission, propulsion, or environment object not populated
  def _calc_arrive_taxi_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      # horizontal accelerations
      v0_h_m_p_s = 0.0
      vf_h_m_p_s = 2.0*self.mission.arrive_taxi_avg_h_m_p_s
      d_h_m = self.mission.arrive_taxi_avg_h_m_p_s*self.mission.arrive_taxi_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0-v0_h_m_p_s**2.0)/(2.0*d_h_m)
      
      # horizontal force 
      new_mass = self.max_takeoff_mass_kg-self._payload_kg
      force_h_n = new_mass*a_h_m_p_s2

      return (force_h_n*self.mission.arrive_taxi_avg_h_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft arrive_taxi_avg_shaft_power_kw
  # screw the epu
  # return None if aircraft field or power object not populated
  def _calc_arrive_taxi_fuel_consumption(self):
    if self.arrive_taxi_avg_shaft_power_kw != None and self.power != None:
      return self.arrive_taxi_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.arrive_taxi_s
    else:
      return None

  # sim and im losing it
  # return None if aircraft field or power object not populated
  def _calc_arrive_taxi_kg_per_s(self):
    if self.arrive_taxi_avg_shaft_power_kw != None and self.mission != None:
      return self.arrive_taxi_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None
    
  # ----- Reserve Hover Climb (Segment B') -----
  # requires mission reserve_hover_climb_avg_v_m_p_s, reserve_hover_climb_s
  # vertical power component only, assumes drag effects are negligible
  # initial vertical velocity = 0, accelerates to final velocity based on average climb rate
  # average velocity provided → used to find displacement, acceleration, and final velocity
  # includes both the induced hover power (to balance weight) and the additional power 
  # required for vertical acceleration during reserve hover climb
  # return None if mission or propulsion object not populated
  def _calc_reserve_hover_climb_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None:
        
        # vertical kinematics (upward positive)
        d_v_m = self.mission.reserve_hover_climb_avg_v_m_p_s*self.mission.reserve_hover_climb_s
        vf_v_m_p_s = (2.0*d_v_m)/self.mission.reserve_hover_climb_s
        a_v_m_p_s2 = vf_v_m_p_s**2.0/(2.0*d_v_m)
        
        # induced velocity in hover (prop thrust momentum theory)
        v_i_hover = math.sqrt((self.max_takeoff_mass_kg*self.environ.g_m_p_s2)/\
                              (2.0*self.environ.air_density_sea_lvl_kg_p_m3*self.propulsion.disk_area_m2))
        
        # induced power (hover)
        P_hover_W = (self.max_takeoff_mass_kg*self.environ.g_m_p_s2)*v_i_hover
        
        return \
          (P_hover_W+self.max_takeoff_mass_kg*a_v_m_p_s2*\
            self.mission.reserve_hover_climb_avg_v_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
        return None

  # requires aircraft reserve_hover_climb_avg_shaft_power_kw
  # when this breaks im gonna cry
  # return None if aircraft field or power object not populated
  def _calc_reserve_hover_climb_fuel_consumption(self):
    if self.reserve_hover_climb_avg_shaft_power_kw != None and self.power != None:
      return self.reserve_hover_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.reserve_hover_climb_s
    else:
      return None
      
  # sim (not gonna cry as much as i did for my car)
  # return None if aircraft field or power object not populated
  def _calc_reserve_hover_climb_kg_per_s(self):
    if self.reserve_hover_climb_avg_shaft_power_kw!= None and self.mission != None:
      return self.reserve_hover_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Reserve Transition Climb (Segment C') -----
  # requires mission reserve_trans_climb_avg_h_m_p_s, reserve_trans_climb_v_m_p_s, reserve_trans_climb_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, hover-induced power, and climb forces
  # horizontal velocity: initial = 0, average horizontal velocity provided → used to find displacement and final velocity
  # vertical velocity: constant throughout the segment (no vertical acceleration)
  # return None if mission, propulsion, or environment object not populated
  def _calc_reserve_trans_climb_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.reserve_trans_climb_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.reserve_trans_climb_v_m_p_s, self.mission.reserve_trans_climb_avg_h_m_p_s)

      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # horizontal acceleration
      v0_h_m_p_s = 0.0
      vf_h_m_p_s = 2.0*self.mission.reserve_trans_climb_avg_h_m_p_s
      d_h_m = self.mission.reserve_trans_climb_avg_h_m_p_s*self.mission.reserve_trans_climb_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0 - v0_h_m_p_s**2.0)/(2.0*d_h_m)

      # vertical component (constant velocity, no acceleration)
      a_v_m_p_s2 = 0.0

      # force components
      force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2
      force_v_n = self.max_takeoff_mass_kg*a_v_m_p_s2

      # induced velocity & power based on thrust deficit
      T_req_n = max(0.0, (weight_n - lift_n) + self.max_takeoff_mass_kg*a_v_m_p_s2)
      if T_req_n > 0.0:
        v_i_hover = math.sqrt(T_req_n/(2.0*self.environ.air_density_sea_lvl_kg_p_m3*self.propulsion.disk_area_m2))
      else:
        v_i_hover = 0.0

      # hover-induced power for unsupported weight only (no efficiency here yet)
      P_hover_W = T_req_n*v_i_hover

      # total shaft power (apply rotor efficiency once)
      return (P_hover_W+force_h_n*self.mission.reserve_trans_climb_avg_h_m_p_s+\
              force_v_n*self.mission.reserve_trans_climb_v_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft reserve_trans_climb_avg_shaft_power_kw
  # hashtag long live the beast
  # return None if aircraft field or power object not populated
  def _calc_reserve_trans_climb_fuel_consumption(self):
    if self.reserve_trans_climb_avg_shaft_power_kw != None and self.power != None:
      return self.reserve_trans_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.reserve_trans_climb_s
    else:
      return None

  # sim
  # return None if aircraft field or power object not populated
  def _calc_reserve_trans_climb_kg_per_s(self):
    if self.reserve_trans_climb_avg_shaft_power_kw != None and self.mission != None:
      return self.reserve_trans_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs
      return None

# ----- Reserve Acceleration Climb (Segment E') -----
  # requires mission reserve_accel_climb_avg_h_m_p_s, reserve_accel_climb_v_m_p_s, reserve_accel_climb_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, and climb forces
  # horizontal velocity: initial = final of Reserve Transition Climb (2*reserve_trans_climb_avg_h_m_p_s),
  # vertical velocity: constant throughout the segment (no vertical acceleration)
  # return None if mission, propulsion, or environment object not populated
  def _calc_reserve_accel_climb_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.reserve_accel_climb_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.reserve_accel_climb_v_m_p_s, self.mission.reserve_accel_climb_avg_h_m_p_s)

      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor
      
      # horizontal accelerations
      v0_h_m_p_s = 2.0*self.mission.reserve_trans_climb_avg_h_m_p_s
      vf_h_m_p_s = 2.0*self.mission.reserve_accel_climb_avg_h_m_p_s-v0_h_m_p_s
      d_h_m = self.mission.reserve_accel_climb_avg_h_m_p_s*self.mission.reserve_accel_climb_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0 - v0_h_m_p_s**2.0)/(2.0*d_h_m)

      # vertical component (constant velocity, no acceleration)
      a_v_m_p_s2 = 0.0

      # force components
      force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2
      force_v_n = (weight_n-lift_n)+self.max_takeoff_mass_kg*a_v_m_p_s2
      return (force_h_n*self.mission.reserve_accel_climb_avg_h_m_p_s+force_v_n*self.mission.reserve_accel_climb_v_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft reserve_accel_climb_avg_shaft_power_kw
  # i think the reserve mission is useless
  # return None if aircraft field or power object not populated
  def _calc_reserve_accel_climb_fuel_consumption(self):
    if self.reserve_accel_climb_avg_shaft_power_kw != None and self.power != None:
      return self.reserve_accel_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.reserve_accel_climb_s
    else:
      return None

  # sim again
  # return None if aircraft field or power object not populated
  def _calc_reserve_accel_climb_kg_per_s(self):
    if self.reserve_accel_climb_avg_shaft_power_kw != None and self.mission != None:
      return self.reserve_accel_climb_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Reserve Cruise (Segment F') -----
  # requires mission reserve_cruise_h_m_p_s, reserve_cruise_s
  # horizontal power component only
  # includes aerodynamic lift, induced drag, parasite drag, weight, and horizontal motion
  # return None if mission, propulsion, or environment object not populated
  def _calc_reserve_cruise_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_max_alt_kg_p_m3*self.mission.reserve_cruise_h_m_p_s**2.0
      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      lift_n = weight_n
      
      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      if self.wing_airfoil_cd_at_cruise_cl != None and self.stopped_rotor_cd0 != None:
        cd0_cruise = cd0+self.wing_airfoil_cd_at_cruise_cl+self.stopped_rotor_cd0
      else:
        cd0_cruise = cd0
      dp_n = q*self.wing_area_m2*cd0_cruise
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      return (total_drag_n*self.mission.reserve_cruise_h_m_p_s)/(self.propulsion.rotor_effic*W_P_KW)
    else:
      return None

  # requires aircraft reserve_cruise_shaft_power_kw
  # poor aarav with the other code
  # return None if aircraft field or power object not populated
  def _calc_reserve_cruise_fuel_consumption(self):
    if self.reserve_cruise_avg_shaft_power_kw != None and self.power != None:
      return self.reserve_cruise_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.reserve_cruise_s
    else:
      return None
    
  # sim so brutal
  # return None if aircraft field or power object not populated
  def _calc_reserve_cruise_kg_per_s(self):
    if self.reserve_cruise_avg_shaft_power_kw != None and self.mission != None:
      return self.reserve_cruise_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Reserve Deceleration Descend (Segment G') -----
  # requires mission reserve_decel_descend_avg_h_m_p_s, reserve_decel_descend_v_m_p_s, reserve_decel_descend_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, descend forces, and vertical thrust assist if gravity is insufficient
  # provide vertical thrust assist and spoiler drag (if needed)
  # return None if mission, propulsion, or environment object not populated
  def _calc_reserve_decel_descend_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.reserve_decel_descend_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.reserve_decel_descend_v_m_p_s, self.mission.reserve_decel_descend_avg_h_m_p_s)

      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # horizontal accelerations
      v0_h_m_p_s = self.mission.reserve_cruise_h_m_p_s
      vf_h_m_p_s = 2.0*self.mission.reserve_decel_descend_avg_h_m_p_s-v0_h_m_p_s
      d_h_m = self.mission.reserve_decel_descend_avg_h_m_p_s*self.mission.reserve_decel_descend_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0-v0_h_m_p_s**2.0)/(2.0*d_h_m)

      # vertical acceleration 
      v0_v_m_p_s = 0.0
      vf_v_m_p_s = self.mission.reserve_decel_descend_v_m_p_s
      d_v_m = 0.5*(v0_v_m_p_s+vf_v_m_p_s)*self.mission.reserve_decel_descend_s
      a_v_m_p_s2 = (vf_v_m_p_s**2.0-v0_v_m_p_s**2.0)/(2.0*d_v_m)

      # force components
      force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2
      force_v_n = (weight_n-lift_n)-self.max_takeoff_mass_kg*a_v_m_p_s2 # physical: downward, speeding up

      # compute shaft power baseline
      shaft_power_kw = (force_h_n*self.mission.reserve_decel_descend_avg_h_m_p_s+force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)
      
      # check vertical deficit: if gravity cannot provide enough, add vertical thrust assist shaft power
      vertical_deficit_n = self.max_takeoff_mass_kg*a_v_m_p_s2-(weight_n-lift_n)
      shaft_power_deficit_kw = 0.0
      if vertical_deficit_n > 0.0:
        # convert deficit to power explicitly
        shaft_power_deficit_kw = (vertical_deficit_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)
      
      # total shaft power (baseline + vertical assist)
      shaft_power_kw += shaft_power_deficit_kw

      # check for negative power to add spoiler drag
      if shaft_power_kw < 0.0:
        # required additional horizontal force to neutralize negative power
        required_extra_force_n = -force_h_n
        # compute equivalent delta Cd
        delta_cd_spoiler = required_extra_force_n/(q*self.wing_area_m2)
        if delta_cd_spoiler < 0.0:
          delta_cd_spoiler = 0.0
        # recompute with spoilers
        dp_spoiler_n = q*self.wing_area_m2*delta_cd_spoiler
        total_drag_n = (di_n+dp_n+dp_spoiler_n)*self.trim_drag_factor*self.excres_protub_factor
        force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2
      
        # total shaft power
        shaft_power_kw = (force_h_n*self.mission.reserve_decel_descend_avg_h_m_p_s+force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW) + shaft_power_deficit_kw

      return shaft_power_kw
    else:
      return None

  # requires aircraft reserve_decel_descend_avg_shaft_power_kw
  # i wish the bear were in the superbowl
  # return None if aircraft field or power object not populated
  def _calc_reserve_decel_descend_fuel_consumption(self):
    if self.reserve_decel_descend_avg_shaft_power_kw != None and self.power != None:
      return self.reserve_decel_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.reserve_decel_descend_s
    else:
      return None

  # next year for them i trust (this is for sim btw)
  # return None if aircraft field or power object not populated
  def _calc_reserve_decel_descend_kg_per_s(self):
    if self.reserve_decel_descend_avg_shaft_power_kw != None and self.mission != None:
      return self.reserve_decel_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

# ----- Reserve Transition Descend (Segment I') -----
  # requires mission reserve_trans_descend_avg_h_m_p_s, reserve_trans_descend_v_m_p_s, reserve_trans_descend_s
  # includes aerodynamic lift, induced drag, parasite drag, weight, descend forces,
  # hover-induced thrust assist if gravity is insufficient, and spoiler drag if power is negative
  # horizontal velocity: initial from reserve decel segment to 0; vertical velocity changes from previous segment to final
  # return None if mission, propulsion, or environment object not populated
  def _calc_reserve_trans_descend_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:    
      q = 0.5*self.environ.air_density_sea_lvl_kg_p_m3*self.mission.reserve_trans_descend_avg_h_m_p_s**2.0
      theta = math.atan2(self.mission.reserve_trans_descend_v_m_p_s, self.mission.reserve_trans_descend_avg_h_m_p_s)

      weight_n = self.max_takeoff_mass_kg*self.environ.g_m_p_s2
      lift_n = weight_n*math.cos(theta)
      vehicle_cl = lift_n/(q*self.wing_area_m2)

      # induced drag
      di_n = (lift_n**2.0)/(q*self.wing_area_m2*math.pi*self.wing_aspect_ratio*self.span_effic_factor)
      # parasite drag
      cd0 = self._calc_total_drag_coef()
      if cd0 == None:
        return None
      dp_n = q*self.wing_area_m2*cd0
      # total drag
      total_drag_n = (di_n+dp_n)*self.trim_drag_factor*self.excres_protub_factor

      # horizontal accelerations
      v0_h_m_p_s = 2.0*self.mission.reserve_decel_descend_avg_h_m_p_s-self.mission.reserve_cruise_h_m_p_s
      vf_h_m_p_s = 0.0
      d_h_m = self.mission.reserve_trans_descend_avg_h_m_p_s*self.mission.reserve_trans_descend_s
      a_h_m_p_s2 = (vf_h_m_p_s**2.0-v0_h_m_p_s**2.0)/(2.0*d_h_m)

      # vertical acceleration 
      v0_v_m_p_s = self.mission.reserve_decel_descend_v_m_p_s
      vf_v_m_p_s = self.mission.reserve_trans_descend_v_m_p_s
      d_v_m = 0.5*(v0_v_m_p_s+vf_v_m_p_s)*self.mission.reserve_trans_descend_s
      a_v_m_p_s2 = (vf_v_m_p_s**2.0-v0_v_m_p_s**2.0)/(2.0*d_v_m)

      # force components
      force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2
      # exclude (weight - lift) here; handled via thrust-deficit induced power
      force_v_n = self.max_takeoff_mass_kg*a_v_m_p_s2

      # compute thrust deficit if gravity + lift are insufficient
      T_req_n = max(0.0, (weight_n - lift_n) + self.max_takeoff_mass_kg*a_v_m_p_s2)
      if T_req_n > 0.0:
        v_i_hover = math.sqrt(T_req_n/(2.0*self.environ.air_density_sea_lvl_kg_p_m3*self.propulsion.disk_area_m2))
      else:
        v_i_hover = 0.0

      # hover-induced (assist) power for unsupported weight only (no efficiency here yet)
      P_hover_W = T_req_n*v_i_hover

      # baseline shaft power (sum of aerodynamic and vertical terms)
      shaft_power_kw = (P_hover_W+force_h_n*self.mission.reserve_trans_descend_avg_h_m_p_s+\
                        force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)
      
      # check for negative power to add spoiler drag
      if shaft_power_kw < 0.0:
        # required additional horizontal force to neutralize negative power
        required_extra_force_n = -force_h_n
        # compute equivalent delta Cd
        delta_cd_spoiler = required_extra_force_n/(q*self.wing_area_m2)
        if delta_cd_spoiler < 0.0:
          delta_cd_spoiler = 0.0
        # recompute with spoilers
        dp_spoiler_n = q*self.wing_area_m2*delta_cd_spoiler
        total_drag_n = (di_n+dp_n+dp_spoiler_n)*self.trim_drag_factor*self.excres_protub_factor
        force_h_n = total_drag_n+self.max_takeoff_mass_kg*a_h_m_p_s2
      
        # total shaft power
        shaft_power_kw = (P_hover_W+force_h_n*self.mission.reserve_trans_descend_avg_h_m_p_s+\
                          force_v_n*(0.5*(v0_v_m_p_s+vf_v_m_p_s)))/(self.propulsion.rotor_effic*W_P_KW)

      return shaft_power_kw
    else:
      return None

  # requires aircraft reserve_trans_descend_avg_shaft_power_kw
  # odds someone actually sees these notes?
  # return None if aircraft field or power object not populated
  def _calc_reserve_trans_descend_fuel_consumption(self):
    if self.reserve_trans_descend_avg_shaft_power_kw != None and self.power != None:
      return self.reserve_trans_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission.reserve_trans_descend_s
    else:
      return None

  # simulation (odds not high but ill give someone 10 bucks for the first comment)
  # return None if aircraft field or power object not populated
  def _calc_reserve_trans_descend_kg_per_s(self):
    if self.reserve_trans_descend_avg_shaft_power_kw != None and self.mission != None:
      return self.reserve_trans_descend_avg_shaft_power_kw * self.power.BSFC_general_kWs
    else:
      return None

# ----- Reserve Hover Descend (Segment J') -----
  # requires mission reserve_hover_descend_avg_v_m_p_s, reserve_hover_descend_s
  # vertical power component only, assumes drag effects are negligible
  # initial vertical velocity = 2*avg (downward), final = 0.0
  # upward positive convention → acceleration is negative
  # compute induced power from actual thrust
  # return None if mission, propulsion, or environment object not populated
  def _calc_reserve_hover_descend_avg_shaft_power_kw(self):
    if self.mission != None and self.propulsion != None and self.environ != None:
        
        # vertical kinematics (upward positive)
        v0_v_m_p_s = 2.0*self.mission.reserve_hover_descend_avg_v_m_p_s
        vf_v_m_p_s = 0.0
        d_v_m = self.mission.reserve_hover_descend_avg_v_m_p_s*self.mission.reserve_hover_descend_s
        a_v_m_p_s2 = (vf_v_m_p_s**2.0 - v0_v_m_p_s**2.0) / (2.0*d_v_m)

        # additional power due to acceleration
        force_v_n =  (self.max_takeoff_mass_kg*a_v_m_p_s2)

        # induced velocity in hover (prop thrust momentum theory)
        v_i_hover = math.sqrt((self.max_takeoff_mass_kg*self.environ.g_m_p_s2)/\
                              (2.0*self.environ.air_density_sea_lvl_kg_p_m3*self.propulsion.disk_area_m2))

        # induced hover power
        P_hover_W = (self.max_takeoff_mass_kg*self.environ.g_m_p_s2)*v_i_hover

        # total shaft power (hover & vertical component)
        return \
          (P_hover_W + force_v_n * self.mission.reserve_hover_descend_avg_v_m_p_s) / \
            (self.propulsion.rotor_effic * W_P_KW)
    else:
        return None

  # requires aircraft reserve_hover_descend_avg_shaft_power_kw
  # sooooo close
  # return None if aircraft field or power object not populated
  def _calc_reserve_hover_descend_fuel_consumption(self):
    if self.reserve_hover_descend_avg_shaft_power_kw != None and self.power != None:
      return self.reserve_hover_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs*self.mission._reserve_hover_descend_s
    else:
      return None

  # simmmmmmm
  # return None if aircraft field or power object not populated
  def _calc_reserve_hover_descend_kg_per_s(self):
    if self.reserve_hover_descend_avg_shaft_power_kw!= None and self.mission != None:
      return self.reserve_hover_descend_avg_shaft_power_kw*self.power.BSFC_general_kWs
    else:
      return None

  # calculates total energy required for the mission
  def _calc_total_mission_fuel_consumption(self):
    segments = [
      self.depart_taxi_fuel_consumption,
      self.hover_climb_fuel_consumption,
      self.trans_climb_fuel_consumption,
      self.depart_proc_fuel_consumption,
      self.accel_climb_fuel_consumption,
      self.cruise_initial_fuel_consumption,
      self.decel_descend_fuel_consumption,
      self.arrive_proc_fuel_consumption,
      self.trans_descend_fuel_consumption,
      self.hover_descend_fuel_consumption,
      self.arrive_taxi_fuel_consumption,
    ]
    total_fuel_consumption = sum(e for e in segments if e is not None)
    if total_fuel_consumption > 0:
      return total_fuel_consumption 
    else:
      return None

  # calculates total energy required for the mission
  def _calc_total_reserve_fuel_consumption(self):
    segments = [
      self.reserve_hover_climb_fuel_consumption,
      self.reserve_trans_climb_fuel_consumption,
      self.reserve_accel_climb_fuel_consumption,
      self.reserve_cruise_fuel_consumption,
      self.reserve_decel_descend_fuel_consumption,
      self.reserve_trans_descend_fuel_consumption,
      self.reserve_hover_descend_fuel_consumption,
    ]
    total_reserve_fuel_consumption = sum(e for e in segments if e is not None)
    if total_reserve_fuel_consumption > 0:
      return total_reserve_fuel_consumption  
    else:
      return None

  #use this instead of writing within each function
  def _calc_mass_without_payload(self):
    if self.power is None:
      return None
    new_mass = self.max_takeoff_mass_kg-self._payload_kg-self._personelle_payload_kg
    if new_mass != None and self.power != None:
      return new_mass
    else: 
      return None
    
  # literally already calculated so this is redundant ig
  def _calc_fuel_mass_kg(self):
    if self.power is None:
      return None
    total_fuel_consumption = self._calc_total_mission_fuel_consumption()
    if total_fuel_consumption != None and self.power != None:
      return total_fuel_consumption
    else:
      return None 
  
  # estimates wing structural mass [kg] using NDARC AFDD93 model 
  # based on Raymer estimation and 0.9 technology factor
  def _calc_wing_mass_kg(self):
    max_takeoff_mass_lb = self.max_takeoff_mass_kg*KG_2_LB
    wing_area_ft2 = self.wing_area_m2*(M_2_FT**2)
    wing_mass_lb = (
      5.66411
      *(max_takeoff_mass_lb/1000.0)**0.847
      *(3.8*1.5)**0.39579
      *(wing_area_ft2)**0.21754
      *(self.wing_aspect_ratio)**0.50016
      *((1.0+self.wing_taper_ratio)/self.wing_t_p_c)**0.09359
      *0.9
    )
    return wing_mass_lb/KG_2_LB

  # estimates horizontal tail structural mass [kg] using NDARC model 
  # based on tail area, dive speed, and 0.9 technology factor
  def _calc_horiz_tail_mass_kg(self):
    if self.mission is None:
      return None
    horiz_tail_area_ft2 = self.horiz_tail_area_m2*(M_2_FT**2)
    dive_speed_kts = 1.4*self.mission.cruise_h_m_p_s*M_P_S_2_KTS
    horiz_tail_mass_lb = (
      horiz_tail_area_ft2
      *(0.00395*(horiz_tail_area_ft2**0.2)*dive_speed_kts-0.4885)
      *0.9
    )
    return horiz_tail_mass_lb/KG_2_LB

  # estimates vertical tail structural mass [kg] using NDARC model 
  # based on tail area, dive speed, and 0.9 technology factor
  def _calc_vert_tail_mass_kg(self):
    if self.mission is None:
      return None
    vert_tail_area_ft2 = self.vert_tail_area_m2*(M_2_FT**2)
    dive_speed_kts = 1.4*self.mission.cruise_h_m_p_s*M_P_S_2_KTS
    vert_tail_mass_lb = (
        vert_tail_area_ft2
        *(0.00395*(vert_tail_area_ft2**0.2)*dive_speed_kts-0.4885)
        *0.9
    )
    return vert_tail_mass_lb/KG_2_LB

  # estimates fuselage structural mass [kg] using NDARC model 
  # based on wetted area, fineness ratio, dynamic pressure, and 0.9 technology factor
  def _calc_fuselage_mass_kg(self):
    if self.mission is None or self.environ is None:
      return None
    fuselage_wetted_area_ft2 = self.fuselage_wetted_area_m2*(M_2_FT**2)
    max_takeoff_mass_lb = self.max_takeoff_mass_kg*KG_2_LB
    fuselage_length_ft = self.fuselage_l_m*0.5*M_2_FT
    cruise_speed_m_p_s = self.mission.cruise_h_m_p_s
    dyn_pressure_lb_ft2 = (
        0.5*self.environ.air_density_sea_lvl_kg_p_m3*(cruise_speed_m_p_s**2.0)
        *N_P_M2_2_LB_P_FT2
    )
    fuselage_mass_lb = (
        0.052
        *(fuselage_wetted_area_ft2**1.086)
        *((3.8*1.5*max_takeoff_mass_lb)**0.177)
        *(fuselage_length_ft**-0.051)
        *(self.fuselage_fineness_ratio**-0.072)
        *(dyn_pressure_lb_ft2**0.241)
        *0.9
    )
    return fuselage_mass_lb/KG_2_LB
  
  # estimates boom structural mass [kg] using NDARC engine support model and cowling equations
  # based on EPU weight, rotor count, rotor diameter, and wing MAC
  def _calc_boom_mass_kg(self):
    if self.propulsion is None:
      return None
    rotor_count = self.propulsion.rotor_count
    rotor_diameter_m = self.propulsion.rotor_diameter_m
    wing_mac_m = self.wing_mac_m

    boom_mass_kg = (
      0.0412*(rotor_count**1.3762)/KG_2_LB
      + 6*0.2315*((1.2*rotor_diameter_m + wing_mac_m)**1.3476)
    )*2
    return boom_mass_kg

  # estimates landing gear mass [kg] using NDARC simple model 
  # assumes 3.25% of MTOW with factors for crashworthiness (1.14) and retractable gear (1.08)
  def _calc_landing_gear_mass_kg(self):
    return 0.0325*self.max_takeoff_mass_kg*1.14*1.08
    
  # NDARC Section 19.2 AFDD00 rotor + hub mass model
  # assumes 2-bladed rotors, flap natural frequency at 1.1 × max RPM
  # returns lift rotor + hub mass [kg]
  def _calc_lift_rotor_hub_mass_kg(self):
    if self.propulsion is None or self.environ is None:
      return None
    rotor_radius_ft = (self.propulsion.rotor_diameter_m / 2.0) * M_2_FT
    solidity = self.rotor_solidity
    sound_speed_m_p_s = self.environ.sound_speed_m_p_s
    tip_mach = self.propulsion.tip_mach
    rho_sl = self.environ.air_density_sea_lvl_kg_p_m3
    rho_alt = self.environ.air_density_max_alt_kg_p_m3
    over_torque_factor = self.over_torque_factor

    # common geometric term:
    term_common = (math.pi / 2.0/ 2.0) * self.propulsion.rotor_diameter_m * solidity * M_2_FT

    tip_speed_ft_s = (
      sound_speed_m_p_s
      * tip_mach
      * math.sqrt(rho_sl / rho_alt)
      * math.sqrt(over_torque_factor)
      * M_2_FT
    )

    lift_rotor_hub_mass_lb = (
      (
        0.0024419
        * (self.propulsion.lift_rotor_count)
        * (2.0 ** 0.53479)
        * (rotor_radius_ft ** 1.74231)
        * (term_common ** 0.77291)
        * (tip_speed_ft_s ** 0.87562)
        * (1.1 ** 2.51048)
      )
      + (
        0.00037547
        * (self.propulsion.lift_rotor_count)
        * (2.0 ** 0.71443)
        * (rotor_radius_ft ** 1.99321)
        * (term_common ** 0.79577)
        * (tip_speed_ft_s ** 0.96323)
        * (1.1 ** 0.46203)
        * (1.1 ** 2.58473)
      )
    )
    return lift_rotor_hub_mass_lb / KG_2_LB

  # NDARC Section 19.2 AFDD00 tilt rotor mass model
  # assumes 3-bladed rotors, flap natural frequency at 1.1 × max RPM
  # returns tilt rotor mass [kg]
  def _calc_tilt_rotor_mass_kg(self):
    if self.propulsion is None or self.environ is None:
      return None
    
    rotor_radius_ft = (self.propulsion.rotor_diameter_m / 2.0) * M_2_FT
    solidity = self.rotor_solidity
    sound_speed_m_p_s = self.environ.sound_speed_m_p_s
    tip_mach = self.propulsion.tip_mach
    rho_sl = self.environ.air_density_sea_lvl_kg_p_m3
    rho_alt = self.environ.air_density_max_alt_kg_p_m3
    over_torque_factor = self.over_torque_factor

    # common geometric term 
    term_common = (math.pi / 2.0 / 3.0) * self.propulsion.rotor_diameter_m * solidity * M_2_FT

    tip_speed_ft_s = (
      sound_speed_m_p_s
      * tip_mach
      * math.sqrt(rho_sl / rho_alt)
      * math.sqrt(over_torque_factor)
      * M_2_FT
    )

    tilt_rotor_mass_lb = (
      (
        0.0024419 * 1.1794
        * (self.propulsion.tilt_rotor_count)
        * (3.0 ** 0.53479)
        * (rotor_radius_ft ** 1.74231)
        * (term_common ** 0.77291)
        * (tip_speed_ft_s ** 0.87562)
        * (1.1 ** 2.51048)
      )
      + (
        0.00037547 * (1.1794 ** 1.02958)
        * (self.propulsion.tilt_rotor_count)
        * (3.0 ** 0.71443)
        * (rotor_radius_ft ** 1.99321)
        * (term_common ** 0.79577)
        * (tip_speed_ft_s ** 0.96323)
        * (1.1 ** 0.46203)
        * (1.1 ** 2.58473)
      )
    )
    return tilt_rotor_mass_lb / KG_2_LB

  # calculates aircraft empty mass
  def _calc_empty_mass_kg(self):
    structural_mass = (
      self.wing_mass_kg +
      self.horiz_tail_mass_kg +
      self.vert_tail_mass_kg +
      self.fuselage_mass_kg +
      self.boom_mass_kg +
      self.landing_gear_mass_kg +
      self.lift_rotor_hub_mass_kg +
      self.tilt_rotor_mass_kg
    )
    subsys_mass = (
      self.actuator_mass_kg +
      self.furnishings_mass_kg +
      self.environmental_control_system_mass_kg +
      self.avionics_mass_kg +
      self.hivolt_power_dist_mass_kg +
      self.lovolt_power_coms_mass_kg
    )
    subtotal = structural_mass + subsys_mass
    return subtotal * (1.0 + self.mass_margin_factor)

  # iterate Maximum Takeoff Weight (MTOW) until convergence
  def _iterate_mtow(self, tol=1e-3, max_iter=150):
    mtow_guess = self.max_takeoff_mass_kg
    history = []

    for i in range(max_iter):
      self.max_takeoff_mass_kg = mtow_guess

      # recalculate dependent masses on this guess
      empty_mass_kg = self.empty_mass_kg
      new_mtow = empty_mass_kg + self.payload_kg + self.fuel_mass_kg +self.personelle_payload_mass_kg

      delta = new_mtow - mtow_guess

      # store iteration data
      history.append({
          "iteration": i,
          "mtow_guess_kg": mtow_guess,
          "new_mtow_kg": new_mtow,
          "delta_kg": delta,
          "empty_mass_kg": empty_mass_kg,
          "fuel_mass_kg": self.fuel_mass_kg,
          "payload_mass_kg": self.payload_kg,
          "personelle_payload_mass_kg": self.personelle_payload_mass_kg,
          "total_fuel_consumed": self._calc_total_mission_fuel_consumption()
      })

      if abs(delta) < tol:
        self.max_takeoff_mass_kg = new_mtow
        return new_mtow, history
      
      mtow_guess = new_mtow
    
    self.max_takeoff_mass_kg = mtow_guess

    return mtow_guess, history

  @property
  def max_takeoff_mass_kg(self):
    return self._max_takeoff_mass_kg
  
  @max_takeoff_mass_kg.setter
  def max_takeoff_mass_kg(self, value):
    self._max_takeoff_mass_kg = value

  @property
  def payload_kg(self):
    return self._payload_kg
  
  @property
  def personelle_payload_mass_kg(self):
    return self._personelle_payload_kg

  @property
  def mass_without_payload(self):
    return self._calc_mass_without_payload

  @property
  def vehicle_cl_max(self):
    return self._vehicle_cl_max

  @property
  def wing_taper_ratio(self):
    return self._wing_taper_ratio

  @property
  def wingspan_m(self):
    return self._wingspan_m

  @property
  def d_value_m(self):
    return self._d_value_m

  @property
  def stall_speed_m_p_s(self):
    return self._stall_speed_m_p_s

  @property
  def fuselage_l_m(self):
    return self._fuselage_l_m

  @property
  def fuselage_w_m(self):
    return self._fuselage_w_m

  @property
  def fuselage_h_m(self):
    return self._fuselage_h_m

  @property
  def wing_airfoil_cd_at_cruise_cl(self):
    return self._wing_airfoil_cd_at_cruise_cl

  @property
  def empennage_airfoil_cd0(self):
    return self._empennage_airfoil_cd0

  @property
  def span_effic_factor(self):
    return self._span_effic_factor

  @property
  def trim_drag_factor(self):
    return self._trim_drag_factor

  @property
  def landing_gear_drag_area_m2(self):
    return self._landing_gear_drag_area_m2

  @property
  def excres_protub_factor(self):
    return self._excres_protub_factor

  @property
  def horiz_tail_vol_coeff(self):
    return self._horiz_tail_vol_coeff

  @property
  def vert_tail_vol_coeff(self):
    return self._vert_tail_vol_coeff

  @property
  def ratio_disk_to_stopped_rotor_area(self):
    return self._ratio_disk_to_stopped_rotor_area

  @property
  def wing_t_p_c(self):
    return self._wing_t_p_c

  @property
  def actuator_mass_kg(self):
    return self._actuator_mass_kg

  @property
  def furnishings_mass_kg(self):
    return self._furnishings_mass_kg

  @property
  def environmental_control_system_mass_kg(self):
    return self._environmental_control_system_mass_kg

  @property
  def avionics_mass_kg(self):
    return self._avionics_mass_kg

  @property
  def hivolt_power_dist_mass_kg(self):
    return self._hivolt_power_dist_mass_kg

  @property
  def lovolt_power_coms_mass_kg(self):
    return self._lovolt_power_coms_mass_kg

  @property
  def mass_margin_factor(self):
    return self._mass_margin_factor

  @property
  def environ(self):
    return copy.deepcopy(self._environ)

  @property
  def mission(self):
    return copy.deepcopy(self._mission)

  @property
  def power(self):
    return copy.deepcopy(self._power)

  @property
  def propulsion(self):
    return copy.deepcopy(self._propulsion)

  @property
  def hover_shaft_power_kw(self):
    return self._calc_hover_shaft_power_kw()

  @property
  def wing_area_m2(self):
    return self._calc_wing_area_m2()

  @property
  def cruise_cl(self):
    return self._calc_cruise_cl()

  @property
  def fuselage_fineness_ratio(self):
    return self._calc_fuselage_fineness_ratio()

  @property
  def fuselage_cd0_p_cf(self):
    return self._calc_fuselage_cd0_p_cf()

  @property
  def fuselage_cruise_reynolds(self):
    return self._calc_fuselage_cruise_reynolds()

  @property
  def fuselage_cf(self):
    return self._calc_fuselage_cf()

  @property
  def fuselage_cd0(self):
    return self._calc_fuselage_cd0()

  @property
  def wing_aspect_ratio(self):
    return self._calc_wing_aspect_ratio()

  @property
  def induced_drag_cdi(self):
    return self._calc_induced_drag_cdi()

  @property
  def wing_root_chord_m(self):
    return self._calc_wing_root_chord_m()

  @property
  def wing_mac_m(self):
    return self._calc_wing_mac_m()

  @property
  def horiz_tail_area_m2(self):
    return self._calc_horiz_tail_area_m2()

  @property
  def vert_tail_area_m2(self):
    return self._calc_vert_tail_area_m2()

  @property
  def horiz_tail_cd0(self):
    return self._calc_horiz_tail_cd0()

  @property
  def vert_tail_cd0(self):
    return self._calc_vert_tail_cd0()

  @property
  def landing_gear_cd0(self):
    return self._calc_landing_gear_cd0()

  @property
  def stopped_rotor_cd0(self):
    return self._calc_stopped_rotor_cd0()

  @property
  def cruise_cd(self):
    return self._calc_cruise_cd()

  @property
  def cruise_l_p_d(self):
    return self._calc_cruise_l_p_d()

  @property
  def total_drag_coef(self):
    return self._calc_total_drag_coef()
    
  @property
  def fuselage_wetted_area_m2(self):
    return self._calc_fuselage_wetted_area_m2()

  @property
  def over_torque_factor(self):
    return self._calc_over_torque_factor()

  @property
  def rotor_solidity(self):
    return self._calc_rotor_solidity()
  
  @property 
  def hover_fuel_consumption(self):
    return self._calc_hover_fuel_consumption()  

  @property
  def depart_taxi_avg_shaft_power_kw(self):
    return self._calc_depart_taxi_avg_shaft_power_kw()

  @property
  def depart_taxi_fuel_consumption(self):
    return self._calc_depart_taxi_fuel_consumption()

  @property
  def depart_taxi_kg_per_s(self):
    return self._calc_depart_taxi_kg_per_s()

  @property
  def hover_climb_avg_shaft_power_kw(self):
    return self._calc_hover_climb_avg_shaft_power_kw()

  @property
  def hover_climb_fuel_consumption(self):
    return self._calc_hover_climb_fuel_consumption()

  @property
  def hover_climb_kg_per_s(self):
    return self._calc_hover_climb_kg_per_s()
    
  @property
  def trans_climb_avg_shaft_power_kw(self):
    return self._calc_trans_climb_avg_shaft_power_kw()
  
  @property
  def trans_climb_fuel_consumption(self):
    return self._calc_trans_climb_fuel_consumption()
  
  @property
  def trans_climb_kg_per_s(self):
    return self._calc_reserve_trans_climb_kg_per_s()

  @property
  def depart_proc_avg_shaft_power_kw(self):
    return self._calc_depart_proc_avg_shaft_power_kw()
  
  @property
  def depart_proc_fuel_consumption(self):
    return self._calc_depart_proc_fuel_consumption()
  
  @property
  def depart_proc_kg_per_s(self):
    return self._calc_depart_proc_kg_per_s()

  @property
  def accel_climb_avg_shaft_power_kw(self):
    return self._calc_accel_climb_avg_shaft_power_kw()
  
  @property
  def accel_climb_fuel_consumption(self):
    return self._calc_accel_climb_fuel_consumption()
  
  @property
  def accel_climb_kg_per_s(self):
    return self._calc_accel_climb_kg_per_s()
  
#took the cruise and added initial because the mission has 2 ccruise segments
  @property
  def cruise_initial_avg_shaft_power_kw(self):
    return self._calc_cruise_initial_avg_shaft_power_kw()
  
  @property
  def cruise_initial_fuel_consumption(self):
    return self._calc_cruise_initial_fuel_consumption()

  @property
  def cruise_initial_kg_per_s(self):
    return self._calc_cruise_initial_kg_per_s()
  
  #the following is all new mission segments with the same physics
  @property
  def decel_descend_drop_point_avg_shaft_power_kw(self):
    return self._calc_decel_descend_drop_point_avg_shaft_power_kw()
  
  @property
  def decel_descend_drop_point_fuel_consumption(self):
    return self._calc_decel_descend_drop_point_fuel_consumption()

  @property
  def decel_descend_drop_point_kg_per_s(self):
    return self._calc_decel_descend_drop_point_kg_per_s()

  @property
  def drop_payload_avg_shaft_power_kw(self):
    return self._calc_drop_payload_avg_shaft_power_kw()
  
  @property
  def drop_payload_fuel_consumption(self):
    return self._calc_drop_payload_fuel_consumption()
  
  @property
  def drop_payload_kg_per_s(self):
    return self._calc_drop_payload_kg_per_s()
  
  @property
  def accel_climb_drop_point_avg_shaft_power_kw(self):
    return self._calc_accel_climb_drop_point_avg_shaft_power_kw()
  
  @property
  def accel_climb_drop_point_fuel_consumption(self):
    return self._calc_accel_climb_drop_point_fuel_consumption()
  
  @property
  def accel_climb_drop_point_kg_per_s(self):
    return self._calc_accel_climb_drop_point_kg_per_s()
  
  @property
  def cruise_return_avg_shaft_power_kw(self):
    return self._calc_cruise_return_avg_shaft_power_kw()
  
  @property
  def cruise_return_fuel_consumption(self):
    return self._calc_cruise_return_fuel_consumption()
  
  @property
  def cruise_return_kg_per_s(self):
    return self._calc_cruise_return_kg_per_s()

  #this is unchanged from the original code
  @property
  def decel_descend_avg_shaft_power_kw(self):
    return self._calc_decel_descend_avg_shaft_power_kw()
  
  @property
  def decel_descend_fuel_consumption(self):
    return self._calc_decel_descend_fuel_consumption()

  @property
  def decel_descend_kg_per_s(self):
    return self._calc_decel_descend_kg_per_s()
    
  @property
  def arrive_proc_avg_shaft_power_kw(self):
    return self._calc_arrive_proc_avg_shaft_power_kw()
  
  @property
  def arrive_proc_fuel_consumption(self):
    return self._calc_arrive_proc_fuel_consumption()

  @property
  def arrive_proc_kg_per_s(self):
    return self._calc_arrive_proc_kg_per_s()
  
  @property
  def trans_descend_avg_shaft_power_kw(self):
    return self._calc_trans_descend_avg_shaft_power_kw()
  
  @property
  def trans_descend_fuel_consumption(self):
    return self._calc_trans_descend_fuel_consumption()

  @property
  def trans_descend_kg_per_s(self):
    return self._calc_trans_descend_kg_per_s()

  @property
  def hover_descend_avg_shaft_power_kw(self):
    return self._calc_hover_descend_avg_shaft_power_kw()
  
  @property
  def hover_descend_fuel_consumption(self):
    return self._calc_hover_descend_fuel_consumption()

  @property
  def hover_descend_kg_per_s(self):
    return self._calc_hover_descend_kg_per_s()
  
  @property
  def arrive_taxi_avg_shaft_power_kw(self):
    return self._calc_arrive_taxi_avg_shaft_power_kw()
  
  @property
  def arrive_taxi_fuel_consumption(self):
    return self._calc_arrive_taxi_fuel_consumption()
  
  @property
  def arrive_taxi_kg_per_s(self):
    return self._calc_arrive_taxi_kg_per_s()

  @property
  def reserve_hover_climb_avg_shaft_power_kw(self):
    return self._calc_reserve_hover_climb_avg_shaft_power_kw()
  
  @property
  def reserve_hover_climb_fuel_consumption(self):
    return self._calc_reserve_hover_climb_fuel_consumption()

  @property
  def reserve_hover_climb_kg_per_s(self):
    return self._calc_reserve_hover_climb_kg_per_s()

  @property
  def reserve_trans_climb_avg_shaft_power_kw(self):
    return self._calc_reserve_trans_climb_avg_shaft_power_kw()
  
  @property
  def reserve_trans_climb_fuel_consumption(self):
    return self._calc_reserve_trans_climb_fuel_consumption()

  @property
  def reserve_trans_climb_kg_per_s(self):
    return self._calc_reserve_trans_climb_kg_per_s()
  
  @property
  def reserve_accel_climb_avg_shaft_power_kw(self):
    return self._calc_reserve_accel_climb_avg_shaft_power_kw()
  
  @property
  def reserve_accel_climb_fuel_consumption(self):
    return self._calc_reserve_accel_climb_fuel_consumption()

  @property
  def reserve_accel_climb_kg_per_s(self):
    return self._calc_reserve_accel_climb_kg_per_s()
  
  @property
  def reserve_cruise_avg_shaft_power_kw(self):
    return self._calc_reserve_cruise_avg_shaft_power_kw()
  
  @property
  def reserve_cruise_fuel_consumption(self):
    return self._calc_reserve_cruise_fuel_consumption()

  @property
  def reserve_cruise_kg_per_s(self):
    return self._calc_reserve_cruise_kg_per_s()

  @property
  def reserve_decel_descend_avg_shaft_power_kw(self):
    return self._calc_reserve_decel_descend_avg_shaft_power_kw()
  
  @property
  def reserve_decel_descend_fuel_consumption(self):
    return self._calc_reserve_decel_descend_fuel_consumption()

  @property
  def reserve_decel_descend_kg_per_s(self):
    return self._calc_reserve_decel_descend_kg_per_s()

  @property
  def reserve_trans_descend_avg_shaft_power_kw(self):
    return self._calc_reserve_trans_descend_avg_shaft_power_kw()

  @property
  def reserve_trans_descend_fuel_consumption(self):
    return self._calc_reserve_trans_descend_fuel_consumption()

  @property
  def reserve_trans_descend_kg_per_s(self):
    return self._calc_reserve_trans_descend_kg_per_s()

  @property
  def reserve_hover_descend_avg_shaft_power_kw(self):
    return self._calc_reserve_hover_descend_avg_shaft_power_kw()

  @property
  def reserve_hover_descend_fuel_consumption(self):
    return self._calc_reserve_hover_descend_fuel_consumption()

  @property
  def reserve_hover_descend_kg_per_s(self):
    return self._calc_reserve_hover_descend_kg_per_s()

  @property
  def total_mission_fuel_consumption(self):
    return self._calc_total_mission_fuel_consumption()

  @property
  def total_reserve_fuel_consumption(self):
    return self._calc_total_reserve_fuel_consumption()

  @property
  def fuel_mass_kg(self):
    return self._calc_fuel_mass_kg()

  @property
  def wing_mass_kg(self):
    return self._calc_wing_mass_kg()

  @property
  def horiz_tail_mass_kg(self):
    return self._calc_horiz_tail_mass_kg()

  @property
  def vert_tail_mass_kg(self):
    return self._calc_vert_tail_mass_kg()

  @property
  def fuselage_mass_kg(self):
    return self._calc_fuselage_mass_kg()

  @property
  def boom_mass_kg(self):
    return self._calc_boom_mass_kg()

  @property
  def landing_gear_mass_kg(self):
    return self._calc_landing_gear_mass_kg()

  @property
  def lift_rotor_hub_mass_kg(self):
    return self._calc_lift_rotor_hub_mass_kg()

  @property
  def tilt_rotor_mass_kg(self):
    return self._calc_tilt_rotor_mass_kg()

  @property
  def empty_mass_kg(self):
    return self._calc_empty_mass_kg()