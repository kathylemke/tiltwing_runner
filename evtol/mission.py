# mission.py
#
# A Python class containing aircraft mission characteristics
#
# Written by First Last
# Other contributors: Bradley Denby, Darshan Sarojini, Dylan Hogge, John Riris, Khoa Nguyen
#
# See the LICENSE file for the license

# import Python modules
import json # json parsing

class Mission:
  # class constructor
  def __init__(self, path_to_json: str):
    # open and load JSON specification
    ifile = open(path_to_json, 'r')
    ijson = json.load(ifile)
    # mission properties
    self._depart_taxi_avg_h_m_p_s = ijson['mission']['depart_taxi_avg_h_m_p_s']
    self._depart_taxi_s = ijson['mission']['depart_taxi_s']
    self._hover_climb_avg_v_m_p_s = ijson['mission']['hover_climb_avg_v_m_p_s']
    self._hover_climb_s = ijson['mission']['hover_climb_s']
    self._trans_climb_avg_h_m_p_s = ijson['mission']['trans_climb_avg_h_m_p_s']
    self._trans_climb_v_m_p_s= ijson['mission']['trans_climb_v_m_p_s']
    self._trans_climb_s = ijson['mission']['trans_climb_s']
    self._depart_proc_h_m_p_s = ijson['mission']['depart_proc_h_m_p_s']
    self._depart_proc_s = ijson['mission']['depart_proc_s']
    self._accel_climb_avg_h_m_p_s = ijson['mission']['accel_climb_avg_h_m_p_s']
    self._accel_climb_v_m_p_s = ijson['mission']['accel_climb_v_m_p_s']
    self._accel_climb_s = ijson['mission']['accel_climb_s']
#modified
    self._cruise_h_m_p_s = ijson['mission']['cruise_h_m_p_s']
    self._cruise_initial_s = ijson['mission']['cruise_initial_s']
#new mission
    self._decel_descend_drop_point_avg_h_m_p_s = ijson['mission']['decel_descend_drop_point_avg_h_m_p_s']
    self._decel_descend_drop_point_v_m_p_s = ijson['mission']['decel_descend_drop_point_v_m_p_s']
    self._decel_descend_drop_point_s = ijson['mission']['decel_descend_drop_point_s']
    self._drop_payload_h_m_p_s = ijson['mission']['drop_payload_h_m_p_s']
    self._drop_payload_s = ijson['mission']['drop_payload_s']
    self._accel_climb_drop_point_avg_h_m_p_s = ijson['mission']['accel_climb_drop_point_avg_h_m_p_s']
    self._accel_climb_drop_point_v_m_p_s = ijson['mission']['accel_climb_drop_point_v_m_p_s']
    self._accel_climb_drop_point_s = ijson['mission']['accel_climb_drop_point_s']
    self._cruise_return_s = ijson['mission']['cruise_initial_s']

#same as before
    self._decel_descend_avg_h_m_p_s = ijson['mission']['decel_descend_avg_h_m_p_s']
    self._decel_descend_v_m_p_s = ijson['mission']['decel_descend_v_m_p_s']
    self._decel_descend_s = ijson['mission']['decel_descend_s']

    self._arrive_proc_h_m_p_s = ijson['mission']['arrive_proc_h_m_p_s']
    self._arrive_proc_s = ijson['mission']['arrive_proc_s']
    self._trans_descend_avg_h_m_p_s = \
     ijson['mission']['trans_descend_avg_h_m_p_s']
    self._trans_descend_v_m_p_s = ijson['mission']['trans_descend_v_m_p_s']
    self._trans_descend_s = ijson['mission']['trans_descend_s']
    self._hover_descend_avg_v_m_p_s = \
     ijson['mission']['hover_descend_avg_v_m_p_s']
    self._hover_descend_s = ijson['mission']['hover_descend_s']
    self._arrive_taxi_avg_h_m_p_s = ijson['mission']['arrive_taxi_avg_h_m_p_s']
    self._arrive_taxi_s = ijson['mission']['arrive_taxi_s']
    self._reserve_hover_climb_avg_v_m_p_s = \
     ijson['mission']['reserve_hover_climb_avg_v_m_p_s']
    self._reserve_hover_climb_s = ijson['mission']['reserve_hover_climb_s']
    self._reserve_trans_climb_avg_h_m_p_s = \
     ijson['mission']['reserve_trans_climb_avg_h_m_p_s']
    self._reserve_trans_climb_v_m_p_s = \
     ijson['mission']['reserve_trans_climb_v_m_p_s']
    self._reserve_trans_climb_s = ijson['mission']['reserve_trans_climb_s']
    self._reserve_accel_climb_avg_h_m_p_s = \
     ijson['mission']['reserve_accel_climb_avg_h_m_p_s']
    self._reserve_accel_climb_v_m_p_s = \
     ijson['mission']['reserve_accel_climb_v_m_p_s']
    self._reserve_accel_climb_s = ijson['mission']['reserve_accel_climb_s']
    self._reserve_cruise_h_m_p_s = ijson['mission']['reserve_cruise_h_m_p_s']
    self._reserve_cruise_s = ijson['mission']['reserve_cruise_s']
    self._reserve_decel_descend_avg_h_m_p_s = \
     ijson['mission']['reserve_decel_descend_avg_h_m_p_s']
    self._reserve_decel_descend_v_m_p_s = \
     ijson['mission']['reserve_decel_descend_v_m_p_s']
    self._reserve_decel_descend_s = ijson['mission']['reserve_decel_descend_s']
    self._reserve_trans_descend_avg_h_m_p_s = \
     ijson['mission']['reserve_trans_descend_avg_h_m_p_s']
    self._reserve_trans_descend_v_m_p_s = \
     ijson['mission']['reserve_trans_descend_v_m_p_s']
    self._reserve_trans_descend_s = ijson['mission']['reserve_trans_descend_s']
    self._reserve_hover_descend_avg_v_m_p_s = \
     ijson['mission']['reserve_hover_descend_avg_v_m_p_s']
    self._reserve_hover_descend_s = ijson['mission']['reserve_hover_descend_s']
    self._cruise_s = ijson['mission']['cruise_s']
    # add some new variables needed for simulation code
    self._takeoff_alt_m = ijson['mission']['takeoff_alt_m']
    self._land_alt_m = ijson['mission']['land_alt_m']
    self._cruise_alt_m = ijson['mission']['cruise_alt_m']

    # close JSON file
    ifile.close()

# no need to calculate new mission paramters for sim but few new properties
# these properties do not change from input json to aircraft code to input json for sim
# in big picture, learning alg could change these parameters because they do influence other calculations
# need to further develop mission for it to be dynamic in mission optimization


  @property
  def takeoff_alt_m(self):
    return self._takeoff_alt_m
  
  @property
  def land_alt_m(self):
    return self._land_alt_m
  
  @property
  def cruise_alt_m(self):
    return self._cruise_alt_m

  @property
  def depart_taxi_avg_h_m_p_s(self):
    return self._depart_taxi_avg_h_m_p_s

  @property
  def depart_taxi_s(self):
    return self._depart_taxi_s

  @property
  def hover_climb_avg_v_m_p_s(self):
    return self._hover_climb_avg_v_m_p_s

  @property
  def hover_climb_s(self):
    return self._hover_climb_s

  @property
  def trans_climb_avg_h_m_p_s(self):
    return self._trans_climb_avg_h_m_p_s

  @property
  def trans_climb_v_m_p_s(self):
    return self._trans_climb_v_m_p_s

  @property
  def trans_climb_s(self):
    return self._trans_climb_s

  @property
  def depart_proc_h_m_p_s(self):
    return self._depart_proc_h_m_p_s

  @property
  def depart_proc_s(self):
    return self._depart_proc_s

  @property
  def accel_climb_avg_h_m_p_s(self):
    return self._accel_climb_avg_h_m_p_s

  @property
  def accel_climb_v_m_p_s(self):
    return self._accel_climb_v_m_p_s

  @property
  def accel_climb_s(self):
    return self._accel_climb_s
  
  #modifications

  @property
  def cruise_h_m_p_s(self):
    return self._cruise_h_m_p_s

  @property
  def cruise_initial_s(self):
    return self._cruise_initial_s
  
  #new segments
  @property
  def decel_descend_drop_point_avg_h_m_p_s(self):
    return self._decel_descend_drop_point_avg_h_m_p_s

  @property
  def decel_descend_drop_point_v_m_p_s(self):
    return self._decel_descend_drop_point_v_m_p_s

  @property
  def decel_descend_drop_point_s(self):
    return self._decel_descend_drop_point_s
  
  @property
  def drop_payload_h_m_p_s(self):
    return self._drop_payload_h_m_p_s
  
  @property
  def drop_payload_s(self):
    return self._drop_payload_s
  
  @property
  def accel_climb_drop_point_avg_h_m_p_s(self):
    return self._accel_climb_drop_point_avg_h_m_p_s

  @property
  def accel_climb_drop_point_v_m_p_s(self):
    return self._accel_climb_drop_point_v_m_p_s

  @property
  def accel_climb_drop_point_s(self):
    return self._accel_climb_drop_point_s
  
  @property
  def cruise_return_s(self):
    return self._cruise_return_s
  #original code
  @property
  def decel_descend_avg_h_m_p_s(self):
    return self._decel_descend_avg_h_m_p_s

  @property
  def decel_descend_v_m_p_s(self):
    return self._decel_descend_v_m_p_s

  @property
  def decel_descend_s(self):
    return self._decel_descend_s

  @property
  def arrive_proc_h_m_p_s(self):
    return self._arrive_proc_h_m_p_s

  @property
  def arrive_proc_s(self):
    return self._arrive_proc_s

  @property
  def trans_descend_avg_h_m_p_s(self):
    return self._trans_descend_avg_h_m_p_s

  @property
  def trans_descend_v_m_p_s(self):
    return self._trans_descend_v_m_p_s

  @property
  def trans_descend_s(self):
    return self._trans_descend_s

  @property
  def hover_descend_avg_v_m_p_s(self):
    return self._hover_descend_avg_v_m_p_s

  @property
  def hover_descend_s(self):
    return self._hover_descend_s

  @property
  def arrive_taxi_avg_h_m_p_s(self):
    return self._arrive_taxi_avg_h_m_p_s

  @property
  def arrive_taxi_s(self):
    return self._arrive_taxi_s

  @property
  def reserve_hover_climb_avg_v_m_p_s(self):
    return self._reserve_hover_climb_avg_v_m_p_s

  @property
  def reserve_hover_climb_s(self):
    return self._reserve_hover_climb_s

  @property
  def reserve_trans_climb_avg_h_m_p_s(self):
    return self._reserve_trans_climb_avg_h_m_p_s

  @property
  def reserve_trans_climb_v_m_p_s(self):
    return self._reserve_trans_climb_v_m_p_s

  @property
  def reserve_trans_climb_s(self):
    return self._reserve_trans_climb_s

  @property
  def reserve_accel_climb_avg_h_m_p_s(self):
    return self._reserve_accel_climb_avg_h_m_p_s

  @property
  def reserve_accel_climb_v_m_p_s(self):
    return self._reserve_accel_climb_v_m_p_s

  @property
  def reserve_accel_climb_s(self):
    return self._reserve_accel_climb_s

  @property
  def reserve_cruise_h_m_p_s(self):
    return self._reserve_cruise_h_m_p_s

  @property
  def reserve_cruise_s(self):
    return self._reserve_cruise_s

  @property
  def reserve_decel_descend_avg_h_m_p_s(self):
    return self._reserve_decel_descend_avg_h_m_p_s

  @property
  def reserve_decel_descend_v_m_p_s(self):
    return self._reserve_decel_descend_v_m_p_s

  @property
  def reserve_decel_descend_s(self):
    return self._reserve_decel_descend_s

  @property
  def reserve_trans_descend_avg_h_m_p_s(self):
    return self._reserve_trans_descend_avg_h_m_p_s

  @property
  def reserve_trans_descend_v_m_p_s(self):
    return self._reserve_trans_descend_v_m_p_s

  @property
  def reserve_trans_descend_s(self):
    return self._reserve_trans_descend_s

  @property
  def reserve_hover_descend_avg_v_m_p_s(self):
    return self._reserve_hover_descend_avg_v_m_p_s

  @property
  def reserve_hover_descend_s(self):
    return self._reserve_hover_descend_s

  @property
  def cruise_s(self):
      return self._cruise_s
