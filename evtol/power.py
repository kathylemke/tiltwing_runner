# power.py
#
# A Python class containing aircraft power characteristics for fuel powered propellor engine
#
# Written by Katherine Lemke
# Other contributors: Bradley Denby, Darshan Sarojini, Dylan Hogge, John Riris, Khoa Nguyen
#
# See the LICENSE file for the license

# import Python modules
import json # json parsing

class Power:
  # class constructor
  def __init__(self, path_to_json: str):
    # open and load JSON specification
    ifile = open(path_to_json, 'r')
    ijson = json.load(ifile)
    # power properties
    self._BSFC_general_kWs = \
     ijson['power']['BSFC_general_kWs']
    self._hover_power_effic = ijson['power']['hover_power_effic']
    # close JSON file
    ifile.close()

  # scale the battery specific energy to the accessible energy fraction and
  # account for the integration factor; BOL = beginning of life
  def _calc_batt_bol_usable_spec_energy_w_h_p_kg(self):
    return \
     self.batt_int_factor*(1.0-self.batt_inaccessible_energy_frac)*\
     self.batt_spec_energy_w_h_p_kg

  # same as BOL with an additional factor accounting for end-of-life capacity
  def _calc_batt_eol_usable_spec_energy_w_h_p_kg(self):
    return \
     self.batt_eol_capacity*self._calc_batt_bol_usable_spec_energy_w_h_p_kg()

  # defines equivalence check for this class
  def __eq__(self, other):
    if isinstance(other, Power):
      return (\
        self.BSFC_general_kWs == other.BSFC_general_kWs and
       self.hover_power_effic == other.hover_power_effic)
    else:
      return NotImplemented

  @property
  def BSFC_general_kWs(self):
    return self._BSFC_general_kWs

  @property
  def hover_power_effic(self):
    return self._hover_power_effic
