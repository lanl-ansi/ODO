//
//  Net.h
//
//  Created by Guanglei Wang on 16/06/2014.
//

#ifndef Net_h
#define Net_h

#include <map>
#include <math.h>
#include "Bus.h"
#include "Gen.h"
#include "Line.h"
#include <gravity/Net.h>
#include <gravity/model.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <assert.h>
#include <ctime>

using namespace gravity;


/** @brief Seasons type */
typedef enum { summer_=0, winter_=1, spring_=2, autumn_=3} SeasonType;


/** @brief Wind Farm Data */
class WindData{
public:
    double _cap;
    double _cap_cost;
    double _fixed_cost;
    double _var_cost;
    unsigned _lifetime;
};

/** @brief PV Gens Data */
class PVData{
public:
    double _cap;
    double _cap_cost;
    double _fixed_cost;
    double _var_cost;
    unsigned _lifetime;
};

/** @brief Resiliency Scenarios */
class Scenario{
public:
    string                  _name;
    tuple<int,int,int,int>  _year_month_day_hour;
    int                     _nb_hours;
    map<string,aux*>        _out_gens;
    map<string,Arc*>        _out_arcs;
    Scenario(const string& name, int year, int month, int day, int hour, int nb_hours):_name(name), _year_month_day_hour({year,month,day,hour}), _nb_hours(nb_hours){};
};

/** @brief A Month has a name, belongs to a given season and has stats on the number of week/weekends and peak days*/
class Month{
public:
    string _name;
    SeasonType _season;
    unsigned _nb_week_days = 0;
    unsigned _nb_weekend_days = 0;
    unsigned _nb_peak_days = 0;
    double _wind_average[24];
    double _solar_average[24];
    double _wind_variance[24];
    double _solar_variance[24];
    
    Month(string name, SeasonType season, unsigned nb_week_days, unsigned nb_weekend_days, unsigned nb_peak_days):_name(name), _season(season), _nb_week_days(nb_week_days), _nb_weekend_days(nb_weekend_days), _nb_peak_days(nb_peak_days){};
    void print() const{
        cout << _name << " : " << "in season " << _season << ", nb_week_days = " << _nb_week_days<< ", nb_weekend_days = " << _nb_weekend_days << ", nb_peak_days = " << _nb_peak_days << endl;
        cout << "Hourly Average Wind Data: ";
        for (unsigned t=0; t<24; t++) {
            cout << _wind_average[t];
            if (t<23) {
                cout << ", ";
            }
        }
        cout << ";" << endl;
        cout << "Hourly Variance in Wind Data: ";
        for (unsigned t=0; t<24; t++) {
            cout << _wind_variance[t];
            if (t<23) {
                cout << ", ";
            }
        }
        cout << ";" << endl;
    };
};


typedef enum { ACPOL, ACRECT, DISTF, CDISTF, LDISTF, QC, QC_SDP, OTS, DF, SOCP, SDP, DC, QC_OTS_L, QC_OTS_N, QC_OTS_O, SOCP_OTS, GRB_TEST } PowerModelType;
// A PowerNet extends a standard network by incorporating additional parameters related to power systems.
class PowerNet: public Net {

public:
    /**Date represented as a tuple <Year,Month,Day,hour>*/
    time_t                                                              _rawtime;
    struct tm*                                                          _start_date;/**< First time stamp in simulation */

    map<string, Node*>                                                  _load_map;
    map<string, shared_ptr<Scenario>>                                   _res_scenarios;/**< Resiliency scenarios */
    vector<string>                                                      _scenario_names;/**< Resiliency scenarios */
    size_t _max_it = 10000;
    size_t _max_time = 3600;
    double  _tol = 1e-6;
    unsigned _nb_years = 5;
    unsigned _nb_hours = 24;
    double _inflation_rate = 0.02, _demand_growth = 0.02;
    double _pv_cap_cost = 2500, _wind_cap_cost = 2500, _pv_eff = 0.8, _wind_eff = 0.8;
    bool add_3d_nlin = true;
    string ref_bus;
    double bMVA; /**< Base MVA */
    double bV; /**< Base Voltage */
    double _power_factor = 1, PeakPVEfficiency = 0.15;
    double m_theta_lb = 0, m_theta_ub = 0; /**< BigM values for phase angles */
    double vmin = 0, vmax = 0; /**< Global values for voltage bounds */
    size_t nb_nodes = 0, nb_branches = 0, nb_gens = 0;
    set<int> _microgrid_ids;
    param<> cb_f, cb_v;/**< Battery fixed nd variable costs */
    param<> inverter_capcost, gen_capcost, expansion_capcost, pv_capcost, pv_varcost;/**< Inverter, Generators, Expansion and PV capital costs */
    
    param<> pg_min, pg_max, qg_min, qg_max, pg_s, qg_s; /**< Upper and lower bounds on generation along with nominal values (default set points)*/
    param<> pb_min, pb_max, qb_min, qb_max; /**< Upper and lower bounds on battery generation */
    param<> pv_min, pv_max, qv_min, qv_max; /**< Upper and lower bounds on PV generation */
    param<> pw_min, pw_max, qw_min, qw_max; /**< Upper and lower bounds on wind generation */
    param<> pv_out; /**< Normalized PV generation on bus */
    param<> c0, c1, c2; /**< Generation costs */
    param<> ramp_up, ramp_down; /**< Generation ramp up/down params */
    param<> gen_eff; /**< Diesel generation efficiency */
    param<int> min_ut, min_dt; /**< Minimum Uptime and Downtime for Generators */
    int max_ident_units; /**< Maximum number of identical units */
    param<int> min_diesel_invest, max_diesel_invest; /**< Minimum and Maximum number of Diesel generation investment at a given bus */
    param<int> min_batt_invest, max_batt_invest; /**< Minimum and Maximum number of Battery investment at a given bus */
    param<> th_min, th_max, tan_th_min, tan_th_max, cphi, sphi, cos_d; /**< Upper and lower bounds on phase angles. tan is for the tangent of the angles */
    param<> v_min, v_max, v_s; /**< Voltage bounds and nominal values (default set points) */
    param<> v_diff_max; /**< Voltage bounds difference upper bound */
    param<> w_min, w_max; /**< Voltage bounds in lifted W space */
    param<> pl = param<>("pl"), ql = param<>("ql"), pl_ratio; /**< Load vectors */
    param<> eff_a, eff_b; /**< Efficiency params */
    unsigned _nb_eff_pieces = 1;/**< Number of pieces for the efficiency curves */
    indices _eff_pieces;/**< Set of indices for pieces for the efficiency curves */
    param<> tbound_max_tan, tbound_min_tan;  /** tan (th_min), tan(th_max) **/
    param<> r,x,g, b, ch, tr, as, S_max, wr_min, wr_max, wi_min, wi_max; /**< Power lines parameters, resp., impedance, line charging, and thermal limits. w params are for lifted variavles in W space */
    param<> g_ff, g_ft, g_tt, g_tf, b_ff, b_ft, b_tf, b_tt, Y_t, Y_charge, Y_charge_t; /**< Transformers phase shifters parameters, e.g., g_ft = (-a->b*a->cc - a->g*a->dd)/(pow(a->cc,2)+pow(a->dd,2)) where a->cc = a->tr*cos(a->as) and a->dd = a->tr*sin(a->as);*/
    
    /** Set of all diesel generators */
    std::vector<Gen*> gens;
    
    /** Set of all diesel generators in DEROPT */
    std::vector<DieselGen> _all_diesel_gens;
    
    /** Set of potential diesel generators */
    std::vector<Gen*> _potential_diesel_gens;
    
    /** Set of existing diesel generators */
    std::vector<Gen*> _existing_diesel_gens;
    
    
    /** Set of all PV generators */
    std::vector<PV*> _all_PV_gens;
    
    /** Set of potential PV generators */
    std::vector<PV*> _potential_PV_gens;
    
    /** Set of existing PV generators */
    std::vector<PV*> _existing_PV_gens;
    
    /** Set of existing + potential diesel generators */
    //    std::vector<DieselGen*> _diesel_gens;
    
    /** Set of all wind generators */
    std::vector<WindGen*> _all_wind_gens;
    
    /** Set of existing diesel generators */
    std::vector<WindGen*> _existing_wind_gens;
    
    /** Set of potential diesel generators */
    std::vector<WindGen*> _potential_wind_gens;
    
    /** Set of all battery inverters */
    std::vector<BatteryInverter> _all_battery_inverters;
    
    /** Set of all available switch gears */
    std::vector<Switch> _all_switches;
    
    /** Set of potential battery inverters */
    std::vector<BatteryInverter*> _potential_battery_inverters;
    
    /** Set of existing battery inverters */
    std::vector<BatteryInverter*> _existing_battery_inverters;
    
    /** Set of potential + existing battery inverters */
    std::vector<BatteryInverter*> _battery_inverters;
    
    
    /** Set of potential edge expansions */
    std::vector<Line*> _potential_expansion;
    
    /** Set of potential switch gear investment */
    std::vector<Switch*> _potential_switches;
    
    /** Data on months */
    std::vector<Month> _months_data;
    
    map<int,string> _months_season;
    
    /** Data on wind farm */
    WindData _wind_data;
    
    /** Data on PV gens */
    PVData _PV_data;
    
    
    /** Indices Sets */
    indices hours = indices("hours"); /**< Hours */
    //    indices months = time("jan","feb","mar","apr","may","jun","jul","aug","sep","oct","nov","dec"); /**< Months */
    indices months = time("summer","winter", "autumn"); /**< Months */
    indices years = time("year1"); /**< Years */
    

//    months._name = "months";
    //    indices months = time("jan");
    indices typical_days = time("week","peak","weekend");
//    typical_days._name = "typical_days";
    //    indices typical_days = time("week");
    indices phase_T, T= indices("Time"), Nt = indices("Nt"), Nt_c = indices("Nt_c"), Lt = indices("Lt"), Nt_load, Nt_no_load, Et = indices("Et"),Et_c = indices("Et_c"), Et1 = indices("Et1"), Et2 = indices("Et2"), Et3 = indices("Et3"), Et1_c = indices("Et1_c"), Et2_c = indices("Et2_c"), Et3_c = indices("Et3_c"), pot_G_ph = indices("pot_Gph"), pot_Wind_ph = indices("pot_Wind_ph"), pot_PV_ph = indices("pot_PVph"), pot_E_ph = indices("pot_Eph"), pot_B_ph = indices("pot_Bph"), G_ph= indices("Gph"),Wind_ph= indices("Wph"),PV_ph= indices("PVph"), N_ph= indices("Nph"), exist_G_ph = indices("Exist_Gph"),exist_PV_ph = indices("Exist_PVph"),exist_Wind_ph = indices("Exist_Wind_ph"), exist_B_ph = indices("Exist_Bph"), B_ph = indices("Bph"), exist_E_ph = indices("Exist_Eph"), E_ph = indices("Eph"), E_ph1 = indices("Eph1"),E_ph2 = indices("Eph2"), E_ph3 = indices("Eph3"), E_ph1_c = indices("Eph1_c"),E_ph2_c = indices("Eph2_c"), E_ph3_c = indices("Eph3_c"), Gt = indices("Gt"),Gt_c = indices("Gt_c"), PVt = indices("PVt"), PVt_c = indices("PVt_c"), Windt = indices("Windt"), Windt_c = indices("Windt_c"), exist_Gt= indices("exit_Gt"), exist_PVt= indices("exit_PVt"), exist_Windt= indices("exit_Windt"), exist_Bt= indices("exit_Bt"), exist_Et= indices("exit_Et"), pot_Et= indices("pot_Et"), pot_PVt= indices("pot_PVt"),pot_Windt= indices("pot_Windt"), pot_Gt= indices("pot_Gt"), pot_Bt= indices("pot_Bt"), Bt = indices("Bt"), Bt_c = indices("Bt_c"), Btn = indices("Btn"), Bt1 = indices("Bt1"), Gt1, Wt, PV_pot_t, pot_gen, pot_batt, pot_edges, pot_pv;
    indices Et_opt, Gt_opt, Bt_opt, Bt1_opt, Wt_opt, PVt_opt;
    indices cross_phase = indices("cross_phase");
    indices N_ph1 = indices("Nph1"),N_ph2 = indices("Nph2"), N_ph3 = indices("Nph3");
    
    /** Investment Binary Variables */
    var<int> w_g, w_b, w_e, w_pv, w_wind;
    var<> Pv_cap; /**< Real variable indicating the extra capacity of PV to be installed */
    param<> Pv_cap_; /**< Real variable indicating the extra capacity of PV that has been installed */
    var<> Pg_; /**< Real variable indicating the power generation levels on committed generators */
    var<> theta_ = var<>("𝛉"); /**< Real variable indicating the phase angle on buses */
    param<> vm_s_ = param<>("|V|s"); /**< Real variable indicating the voltage magnitude on buses */
    param<> theta_s_ = param<>("𝛉s"); /**< Real variable indicating the phase angle set point on buses */
    param<> b_fr_ = param<>("b_fr"); /**< parameter storing the imaginary part of line charging on the from side*/
    param<> g_fr_ = param<>("g_fr"); /**< parameter storing the real part of line charging on the from side*/
    param<> b_to_ = param<>("b_to"); /**< parameter storing the imaginary part of line charging on the to side*/
    param<> g_to_ = param<>("g_to"); /**< parameter storing the real part of line charging on the to side*/
    param<> br_r_ = param<>("br_r"); /**< parameter storing the resistance matrix */
    param<> br_x_ = param<>("br_x"); /**< parameter storing the reactance matrix */
    param<> shift_ = param<>("shift"); /**< parameter storing the angle shift on a branch */
    param<> tap_ = param<>("tap"); /**< parameter storing the tap ratio for a transformer */
    param<> Pg_base, Qg_base; /**< Real and Reactive params indicating the power generation levels on committed generators in the base case */
    param<> pg_= param<>("pg"), qg_= param<>("qg"); /**< Real and Reactive params indicating the power generation levels on committed generators in the base case */
    
    param<> pd_= param<>("pd"), qd_= param<>("qd"); /**< Real and Reactive params indicating the power demand levels on buses */
    param<> gs_= param<>("gs"), bs_= param<>("bs"); /**< Real and Reactive params indicating the shunt on a bus */
    param<> Y = param<>("Y");
    param<> Yr11= param<>("Yr11"), Yr12= param<>("Yr12"), Yr13= param<>("Yr13"), Yi11= param<>("Yi11"), Yi12= param<>("Yi12"), Yi13= param<>("Yi13");
    param<> Yr21= param<>("Yr21"), Yr22= param<>("Yr22"), Yr23= param<>("Yr23"), Yi21= param<>("Yi21"), Yi22= param<>("Yi22"), Yi23= param<>("Yi23");
    param<> Yr31= param<>("Yr31"), Yr32= param<>("Yr32"), Yr33= param<>("Yr33"), Yi31= param<>("Yi31"), Yi32= param<>("Yi32"), Yi33= param<>("Yi33");
    
    /** Constructors */
    PowerNet();
    ~PowerNet();
    
    /** Power grid data parser from Matpower*/
    int readgrid(const string& fname, bool reverse_arcs = true);
    
    /** Power grid data parser from GAMS*/
    int readGAMS(const string& fname);
    
    int readODO(const string& fname);
    
    void readJSON(const string& fname);
    
    /** Power grid data parser from DERCAM*/
    int readDERCAM(const string& fname);
    
    
    /** Use the time series data to compute averages for typical days */
    void compute_loads();
    
    PowerNet* clone(int net_id) const; /**< clone the subnetwork correpsonding to net_id */
        
    /** Accessors */
    string get_ref_bus();
    unsigned get_nb_active_gens() const;
    unsigned get_nb_active_bus_pairs() const;
    unsigned get_nb_active_arcs() const;
    unsigned get_nb_active_nodes() const;
    void time_expand(unsigned T); /* < Time expansion of the grid parameters */
    void time_expand(const indices& T); /* < Time expansion of the grid parameters */
    void update_net();
    
    void save_base_case_sol(const string& fname);
    void save_all_sol(const string& fname);
    
    /** get set indexed by bus pairs in the chordal extension */
    gravity::indices get_bus_pairs_chord();

    indices get_conting_buses(const map<string,shared_ptr<Scenario>>& conts) const;
    indices get_conting_arcs(const map<string,shared_ptr<Scenario>>& conts) const;
    indices get_conting_gens(const map<string,shared_ptr<Scenario>>& conts) const;
    indices get_conting_arcs_pot(const map<string,shared_ptr<Scenario>>& conts) const;
    indices get_conting_gens_pot(const map<string,shared_ptr<Scenario>>& conts) const;
    indices get_conting_arcs_exist(const map<string,shared_ptr<Scenario>>& conts) const;
    
    indices get_phase(const indices& set, int ph) const;
    
    /** Power Model<>s */
    
    vector<shared_ptr<Model<>>> conting_mods;/* < Contingency Model<>s */
    
    void update_status(unique_ptr<Model<>> Model);
    
    void fix_investment();
    
    shared_ptr<Model<>> build_ODO_model(PowerModelType model=LDISTF, int output=5, double tol=1e-6, int nb_hours = 24, bool networked=false);
    shared_ptr<Model<>> build_ODO_model_polar(int output=5, double tol=1e-6, int nb_hours = 24, bool networked=false);
    
    unique_ptr<Model<>> build_ACOPF_N_1(PowerModelType Model=ACPOL, int output=0, double tol=1e-6);
    
    unique_ptr<Model<>> build_SOCP_OPF_N_1(PowerModelType Model=ACPOL, int output=0, double tol=1e-6, bool sdp_cuts = false);
    
    unique_ptr<Model<>> build_SOCP_OPF_MINLP(PowerModelType Model=SOCP, int output=0, double tol=1e-6);
    
    unique_ptr<Model<>> build_ACOPF_MINLP(PowerModelType Model, int output, double tol,const vector<bool>& cont_in);
    
    unique_ptr<Model<>> build_soft_ACOPF_N_1(PowerModelType Model=ACPOL, int output=0, double tol=1e-6, double obj_pen=1e10);
    
    vector<param<>> signs();
    
    indices get_all_conting(const map<string,shared_ptr<Scenario>>& scenarios) const;
    indices get_time_ids_conting(const map<string,shared_ptr<Scenario>>& scenarios) const;
    
    
    indices fixed_from_branch_phase(unsigned ph, bool contingency = false) const;
    indices fixed_to_branch_phase(unsigned ph, bool contingency = false) const;
    indices from_branch_phase(unsigned ph, bool contingency = false) const;
    indices to_branch_phase(unsigned ph, bool contingency = false) const;    
    indices get_branch_phase(unsigned ph, bool contingency = false) const;
    indices get_branch_id_phase(unsigned ph, bool contingency = false) const;
    
    indices gens_per_node_time() const;
    indices PV_per_node_time() const;
    indices Batt_per_node_time() const;
    indices Load_per_node_time() const;
    indices Wind_per_node_time() const;
    indices out_arcs_per_node_time() const;
    indices in_arcs_per_node_time() const;
    
    indices gens_per_node_time_cont(const map<string,shared_ptr<Scenario>>& scenarios) const;
    indices PV_per_node_time_cont() const;
    indices Batt_per_node_time_cont() const;
    indices Wind_per_node_time_cont() const;
    indices out_arcs_per_node_time_cont(const map<string,shared_ptr<Scenario>>& scenarios) const;
    indices in_arcs_per_node_time_cont(const map<string,shared_ptr<Scenario>>& scenarios) const;
    
    indices gens_per_node() const;
    indices out_arcs_per_node() const;
    indices in_arcs_per_node() const;
    
    unique_ptr<Model<>> build_fixed_ACOPF_N_1(PowerModelType Model, int output, double tol, double obj_pen, const vector<indices>& ids_p, const vector<indices>& ids_n);
    double solve_acopf(PowerModelType Model=ACPOL, int output=0, double tol=1e-6);
    unique_ptr<Model<>> build_ROMDST(PowerModelType Model=LDISTF, int output=5, double tol=1e-6, int nb_hours = 24);
    shared_ptr<Model<>> build_SCOPF(PowerModelType Model=LDISTF, int output=5, double tol=1e-6);
    shared_ptr<Model<>> build_ROMDST_contingency(const string& name, PowerModelType Model=LDISTF, int output=5, double tol=1e-6, int nb_hours = 24);
    shared_ptr<Model<>> build_SCOPF_gen_contingency(int cont, const string& name, PowerModelType Model=ACPOL, int output=5, double tol=1e-6);
    shared_ptr<Model<>> build_SCOPF_line_contingency(int cont, const string& name, PowerModelType Model=ACPOL, int output=5, double tol=1e-6);
    
    void fill_wbnds();
    
    vector<shared_ptr<PowerNet>> get_separate_microgrids() const;
};

shared_ptr<Model<>> build_ACOPF(PowerNet& grid, PowerModelType Model=ACPOL, int output=0, double tol=1e-6);

#endif
