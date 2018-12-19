//
//  PowerNet.cpp
//
//

#include "PowerNet.h"
#include <algorithm>
#include <map>
#define _USE_MATH_DEFINES
#include <cmath>
#include <list>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <queue>
#include <time.h>
#include <xlnt/xlnt.hpp>
#include <gravity/solver.h>
#include <rapidjson/document.h>
#include <rapidjson/reader.h>
#include <rapidjson/istreamwrapper.h>
//#define USEDEBUG
#ifdef USEDEBUG
#define DebugOn(x) cout << x
#else
#define Debug(x)
#endif
#define DebugOff(x)

using namespace std;
using namespace rapidjson;

PowerNet::PowerNet() {
    bMVA = 0;
    pg_min.set_name("pg_min");
    pg_max.set_name("pg_max");
    qg_min.set_name("qg_min");
    qg_max.set_name("qg_max");
    pb_min.set_name("pb_min");
    pb_max.set_name("pb_max");
    qb_min.set_name("qb_min");
    qb_max.set_name("qb_max");
    pv_min.set_name("pv_min");
    pv_max.set_name("pv_max");
    qv_min.set_name("qv_min");
    qv_max.set_name("qv_max");
    pw_min.set_name("pw_min");
    pw_max.set_name("pw_max");
    qw_min.set_name("qw_min");
    qw_max.set_name("qw_max");
    pv_out.set_name("pv_out");
    pv_capcost.set_name("pv_capcost");
    pv_varcost.set_name("pv_varcost");
    pg_s.set_name("pg_s");
    qg_s.set_name("qg_s");
    cb_f.set_name("cb_f");
    cb_v.set_name("cb_v");
    c0.set_name("c0");
    c1.set_name("c1");
    c2.set_name("c2");
    ramp_up.set_name("ramp_up");
    ramp_down.set_name("ramp_down");
    gen_eff.set_name("gen_eff");
    min_ut.set_name("min_ut");
    min_dt.set_name("min_dt");
    min_diesel_invest.set_name("min_diesel_invest");
    max_diesel_invest.set_name("max_diesel_invest");
    min_batt_invest.set_name("min_batt_invest");
    max_batt_invest.set_name("max_batt_invest");
    gen_capcost.set_name("cg");
    expansion_capcost.set_name("ce");
    inverter_capcost.set_name("cb");
    th_min.set_name("th_min");
    th_max.set_name("th_max");
    cphi.set_name("cphi");
    sphi.set_name("sphi");
    cos_d.set_name("cos_d");
    tan_th_min.set_name("tan_th_min");
    tan_th_max.set_name("tan_th_max");
    v_diff_max.set_name("v_diff_max");
    v_min.set_name("v_min");
    v_max.set_name("v_max");
    w_min.set_name("w_min");
    w_max.set_name("w_max");
    wr_min.set_name("wr_min");
    wr_max.set_name("wr_max");
    wi_min.set_name("wi_min");
    wi_max.set_name("wi_max");
    v_s.set_name("v_s");
    pl.set_name("pl");
    pl_ratio.set_name("pl_ratio");
    ql.set_name("ql");
    gs.set_name("gs");
    bs.set_name("bs");
    g.set_name("g");
    b.set_name("b");
    r.set_name("r");
    x.set_name("x");
    ch.set_name("ch");
    as.set_name("as");
    tr.set_name("tr");
    S_max.set_name("S_max");
    eff_a.set_name("eff_a");
    eff_b.set_name("eff_b");
    g_ff.set_name("g_ff");
    g_ft.set_name("g_ft");
    g_tf.set_name("g_tf");
    g_tt.set_name("g_tt");

    b_ff.set_name("b_ff");
    b_ft.set_name("b_ft");
    b_tf.set_name("b_tf");
    b_tt.set_name("b_tt");
    Y.set_name("Y");
    Y_t.set_name("Y_t");
    Y_charge.set_name("Y_charge");
    Y_charge_t.set_name("Y_charge_t");
}

PowerNet::~PowerNet() {
    if(!gens.empty()) {
        for (Gen* g:gens) {
            delete g;
        }
        gens.clear();
    }
    for (Node* n:nodes) {
        delete n;
    }
    nodes.clear();
    for (Arc* a:arcs) {
        delete a;
    }
    arcs.clear();
}
//
// Read a grid
// @discussion line with delimiter ";"

string PowerNet::get_ref_bus() {
    return ref_bus;
}

unsigned PowerNet::get_nb_active_gens() const {
    unsigned nb=0;
    for (auto g: gens) {
        if (g->_active) {
            nb++;
        }
    }
    return nb;
}

unsigned PowerNet::get_nb_active_bus_pairs() const {
    unsigned nb=0;
    for (auto bp: _bus_pairs._keys) {
//        if (bp->_active) {
            nb++;
//        }
    }
    return nb;
}


unsigned PowerNet::get_nb_active_arcs() const {
    unsigned nb=0;
    for (auto a: arcs) {
        if (a->_active) {
            nb++;
        }
    }
    return nb;
}

unsigned PowerNet::get_nb_active_nodes() const {
    unsigned nb=0;
    for (auto n: nodes) {
        if (n->_active) {
            nb++;
        }
        else {
            DebugOff("Inactive Node" << n->_name << endl);
        }
    }
    return nb;
}

void PowerNet::time_expand(const indices& T) {
//    c0.time_expand(T);
//    c1.time_expand(T);
//    c2.time_expand(T);
    pl._time_extended = true;
    pl_ratio._time_extended = true;
    ql._time_extended = true;
    pv_out._time_extended = true;
    pw_min._time_extended = true;
    pw_max._time_extended = true;
//    S_max.time_expand(T);
//    th_min.time_expand(T);
//    th_max.time_expand(T);
//    tan_th_min.time_expand(T);
//    tan_th_max.time_expand(T);
//    r.time_expand(T);
//    x.time_expand(T);
//    pg_min.time_expand(T);
//    pg_max.time_expand(T);
//    pv_min.time_expand(T);
//    pv_max.time_expand(T);
//    qg_min.time_expand(T);
//    qg_max.time_expand(T);
//    pb_min.time_expand(T);
//    pb_max.time_expand(T);
//    qb_min.time_expand(T);
//    qb_max.time_expand(T);
//    w_min.time_expand(T);
//    w_max.time_expand(T);
//    v_min.time_expand(T);
//    v_max.time_expand(T);
//    eff_a.time_expand(T);
//    eff_b.time_expand(T);
}

void PowerNet::time_expand(unsigned T) {
    c0.time_expand(T);
    c1.time_expand(T);
    c2.time_expand(T);
    S_max.time_expand(T);
    th_min.time_expand(T);
    th_max.time_expand(T);
    tan_th_min.time_expand(T);
    tan_th_max.time_expand(T);
    g_tt.time_expand(T);
    g_ff.time_expand(T);
    g_ft.time_expand(T);
    g_tf.time_expand(T);
    b_tt.time_expand(T);
    b_ff.time_expand(T);
    b_ft.time_expand(T);
    b_tf.time_expand(T);
    pg_min.time_expand(T);
    pg_max.time_expand(T);
    qg_min.time_expand(T);
    qg_max.time_expand(T);
    w_min.time_expand(T);
    w_max.time_expand(T);
    gs.time_expand(T);
    bs.time_expand(T);
    pl.time_expand(T);
    ql.time_expand(T);
    v_min.time_expand(T);
    v_max.time_expand(T);
}


void PowerNet::readJSON(const string& fname){
    ifstream ifs(fname);
    if ( !ifs.is_open() )
    {
        throw invalid_argument("Cannot open file: " + fname);
    }
    
    IStreamWrapper isw { ifs };
    Document d;
    if (d.ParseStream( isw ).HasParseError()) {
        throw invalid_argument("Parse error while reading JSON file: " + fname);
    }
    DebugOn("Successfuly parsed JSON file: " << fname << endl);
    
    bMVA = d["baseMVA"].GetDouble();
    bV = d["baseKV"].GetDouble();
    Value& buses = d["bus"];
    nb_nodes = buses.Size();
    DebugOn("nb_nodes = " << nb_nodes << endl);
    nodes.resize(nb_nodes);
    for (auto& v : buses.GetObject()) {
        Value& list = v.value;
        auto btype = list["bus_type"].GetInt();
        Debug("btype = " << btype << endl);
        auto index = list["index"].GetInt();
        auto name = string(list["name"].GetString());
        auto status =  list["status"].GetInt();
        auto va =  list["va"].GetArray();
        auto vm =  list["vm"].GetArray();
        auto vmax =  list["vmax"].GetArray();
        auto vmin =  list["vmin"].GetArray();
        auto bus = new Bus(to_string(index));
        bus->vbound.min = vmin[0].GetDouble();
        bus->vbound.max = vmax[0].GetDouble();
        vm_.set_val(bus->_name+",ph0", vm[0].GetDouble());
        vm_.set_val(bus->_name+",ph1", vm[1].GetDouble());
        vm_.set_val(bus->_name+",ph2", vm[2].GetDouble());
        for(auto i = 0;i<3;i++){
            if(vm[i].GetDouble()!=0){
                v_min.set_val(bus->_name+",ph"+to_string(i), vmin[i].GetDouble());
            }
            else {
                v_min.set_val(bus->_name+",ph"+to_string(i), -1);
            }
        }
        v_max.set_val(bus->_name, vmax[0].GetDouble());

        theta_.set_val(bus->_name+",ph0", va[0].GetDouble());
        theta_.set_val(bus->_name+",ph1", va[1].GetDouble());
        theta_.set_val(bus->_name+",ph2", va[2].GetDouble());
        bus->_active = status;
        bus->_id = index-1;
        bus->_type = btype;
        if (btype==3) {
            ref_bus = bus->_name;
        }
        assert(bus->_id<nb_nodes);
        if (!nodeID.insert(pair<string,Node*>(bus->_name, bus)).second) {
            throw invalid_argument("ERROR: adding the same bus twice!");
        }
        nodes[bus->_id] = bus;
        if (!bus->_active) {
            DebugOn("INACTIVE NODE: " << name << endl);
        }
    }
    Value& branches = d["branch"];
    nb_branches = branches.Size();
    DebugOn("nb_branches = " << nb_branches << endl);
    for (auto& a : branches.GetObject()) {
        Value& list = a.value;
        auto index = list["index"].GetInt();
        auto name = string(list["name"].GetString());
        auto status =  list["status"].GetInt();
        auto length = list["length"].GetDouble();
        auto angmin =  list["angmin"].GetArray();
        auto angmax =  list["angmax"].GetArray();
        auto b_fr = list["b_fr"].GetArray();
        auto f_bus = list["f_bus"].GetInt();
        auto t_bus = list["t_bus"].GetInt();
        auto b_to = list["b_to"].GetArray();
        auto g_fr = list["g_fr"].GetArray();
        auto g_to = list["g_to"].GetArray();
        auto br_r = list["br_r"].GetArray();
        auto br_x = list["br_x"].GetArray();
        auto rating = list["current_rating_a"].GetArray();
        auto shift = list["shift"].GetArray();
        auto tap = list["tap"].GetArray();
        auto tr = list["transformer"].GetBool();

        auto arc = new Line(to_string(index) + "," + to_string(f_bus)+","+to_string(t_bus)); // Name of lines
        arc->_id = index-1;
        arc->_src = nodes[f_bus-1];
        arc->_dest= nodes[t_bus-1];
        arc->status = status;
        arc->_len = length;
        arc->_is_transformer = tr;
        arc->connect();
        add_arc(arc);
        _exist_arcs.push_back(arc);
        for (auto i = 0; i<3; i++) {
            auto src = arc->_src->_name;
            auto dest = arc->_dest->_name;
            if (vm_.eval(src+",ph"+to_string(i))>0 && vm_.eval(dest+",ph"+to_string(i))>0) {
                Et_ph.add(arc->_name+",ph"+to_string(i));
                arc->_phases.insert(i);
            }
            else {
                DebugOn("excluding arc: " << arc->_name << " on phase " << i << endl);
            }
            r.set_val(arc->_name+",ph"+to_string(i), br_r[i][i].GetDouble()/bMVA);
            x.set_val(arc->_name+",ph"+to_string(i), br_x[i][i].GetDouble()/bMVA);
            for (auto j = 0; j<3; j++) {
                br_r_.set_val(arc->_name+","+to_string(i+1)+","+to_string(j+1), br_r[i][j].GetDouble()/bMVA);
                br_x_.set_val(arc->_name+","+to_string(i+1)+","+to_string(j+1), br_x[i][j].GetDouble()/bMVA);
            }
            b_fr_.set_val(arc->_name+",ph"+to_string(i), b_fr[i].GetDouble()/bMVA);
            b_to_.set_val(arc->_name+",ph"+to_string(i), b_to[i].GetDouble()/bMVA);
            g_fr_.set_val(arc->_name+",ph"+to_string(i), g_fr[i].GetDouble()/bMVA);
            g_to_.set_val(arc->_name+",ph"+to_string(i), g_to[i].GetDouble()/bMVA);
            shift_.set_val(arc->_name+",ph"+to_string(i), shift[i].GetDouble());
            tap_.set_val(arc->_name+",ph"+to_string(i), tap[i].GetDouble());
            this->S_max.set_val(arc->_name+",ph"+to_string(i), rating[i].GetDouble());
        }
        
    }
    
    Value& generatos = d["generator"];
    nb_gens = generatos.Size();
    DebugOn("nb_gens = " << nb_gens << endl);
    for (auto& g : generatos.GetObject()) {
        Value& list = g.value;
        auto gbus = list["gen_bus"].GetInt();
        Debug("bus = " << gbus << endl);
        auto index = list["index"].GetInt();
        auto name = string(list["name"].GetString());
        auto status =  list["status"].GetInt();
        auto pg =  list["pg"].GetArray();
        auto qg =  list["qg"].GetArray();
        auto qgmax =  list["qmax"].GetArray();
        auto qgmin =  list["qmin"].GetArray();
        auto pgmax =  list["pmax"].GetArray();
        auto pgmin =  list["pmin"].GetArray();
        auto bus = (Bus*)nodes[gbus-1];
        bus->_has_gen = true;
        name = "Existing_Gen," + bus->_name + "," + "slot"+to_string(index-1);
        Gen* gen = new Gen(bus, name, 0, 0, 0, 0);
        gen->_id = index-1;
        gen->_active = status;
        gens.push_back(gen);
        bus->_gen.push_back(gen);
        if(!bus->_active) {
            DebugOff("INACTIVE GENERATOR GIVEN INACTIVE BUS: " << gen->_name << endl);
            gen->_active = false;
        }
        else if (!gen->_active) {
            DebugOn("INACTIVE GENERATOR: " << name << endl);
        }
        pg_.set_val(gen->_name+",ph0"+",jan,week,1", pg[0].GetDouble());
        pg_.set_val(gen->_name+",ph1"+",jan,week,1", pg[1].GetDouble());
        pg_.set_val(gen->_name+",ph2"+",jan,week,1", pg[2].GetDouble());
        qg_.set_val(gen->_name+",ph0"+",jan,week,1", qg[0].GetDouble());
        qg_.set_val(gen->_name+",ph1"+",jan,week,1", qg[1].GetDouble());
        qg_.set_val(gen->_name+",ph2"+",jan,week,1", qg[2].GetDouble());
        if(bus->_type==3){
            pg_min.set_val(gen->_name, numeric_limits<double>::lowest());
            pg_max.set_val(gen->_name, numeric_limits<double>::max());
            qg_min.set_val(gen->_name, numeric_limits<double>::lowest());
            qg_max.set_val(gen->_name, numeric_limits<double>::max());
        }
        else {
            pg_min.set_val(gen->_name, pgmin[0].GetDouble());
            pg_max.set_val(gen->_name, pgmax[0].GetDouble());
            qg_min.set_val(gen->_name, qgmin[0].GetDouble());
            qg_max.set_val(gen->_name, qgmax[0].GetDouble());
        }
    }
    Value& loads = d["load"];
    for (auto& l : loads.GetObject()) {
        Value& list = l.value;
        auto lbus = list["load_bus"].GetInt();
        Debug("bus = " << lbus << endl);
        auto status =  list["status"].GetInt();
        auto pd =  list["pd"].GetArray();
        auto qd =  list["qd"].GetArray();
        auto bus = (Bus*)nodes[lbus-1];
        auto name = bus->_name;
        for(auto i=0; i<3; i++){
            bus->_cond[i]->_pl = pd[i].GetDouble();
            bus->_cond[i]->_ql = qd[i].GetDouble();
            if(status){
                pl.set_val(name+",ph"+to_string(i)+",jan,week,1", pd[i].GetDouble());
                ql.set_val(name+",ph"+to_string(i)+",jan,week,1", qd[i].GetDouble());
            }
            else {
                pl.set_val(name+",ph"+to_string(i)+",jan,week,1", 0);
                ql.set_val(name+",ph"+to_string(i)+",jan,week,1", 0);
            }
        }
    }
    Value& shunt = d["shunt"];
    for (auto& sh : shunt.GetObject()) {
        Value& list = sh.value;
        auto bus_id = list["shunt_bus"].GetInt();
        Debug("bus = " << lbus << endl);
        auto status =  list["status"].GetInt();
        auto bs = list["bs"].GetArray();
        auto gs = list["gs"].GetArray();
        auto bus = (Bus*)nodes[bus_id-1];
        for (auto i = 0; i<3; i++) {
            bus->_cond[i]->_bs = bs[i].GetDouble();
            bus->_cond[i]->_gs = gs[i].GetDouble();
            gs_.set_val(bus->_name+",ph"+to_string(i), bus->_cond[i]->_gs/pow(bMVA,0));
            bs_.set_val(bus->_name+",ph"+to_string(i), bus->_cond[i]->_bs/pow(bMVA,2));
//            gs_.set_val(bus->_name+",ph"+to_string(i), 0);
//            bs_.set_val(bus->_name+",ph"+to_string(i), 0);
        }
        
    }
}

int PowerNet::readODO(const string& fname){
    size_t index = 0;
    string name;
    xlnt::workbook wb;
    double wall0 = get_wall_time();
    clog << "Opening excel file...\n";
    wb.load(fname);
    double wall1 = get_wall_time();
    clog << "Done.\n";
    clog << "Wall clock computing time =  " << wall1 - wall0 << "\n";
    xlnt::worksheet ws;
    bool RunSettings = true;
    try{
        ws = wb.sheet_by_title("RunSettings");
    }
    catch(xlnt::key_not_found err) {
        RunSettings = false;
        cerr << "Cannot find sheet RunSettings, ignoring RunSettings options" << endl;
    }
    if (RunSettings) {
        auto row_it = ws.rows().begin();
        auto row = *row_it++;
        _max_time = row[1].value<int>();
        row = *row_it++;
        _max_it = row[1].value<int>();
        row = *row_it++;
        _tol = row[1].value<double>();
        row = *row_it++;
        _nb_years = row[1].value<int>();
        row = *row_it++;
        _inflation_rate = row[1].value<double>();
        row = *row_it++;
        _demand_growth = row[1].value<double>();
    }
    
    ws = wb.sheet_by_title("CableParams");
    clog << "Processing CableParams for branch expansion costs and line properties" << std::endl;
    auto row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    vector<pair<double,double>> rx; /* Vector of resistance,reactance */
    vector<double> cost; /* Vector of expansion costs */
    vector<double> Smax; /* Vector of thermal limits */
    while (row_it!=ws.rows().end()) {
        auto row = *row_it++;
        rx.push_back(make_pair(row[2].value<double>(), row[3].value<double>()));
        cost.push_back(row[2].value<double>());
        Smax.push_back(row[5].value<double>()/bMVA);
    }
    
    bool found_branch_expansion = true;
    try{
        ws = wb.sheet_by_title("BranchInvest");
    }
    catch(xlnt::key_not_found err) {
        found_branch_expansion = false;
        cerr << "Cannot find sheet BranchInvest, ignoring branch expansion options" << endl;
    }
    if (found_branch_expansion) {
        index = this->arcs.size();
        clog << "Processing BranchInvest" << std::endl;
        auto row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (size_t i = 0; i<nb_nodes-1; i++) {
            auto row = *row_it++;
            for (size_t j = i+1; j<nb_nodes; j++) {
                if (row[j].has_value()) {
                    string list = row[j].to_string();
                    size_t sz = 0;
                    int b_id = 0;
                    b_id = stoi(list, &sz);
                    if (list.size()==1 && b_id==0) {
                        continue;
                    }
                    while (true) {
                        auto src = to_string(i+1);
                        auto dest = to_string(j);
                        DebugOn("Expansion possible on arc (" << i+1 << "," << j << ") with type " << b_id << endl);
                        auto arc = new Line(to_string(index) + "," + src + "," + dest); // Name of lines
                        arc->_expansion = true;
                        arc->_active = false;
                        arc->b_type = b_id;
                        arc->r = rx[index].first;
                        arc->x = rx[index].second;
                        arc->smax = Smax[index];
                        arc->cost = cost[index];
                        arc->_id = index++;
                        arc->_src = get_node(src);
                        arc->_dest= get_node(dest);
                        arc->connect();
                        add_arc(arc);
                        _potential_expansion.push_back(arc);
                        if (list.size()<= sz) {
                            break;
                        }
                        list = list.substr(sz+1);
                        b_id = stoi(list, &sz);
                    }
                }
            }
        }
    }
    
    bool found_SwitchParams = true;
    try{
        ws = wb.sheet_by_title("SwitchParams");
    }
    catch(xlnt::key_not_found err) {
        found_SwitchParams = false;
        cerr << "Cannot find sheet SwitchParams, using default switch gear parameters (Capital cost: capital cost $80,000, remote controlled)   " << endl;
    }
    if(found_SwitchParams){
        size_t idx = 0;
        clog << "Processing SwitchParams (Switch gear capital cost and operation type)" << std::endl;
        row_it = ws.rows().begin();
        while (row_it!=ws.rows().end()) {
            auto row = *row_it++;
            _all_switches.push_back(Switch("Switch"+to_string(idx++),row[2].value<double>(), row[3].value<bool>(), row[4].value<bool>()));
            _all_switches.back().print();
        }
    }
    
    bool found_switch_invest = true;
    try{
        ws = wb.sheet_by_title("SwitchInvest");
    }
    catch(xlnt::key_not_found err) {
        found_switch_invest = false;
        cerr << "Cannot find sheet SwitchInvest, ignoring switch gear investment options" << endl;
    }
    if (found_switch_invest) {
        index = this->arcs.size();
        clog << "Processing SwitchInvest" << std::endl;
        auto row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (size_t i = 0; i<nb_nodes-1; i++) {
            auto row = *row_it++;
            for (size_t j = i+1; j<nb_nodes; j++) {
                if (row[j].has_value()) {
                    string list = row[j].to_string();
                    size_t sz = 0;
                    int s_id = 0;
                    s_id = stoi(list, &sz);
                    if (list.size()==1 && s_id==0) {
                        continue;
                    }
                    while (true) {
                        auto src = to_string(i+1);
                        auto dest = to_string(j);
                        DebugOn("Switch possible on branch (" << i+1 << "," << j << ") with type " << s_id << endl);
                        auto a = (Line*)get_arc(src,dest);
                        a->_switches.push_back(_all_switches[s_id]);
                        _potential_switches.push_back(&_all_switches[s_id]);
                        if (list.size()<= sz) {
                            break;
                        }
                        list = list.substr(sz+1);
                        s_id = stoi(list, &sz);
                    }
                }
            }
        }
    }
    
    ws = wb.sheet_by_title("CableLen");
    clog << "Processing CableLen" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    index = 0;
    double len = 0;
    for (size_t i = 0; i<nb_nodes-1; i++) {
        auto row = *row_it++;
        for (size_t j = i+1; j<nb_nodes; j++) {
            if (row[j].has_value() && row[j].value<unsigned>()>0) {
                auto src = to_string(i+1);
                auto dest = to_string(j);
                len = row[j].value<unsigned>();
                auto a = get_arc(src, dest);
                if(a){
                    DebugOn("(" << i+1 << "," << j << ") has len " << len << endl);
                    a->_len = len;
                }
            }
        }
    }
    
    
    unsigned exist = 0, min_bat = 0, max_bat = 0, min_d = 0, max_d = 0, age = 0;
    double min_PV = 0, max_PV = 0;
    double min_Wind = 0, max_Wind = 0;
    
    bool found_PvWindInvest = true;
    try{
        ws = wb.sheet_by_title("PvWindInvest");
    }
    catch(xlnt::key_not_found err) {
        found_PvWindInvest = false;
        cerr << "Cannot find sheet PvWindInvest, ignoring investments in renewable generation" << endl;
    }
    if(found_PvWindInvest){
        clog << "Processing PvWindInvest (Pv and Wind investment options at each node)" << std::endl;
        row_it = ws.rows().begin();
        row_it++;
        row_it++;//SKIP FIRST ROW
        auto row = *row_it;
        for (auto n: nodes) {
            auto bus = (Bus*)n;
            min_PV = row[1].value<double>();
            max_PV = row[2].value<double>();
            exist = row[3].value<double>();
            bus->_min_PV_cap = min_PV;
            bus->_max_PV_cap = max_PV;
            bus->_existing_PV_cap = exist;
            if (max_PV>0 || exist >0) {
                DebugOn("min PV cap at bus" << bus->_name << " = " << min_PV << endl);
                DebugOn("max PV cap at bus" << bus->_name << " = " << max_PV << endl);
                DebugOn("existing PV cap at bus" << bus->_name << " = " << exist << endl);
            }
            row_it++;
            row = *row_it;
        }
        assert(row[0].value<string>()=="Wind");
        row_it++;
        row_it++;//SKIP FIRST ROW
        row = *row_it;
        for (auto n: nodes) {
            auto bus = (Bus*)n;
            min_Wind = row[1].value<double>();
            max_Wind = row[2].value<double>();
            exist = row[3].value<double>();
            bus->_min_Wind_cap = min_Wind;
            bus->_max_Wind_cap = max_Wind;
            bus->_existing_Wind_cap = exist;
            if(max_Wind>0 || exist >0){
                DebugOn("min Wind cap at bus" << bus->_name << " = " << min_Wind << endl);
                DebugOn("max Wind cap at bus" << bus->_name << " = " << max_Wind << endl);
                DebugOn("existing Wind cap at bus" << bus->_name << " = " << exist << endl);
            }
            row_it++;
            row = *row_it;
        }
    }
    
    bool found_PvWindParams = true;
    try{
        ws = wb.sheet_by_title("PvWindParams");
    }
    catch(xlnt::key_not_found err) {
        found_PvWindParams = false;
        cerr << "Cannot find sheet PvWindParams, using default PV nd Wind parameters (Capital cost: 2500 $/kW, 80% efficiency)" << endl;
    }
    if(found_PvWindParams){
        clog << "Processing PvWindParams (Pv and Wind capital cost and efficiency)" << std::endl;
        row_it = ws.rows().begin();
        auto row = *row_it;
        _pv_cap_cost = row[1].value<double>();
        row_it++;
        row = *row_it;
        _pv_eff = row[1].value<double>();
        DebugOn("PV cap cost = " << _pv_cap_cost << endl);
        DebugOn("PV eff = " << _pv_eff << endl);
        row_it++;
        row = *row_it;
        _wind_cap_cost = row[1].value<double>();
        row_it++;
        row = *row_it;
        _wind_eff = row[1].value<double>();
        DebugOn("Wind cap cost = " << _wind_cap_cost << endl);
        DebugOn("Wind eff = " << _wind_eff << endl);
    }
    
    
    
    ws = wb.sheet_by_title("DieselParams");
    clog << "Processing DieselParams" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    
    unsigned lifetime,type,idx = 1;
    auto first_row = *(ws.rows().begin());
    while (row_it!=ws.rows().end()) {
        double max_p = 0, max_s =  0, capcost = 0, c0 = 0,c1 = 0,c2 = 0,eff = 0,max_ramp_up = 0,max_ramp_down = 0,min_up_time = 0,min_down_time = 0;
        auto row = *row_it++;
        max_s = row[2].value<double>()/bMVA;
        max_p = max_s;
        capcost = row[3].value<double>();
        c0 = row[4].value<double>();
        c1 = row[5].value<double>()*bMVA;
        c2 = row[6].value<double>()*pow(bMVA,2);
        lifetime = row[11].value<int>();
        for (auto i = 7; i<first_row.length(); i++) {
            if (first_row[i].to_string().compare("Type")==0) {
                type = row[i].value<int>();
            }
            if (first_row[i].to_string().compare("efficiency")==0) {
                eff = row[i].value<double>();
            }
            if (first_row[i].to_string().compare("MaxRampUp")==0) {
                max_ramp_up = row[i].value<double>();
            }
            if (first_row[i].to_string().compare("MaxRampDown")==0) {
                max_ramp_down = row[i].value<double>();
            }
            if (first_row[i].to_string().compare("MinDownTime")==0) {
                min_down_time = row[i].value<int>();
            }
            if (first_row[i].to_string().compare("MinUpTime")==0) {
                min_up_time = row[i].value<int>();
            }
        }
        _all_diesel_gens.push_back(DieselGen("DG"+to_string(idx++), max_p, max_s, lifetime, capcost, c0, c1, c2, type, eff, max_ramp_down, max_ramp_up,min_down_time,min_up_time));
        _all_diesel_gens.back().print();
    }
    
    /* Diesel Investment Options */
    bool found_diesel_invest = true;
    try{
        ws = wb.sheet_by_title("DieselInvest");
    }
    catch(xlnt::key_not_found err) {
        found_diesel_invest = false;
        cerr << "Cannot find sheet DieselInvest, ignoring diesel generation investment" << endl;
    }
    if(found_diesel_invest){
        clog << "Processing DieselInvest (Diesel investment options at each node)" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (auto n: nodes) {
            auto bus = (Bus*)n;
            unsigned index = 0;
            auto row = *row_it++;
            while (row[0].to_string().compare((*row_it)[0].to_string())==0) {
                max_d = row[6].value<int>();
                if (max_d>0) {
                    min_d = row[5].value<int>();
                    exist = row[7].value<int>();
                    assert(max_d>=min_d && max_d>=exist);
                    age = row[8].value<int>();
                    bus->_diesel_data[index] = DieselData(min_d,max_d,exist,age);
                    auto copy = _all_diesel_gens[index];
                    for (auto i = 0; i<exist; i++) {
                        auto name = copy._name + "," + bus->_name + "," + "slot"+to_string(i);
                        auto gen = new Gen(bus, name, 0, copy._max_p, -copy._max_s, copy._max_s);
                        gen->set_costs(copy._c0, copy._c1, copy._c2);
                        _existing_diesel_gens.push_back(gen);
                        gens.push_back(gen);
                        bus->_gen.push_back(gen);
                        this->c0.set_val(name,copy._c0);
                        this->c1.set_val(name,copy._c1);
                        this->c2.set_val(name,copy._c2);
                        this->pg_min.set_val(name,0);
                        this->pg_max.set_val(name,copy._max_p);
                        this->qg_min.set_val(name,-copy._max_s);
                        this->qg_max.set_val(name,copy._max_s);
                        this->min_dt.set_val(name,copy._min_down_time);
                        this->min_ut.set_val(name,copy._min_up_time);
                        this->ramp_up.set_val(name,copy._max_ramp_up);
                        this->ramp_down.set_val(name,copy._max_ramp_down);
                        this->gen_eff.set_val(name,copy._eff);
                    }
                    for (auto i = exist; i<exist+max_d; i++) {
                        auto copy = _all_diesel_gens[index];
                        auto name = copy._name + "," + bus->_name + "," + "slot"+to_string(i);
                        auto gen = new Gen(bus, name, 0, copy._max_p, -copy._max_s, copy._max_s);
                        gen->set_costs(copy._c0, copy._c1, copy._c2);
                        _potential_diesel_gens.push_back(gen);
                        gen->_gen_type = index+1;
                        gens.push_back(gen);
                        bus->_gen.push_back(gen);
                        bus->_pot_gen.push_back(gen);
                        this->min_diesel_invest.set_val(name, min_d);
                        this->max_diesel_invest.set_val(name, max_d);
                        this->c0.set_val(name,copy._c0);
                        this->c1.set_val(name,copy._c1);
                        this->c2.set_val(name,copy._c2);
                        this->pg_min.set_val(name,0);
                        this->pg_max.set_val(name,copy._max_p);
                        this->qg_min.set_val(name,-copy._max_s);
                        this->qg_max.set_val(name,copy._max_s);
                        this->gen_capcost.set_val(name,copy._capcost);
                        this->min_dt.set_val(name,copy._min_down_time);
                        this->min_ut.set_val(name,copy._min_up_time);
                        this->ramp_up.set_val(name,copy._max_ramp_up);
                        this->ramp_down.set_val(name,copy._max_ramp_down);
                        this->gen_eff.set_val(name,copy._eff);
                    }
                    
                }
                row = *row_it++;
                index++;
            }
        }
    }
    
    ws = wb.sheet_by_title("BattParams");
    clog << "Processing BattParams (Battery Properties)" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    vector<double> x_eff;
    vector<double> y_eff;
    idx = 1;
    double max_s = 0, capcost = 0;
    while (row_it!=ws.rows().end()) {
        auto row = *row_it++;
        max_s = row[1].value<double>()/bMVA;
        lifetime = row[2].value<int>();
        capcost = row[3].value<double>();
        auto nb_points = (row.length() - 4)/2;
        x_eff.resize(nb_points);
        y_eff.resize(nb_points);
        for (int i = 0; i<nb_points; i++) {
            x_eff[i] = row[2*i+4].value<double>();
            y_eff[i] = row[2*i+5].value<double>();
        }
        _all_battery_inverters.push_back(BatteryInverter("BI"+to_string(idx++),max_s, lifetime, capcost, x_eff, y_eff));
        Debug("Battery " << _all_battery_inverters.size() << ": ");
        _all_battery_inverters.back().print();
    }
    
    
    /* Battery Investment Options */
    /* Battery Investment Options */
    bool found_BattInvest = true;
    try{
        ws = wb.sheet_by_title("BattInvest");
    }
    catch(xlnt::key_not_found err) {
        found_BattInvest = false;
        cerr << "Cannot find sheet BattInvest, ignoring storage." << endl;
    }
    if(found_BattInvest){
        DebugOn("Processing BattInvest" << std::endl);
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW

        for (auto n: nodes) {
            auto bus = (Bus*)n;
            unsigned index = 0;
            auto row = *row_it++;
            while (row[0].to_string().compare((*row_it)[0].to_string())==0) {
                if (row[3].value<int>()!=0) {
                    min_bat = row[2].value<int>();
                    max_bat = row[3].value<int>();
                    exist = row[4].value<int>();
                    age = row[5].value<int>();
                    
                    bus->_battery_data[index] = BatteryData(min_bat,max_bat,exist,age);
                    for (unsigned i = 0; i<exist; i++) {
                        auto copy = new BatteryInverter(_all_battery_inverters[index]);
                        copy->_name += "," + bus->_name + "," + "slot"+to_string(i);
                        _existing_battery_inverters.push_back(copy);
                        _battery_inverters.push_back(copy);
                        bus->_bat.push_back(copy);
                        this->pb_min.set_val(copy->_name, -copy->_max_s);
                        this->pb_max.set_val(copy->_name, copy->_max_s);
                        this->qb_min.set_val(copy->_name, -copy->_max_s);
                        this->qb_max.set_val(copy->_name, copy->_max_s);
                        auto nb_eff_pieces = copy->_x_eff.size() - 1;
                        if(_nb_eff_pieces<nb_eff_pieces){
                            _nb_eff_pieces = nb_eff_pieces;
                        }
                        for(auto p =1; p<= _nb_eff_pieces;p++){
                            auto str = "eff"+to_string(p)+","+copy->_name;
                            if (p > nb_eff_pieces) {
                                eff_a.set_val(str, (copy->_y_eff[nb_eff_pieces] - copy->_y_eff[nb_eff_pieces-1])/(copy->_x_eff[nb_eff_pieces] - copy->_x_eff[nb_eff_pieces-1]));
                                eff_b.set_val(str, copy->_y_eff[nb_eff_pieces] - eff_a.eval()*copy->_x_eff[nb_eff_pieces]);
                            }
                            else{
                                eff_a.set_val(str, (copy->_y_eff[p] - copy->_y_eff[p-1])/(copy->_x_eff[p] - copy->_x_eff[p-1]));
                                eff_b.set_val(str, copy->_y_eff[p] - eff_a.eval()*copy->_x_eff[p]);
                            }
                        }
                    }
                    for (unsigned i = exist; i<exist+max_bat; i++) {
                        auto copy = new BatteryInverter(_all_battery_inverters[index]);
                        copy->_name += "," + bus->_name + "," + "slot"+to_string(i);
                        copy->_bat_type = index+1;
                        bus->_bat.push_back(copy);
                        bus->_pot_bat.push_back(copy);
                        _potential_battery_inverters.push_back(copy);
                        _battery_inverters.push_back(copy);
                        this->min_batt_invest.set_val(copy->_name, min_bat);
                        this->max_batt_invest.set_val(copy->_name, max_bat);
                        this->inverter_capcost.set_val(copy->_name,copy->_capcost);
                        this->pb_min.set_val(copy->_name, -copy->_max_s);
                        this->pb_max.set_val(copy->_name, copy->_max_s);
                        this->qb_min.set_val(copy->_name, -copy->_max_s);
                        this->qb_max.set_val(copy->_name, copy->_max_s);
                        auto nb_eff_pieces = copy->_x_eff.size() - 1;
                        if(_nb_eff_pieces<nb_eff_pieces){
                            _nb_eff_pieces = nb_eff_pieces;
                        }
                        for(auto p =1; p<= _nb_eff_pieces;p++){
                            auto str = "eff"+to_string(p)+","+copy->_name;
                            if (p > nb_eff_pieces) {
                                eff_a.set_val(str, (copy->_y_eff[nb_eff_pieces] - copy->_y_eff[nb_eff_pieces-1])/(copy->_x_eff[nb_eff_pieces] - copy->_x_eff[nb_eff_pieces-1]));
                                eff_b.set_val(str, copy->_y_eff[nb_eff_pieces] - eff_a.eval()*copy->_x_eff[nb_eff_pieces]);
                            }
                            else{
                                eff_a.set_val(str, (copy->_y_eff[p] - copy->_y_eff[p-1])/(copy->_x_eff[p] - copy->_x_eff[p-1]));
                                eff_b.set_val(str, copy->_y_eff[p] - eff_a.eval()*copy->_x_eff[p]);
                            }
                        }
                    }
                }
                row = *row_it++;
                index++;
            }
            bus->print();
        }
        for (auto i = 1; i<=_nb_eff_pieces; i++) {
            _eff_pieces.add("eff"+to_string(i));
        }
    }
    indices months = time("jan","feb","mar","apr","may","jun","jul","aug","sep","oct","nov","dec"); /**< Months */
    bool has_monthseason = true;
    try{
        ws = wb.sheet_by_title("monthseason");
    }
    catch(xlnt::key_not_found err){
        has_monthseason = false;
    }
    unsigned week_days = 0,weekend_days = 0,peak_days = 0;
    if (has_monthseason) {
        clog << "Processing monthseason" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        SeasonType season;
        while (row_it!=ws.rows().end()) {
            auto row = *row_it++;
            name = row[0].to_string();
            if (row[1].value<int>()==1) {
                season = summer_;
            }
            else {
                season = winter_;
            }
            _months_data.push_back(Month(name, season,week_days,weekend_days,peak_days));
        }
    }
    else {
        for (unsigned m = 0; m<12; m++) {
            name = months._indices->at(m);
            _months_data.push_back(Month(name, summer_,week_days,weekend_days,peak_days));
        }
    }
    
    bool has_nb_days = true;
    try{
        ws = wb.sheet_by_title("numberOfDays");
    }
    catch(xlnt::key_not_found err) {
        has_nb_days = false;
    }
    if(has_nb_days){
        clog << "Processing numberOfDays" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (unsigned m = 0; m<12; m++) {
            auto row = *row_it++;
            week_days = row[2].value<int>();
            weekend_days = row[3].value<int>();
            peak_days = row[1].value<int>();
            _months_data[m]._nb_week_days = week_days;
            _months_data[m]._nb_weekend_days = weekend_days;
            _months_data[m]._nb_peak_days = peak_days;
        }
    }
    else {
        for (unsigned m = 0; m<12; m++) {
            _months_data[m]._nb_week_days = 18;
            _months_data[m]._nb_weekend_days = 9;
            _months_data[m]._nb_peak_days = 3;
        }
    }
    
    bool found_SolarAverage = true;
    try{
        ws = wb.sheet_by_title("SolarAverage");
    }
    catch(xlnt::key_not_found err) {
        found_SolarAverage = false;
        cerr << "Cannot find sheet SolarAverage, setting solar generation to 0." << endl;
    }
    if(found_SolarAverage){
        ws = wb.sheet_by_title("SolarAverage");
        clog << "Processing SolarAverage (Hourly average radiance for each month)" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (unsigned m = 0; m<12; m++) {
            auto row = *row_it++;
            for (unsigned t=0; t<24; t++) {
                _months_data[m]._solar_average[t] = row[t+1].value<double>();
                
            }
        }
    }
    bool found_PV_Variance = true;
    try{
        ws = wb.sheet_by_title("SolarVariance");
    }
    catch(xlnt::key_not_found err) {
        found_PV_Variance = false;
        cerr << "Cannot find sheet SolarVariance, setting variance to 10% of average." << endl;
    }
    if(found_PV_Variance){
        clog << "Processing SolarVariance (Hourly Variance for each month)" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (unsigned m = 0; m<12; m++) {
            auto row = *row_it++;
            for (unsigned t=0; t<24; t++) {
                _months_data[m]._solar_variance[t] = row[t+1].value<double>()/bMVA;
            }
        }
    }
    else{
        for (unsigned m = 0; m<12; m++) {
            for (unsigned t=0; t<24; t++) {
                _months_data[m]._solar_variance[t] = _months_data[m]._solar_average[t]/10.;
            }
        }
    }
    bool found_WindAverage = true;
    try{
        ws = wb.sheet_by_title("WindAverage");
    }
    catch(xlnt::key_not_found err) {
        found_WindAverage = false;
        cerr << "Cannot find sheet WindAverage, setting wind generation to 0." << endl;
    }
    if(found_WindAverage){
        clog << "Processing WindAverage (Hourly Average for each month)" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (unsigned m = 0; m<12; m++) {
            auto row = *row_it++;
            for (unsigned t=0; t<24; t++) {
                _months_data[m]._wind_average[t] = row[t+1].value<double>()/bMVA;
            }
        }
    }
    bool found_WindVariance = true;
    try{
        ws = wb.sheet_by_title("WindVariance");
    }
    catch(xlnt::key_not_found err) {
        found_WindVariance = false;
        cerr << "Cannot find sheet WindVariance, setting variance to 10% of average." << endl;
    }
    if(found_WindVariance){
        clog << "Processing WindVariance (Hourly Variance for each month)" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (unsigned m = 0; m<12; m++) {
            auto row = *row_it++;
            for (unsigned t=0; t<24; t++) {
                _months_data[m]._wind_variance[t] = row[t+1].value<double>()/bMVA;
            }
        }
    }
    else{
        for (unsigned m = 0; m<12; m++) {
            for (unsigned t=0; t<24; t++) {
                _months_data[m]._wind_variance[t] = _months_data[m]._wind_average[t]/10.;
            }
        }
    }
    
    
    string key;
    //Building PV and Wind generation parameters
    for (unsigned i = 0; i<nb_nodes; i++) {
        auto b = (Bus*)nodes[i];
        auto potential_PV = b->_max_PV_cap;
        auto potential_wind = b->_max_Wind_cap;
        if (potential_PV>0) {
            auto new_pv = new PV(b->_name, b->_min_PV_cap, potential_PV, _PV_data._lifetime, _PV_data._fixed_cost, _PV_data._var_cost);
            auto name = new_pv->_name;
            b->_pv.push_back(new_pv);
            b->_pot_pv.push_back(new_pv);
            _all_PV_gens.push_back(new_pv);
            _potential_PV_gens.push_back(new_pv);
            this->pv_max.set_val(name, potential_PV/bMVA);
            this->pv_min.set_val(name, 0);
            this->pv_capcost.set_val(name,new_pv->_capcost);
            this->pv_varcost.set_val(name,new_pv->_varcost*bMVA);
            
            for (unsigned m = 0; m<months.size(); m++) {
                for (unsigned t=0; t<24; t++) {
                    auto key = name + "," + months._indices->at(m) + ",week," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                    key = name + "," + months._indices->at(m) + ",peak," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                    key = name + "," + months._indices->at(m) + ",weekend," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                }
            }
        }
        if (b->_existing_PV_cap > 0) {
            auto new_pv = new PV(b->_name+"_exist", b->_existing_PV_cap, b->_existing_PV_cap, _PV_data._lifetime, _PV_data._fixed_cost, _PV_data._var_cost);
            b->_pv.push_back(new_pv);
            auto name = new_pv->_name;
            _existing_PV_gens.push_back(new_pv);
            _all_PV_gens.push_back(new_pv);
            this->pv_max.set_val(name, b->_existing_PV_cap/bMVA);
            this->pv_min.set_val(name, 0);
            for (unsigned m = 0; m<months.size(); m++) {
                for (unsigned t=0; t<24; t++) {
                    auto key = name + "," + months._indices->at(m) + ",week," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                    key = name + "," + months._indices->at(m) + ",peak," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                    key = name + "," + months._indices->at(m) + ",weekend," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                }
            }
        }
        if (potential_wind>0) {
            auto wg = new WindGen(b->_name, _wind_data._cap, _wind_data._lifetime, _wind_data._cap_cost);
            b->_wind.push_back(wg);
            b->_pot_wind.push_back(wg);
            _all_wind_gens.push_back(wg);
            _potential_wind_gens.push_back(wg);
            for (unsigned m = 0; m<months.size(); m++) {
                for (unsigned t=0; t<24; t++) {
                    auto key = name + "," + months._indices->at(m) + ",week," + to_string(t+1) ;
                    pw_max.set_val(key, _months_data[m]._wind_average[t]);
                    pw_min.set_val(key, 0);
                    key = name + "," + months._indices->at(m) + ",peak," + to_string(t+1) ;
                    pw_max.set_val(key, _months_data[m]._wind_average[t]);
                    pw_min.set_val(key, 0);
                    key = name + "," + months._indices->at(m) + ",weekend," + to_string(t+1) ;
                    pw_max.set_val(key, _months_data[m]._wind_average[t]);
                    pw_min.set_val(key, 0);
                }
            }
        }
        if (b->_existing_Wind_cap>0) {
            auto wg = new WindGen(b->_name+"_exist", _wind_data._cap, _wind_data._lifetime, _wind_data._cap_cost);
            b->_wind.push_back(wg);
            _all_wind_gens.push_back(wg);
            _existing_wind_gens.push_back(wg);
            for (unsigned m = 0; m<months.size(); m++) {
                for (unsigned t=0; t<24; t++) {
                    auto key = name + "," + months._indices->at(m) + ",week," + to_string(t+1) ;
                    pw_max.set_val(key, _months_data[m]._wind_average[t]);
                    pw_min.set_val(key, 0);
                    key = name + "," + months._indices->at(m) + ",peak," + to_string(t+1) ;
                    pw_max.set_val(key, _months_data[m]._wind_average[t]);
                    pw_min.set_val(key, 0);
                    key = name + "," + months._indices->at(m) + ",weekend," + to_string(t+1) ;
                    pw_max.set_val(key, _months_data[m]._wind_average[t]);
                    pw_min.set_val(key, 0);
                }
            }
        }
    }
    //    clog << "Loads:\n";
    //    pl.print(true);
    clog << "Reading excel file complete" << std::endl;
    return 0;
}

int PowerNet::readDERCAM(const string& fname){
    unsigned index, b_type;
    string name;
    xlnt::workbook wb;
    double wall0 = get_wall_time();
    clog << "Opening excel file...\n";
    wb.load(fname);
    double wall1 = get_wall_time();
    clog << "Done.\n";
    clog << "Wall clock computing time =  " << wall1 - wall0 << "\n";

    auto ws = wb.sheet_by_title("PowerFlowParams");
    clog << "Processing PowerFlowParams" << std::endl;
    auto row_it = ws.rows().begin();
    //    row_it++;//SKIP FRIST ROW
    auto row = *row_it++;
    name = row[1].to_string();
    Debug("Ref Bus name = " << name << endl);
    ref_bus = name;
    while (row_it!=ws.rows().end())
    {
        auto row = *row_it++;
        name = row[0].to_string();
        if (name.compare("Sbase")==0) {
            bMVA = row[1].value<double>();
            Debug("Base MVA = " << to_string(bMVA) << endl);
        }
        if (name.compare("Vbase")==0) {
            bV = row[1].value<double>();
            Debug("Base V = " << to_string(bV) << endl);
        }
        if (name.compare("NoOfNodes")==0) {
            nb_nodes = row[1].value<double>();
            Debug("Number of buses = " << to_string(nb_nodes) << endl);
            for (size_t i = 0; i<nb_nodes; i++) {
                auto bus = new Bus("Bus"+to_string(i+1));
                add_node(bus);
            }
        }
    }

    ws = wb.sheet_by_title("PowerFlowModel1Params");
    clog << "Processing PowerFlowModel1Params" << std::endl;
    row_it = ws.rows().begin();
    while (row_it!=ws.rows().end())
    {
        auto row = *row_it++;
        name = row[0].to_string();
        if (name.compare("ThetaMin")==0) {
            m_theta_lb = row[1].value<double>();
            Debug("Theta Min = " << to_string(m_theta_lb) << endl);
        }
        else if (name.compare("ThetaMax")==0) {
            m_theta_ub = row[1].value<double>();
            Debug("Theta Max = " << to_string(m_theta_ub) << endl);
        }
        else if (name.compare("VoltMin")==0) {
            vmin = pow(row[1].value<double>(),2);
            Debug("Voltage Min = " << to_string(vmin) << endl);
        }
        else if (name.compare("VoltMax")==0) {
            vmax = pow(row[1].value<double>(),2);
            Debug("Voltage Max = " << to_string(vmax) << endl);
        }

    }


    bool found_general_tech = true;
    try{
        ws = wb.sheet_by_title("GeneralTechConstraints");
    }
    catch(xlnt::key_not_found err) {
        found_general_tech = false;
        cerr << "Cannot find sheet GeneralTechConstraints, trying sheet GeneralTechConstrains" << endl;
    }
    if (!found_general_tech) {
        try{
            ws = wb.sheet_by_title("GeneralTechConstrains");
        }
        catch(xlnt::key_not_found err) {
            cerr << "Cannot find sheet GeneralTechConstraints or GeneralTechConstrains, exiting" << endl;
            return -1;
        }
    }

    clog << "Processing GeneralTechConstraints" << std::endl;
    row_it = ws.rows().begin();
    while (row_it!=ws.rows().end())
    {
        auto row = *row_it++;
        name = row[0].to_string();
        if (name.compare("MaxIdenticalUnits")==0) {
            max_ident_units = row[1].value<int>();
            Debug("Max number of identical units = " << to_string(max_ident_units) << endl);
        }
        if(name.compare("PeakPVEfficiency")==0) {
            PeakPVEfficiency = row[1].value<double>();
        }
    }


    ws = wb.sheet_by_title("PowerFactor");
    clog << "Processing PowerFactor" << std::endl;
    row_it = ws.rows().begin();
    while (row_it!=ws.rows().end())
    {
        auto row = *row_it++;
        name = row[0].to_string();
        if (name.compare("Electricity")==0) {
            _power_factor = row[1].value<double>();
            Debug("Electric Power Factor = " << to_string(_power_factor) << endl);
            break;
        }
    }

    /* Investment Options */
    ws = wb.sheet_by_title("NodeType");
    clog << "Processing NodeType" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    for (auto n: nodes) {
        auto row = *row_it++;
        auto bus = (Bus*)n;
        bus->_type = row[1].value<int>();
        bus->_enable_load = row[3].value<int>()==1;
        if (bus->_enable_load) {
            Debug(bus->_name << " load enabled."<<endl);
        }
        else {
            Debug(bus->_name << " load disabled."<<endl);
        }
    }

    ws = wb.sheet_by_title("BranchType_Cable");
    clog << "Processing Branch_Type_Cable" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    string src, dest;
    index = 0;
    for (size_t i = 0; i<nb_nodes-1; i++) {
        auto row = *row_it++;
        for (size_t j = i+1; j<nb_nodes; j++) {
            if (row[j+1].has_value() && row[j+1].value<unsigned>()>0) {
                src = "Bus"+to_string(i+1);
                dest = "Bus"+to_string(j+1);
                b_type = row[j+1].value<unsigned>();
                DebugOn("Arc (" << i+1 << "," << j+1 << ") has type " << b_type << endl);
                auto arc = new Line(to_string(index) + "," + src + "," + dest); // Name of lines
                arc->b_type = b_type;
                arc->_id = index++;
                arc->_src = get_node(src);
                arc->_dest= get_node(dest);
                arc->connect();
                add_arc(arc);
                _exist_arcs.push_back(arc);
            }
        }
    }
    bool found_branch_expansion = true;
    try{
        ws = wb.sheet_by_title("Branch_Expansion");
    }
    catch(xlnt::key_not_found err) {
        found_branch_expansion = false;
        cerr << "Cannot find sheet Branch_Expansion, ignoring expansion options" << endl;
    }
    if (found_branch_expansion) {
        clog << "Processing Branch_Expansion" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (size_t i = 0; i<nb_nodes-1; i++) {
            auto row = *row_it++;
            for (size_t j = i+1; j<nb_nodes; j++) {
                if (row[j+1].has_value() && row[j+1].value<unsigned>()>0) {
                    src = "Bus"+to_string(i+1);
                    dest = "Bus"+to_string(j+1);
                    b_type = row[j+1].value<unsigned>();
                    Debug("Expansion possible on arc (" << i+1 << "," << j+1 << ") with type " << b_type << endl);
                    auto arc = new Line(to_string(index) + "," + src + "," + dest); // Name of lines
                    arc->_expansion = true;
                    arc->b_type = b_type;
                    arc->_id = index++;
                    arc->_src = get_node(src);
                    arc->_dest= get_node(dest);
                    v_diff_max.set_val(arc->_name, vmax - vmin);
                    arc->connect();
                    add_arc(arc);
                    _potential_expansion.push_back(arc);
                }
            }
        }
    }
    v_diff_max.print(true);

    ws = wb.sheet_by_title("CableLen");
    clog << "Processing CableLen" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    index = 0;
    double len = 0;
    for (size_t i = 0; i<nb_nodes-1; i++) {
        auto row = *row_it++;
        for (size_t j = i+1; j<nb_nodes; j++) {
            if (row[j+1].has_value() && row[j+1].value<unsigned>()>0) {
                src = "Bus"+to_string(i+1);
                dest = "Bus"+to_string(j+1);
                len = row[j+1].value<unsigned>();
                Debug("(" << i+1 << "," << j+1 << ") has len " << len << endl);
                auto a = get_arc(src, dest);
                a->_len = len;
            }
        }
    }

    ws = wb.sheet_by_title("CableParams");
    clog << "Processing CableParams" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    vector<pair<double,double>> rx; /* Vector of resistance,inductance */
    vector<double> cost; /* Vector of expansion costs */
    vector<double> Smax; /* Vector of thermal limits */
    while (row_it!=ws.rows().end()) {
        auto row = *row_it++;
        rx.push_back(make_pair(row[4].value<double>(), row[5].value<double>()));
        cost.push_back(row[2].value<double>());
        Smax.push_back(row[7].value<double>()/bMVA);
    }
    Line* l;
    for (auto a: arcs) {
        l = (Line*)a;
        l->r = rx[l->b_type-1].first;
        l->x = rx[l->b_type-1].second;
//        r.set_val(l->_name,l->r);
//        x.set_val(l->_name,l->x);
        l->cost = cost[l->b_type-1]*l->_len;
        l->smax = Smax[l->b_type-1];
//        this->S_max.set_val(l->_name, l->smax);
        if (l->_expansion) {
            this->expansion_capcost.set_val(l->_name, l->cost);
        }
        l->print();
    }
    S_max.print(true);
    /* Investment Options */
    ws = wb.sheet_by_title("EnableInvestment");
    clog << "Processing EnableInvestment" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    for (auto n: nodes) {
        auto row = *row_it++;
        auto bus = (Bus*)n;
        bus->_diesel_invest = (row[1].value<int>()==1);
        bus->_batt_invest = row[2].value<int>()==1;
        bus->_wind_gens = row[3].value<int>();
    }
    bool found_parameterTable = true;
    try{
        ws = wb.sheet_by_title("parameterTable");
    }
    catch(xlnt::key_not_found err) {
        found_parameterTable = false;
        cerr << "Cannot find sheet parameterTable" << endl;
    }
    if(found_parameterTable){
        clog << "Processing parameterTable" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        row = *row_it;
        while (row[0].to_string().compare("PeakPVEfficiency")) {
            row = *(++row_it);
        }
        PeakPVEfficiency = row[1].value<double>();
        DebugOn("PeakPVEfficiency = " << PeakPVEfficiency << endl);
    }

    ws = wb.sheet_by_title("MaxSpaceAvailablePVSolar");
    clog << "Processing MaxSpaceAvailablePVSolar" << std::endl;
    row_it = ws.rows().begin();
    for (auto n: nodes) {
        row = *row_it++;
        auto bus = (Bus*)n;
        bus->_max_PV_area = row[1].value<double>();
    }

    /* Battery Investment Options */
    bool found_ContinuousInvestParams = true;
    try{
        ws = wb.sheet_by_title("ContinuousInvestParams");
    }
    catch(xlnt::key_not_found err) {
        found_ContinuousInvestParams = false;
        cerr << "Cannot find sheet ContinuousInvestParams, trying sheet ContinuousInvestParameter" << endl;
    }
    if(!found_ContinuousInvestParams){
        try{
            ws = wb.sheet_by_title("ContinuousInvestParameter");
        }
        catch(xlnt::key_not_found err) {
            cerr << "Cannot find sheet ContinuousInvestParams or ContinuousInvestParameter, exiting" << endl;
            return -1;
        }
    }
    DebugOn("Processing ContinuousInvestParams (Battery fixed and variable costs)" << std::endl);
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    row = *row_it;
    while (row[0].to_string().compare("ElectricStorage")) {
        row = *(++row_it);
    }
    cb_f = row[1].value<double>();
    cb_v = row[2].value<double>();
    while (row[0].to_string().compare("PV")) {
        row = *(++row_it);
    }
    _PV_data._fixed_cost = row[1].value<double>();
    _PV_data._var_cost = row[2].value<double>();
    _PV_data._lifetime = row[3].value<double>();




    unsigned min_bat = 0, min_PV = 0;
    //ContinuousVariableForcedInvest
    bool found_ContinuousInvest = true;
    try{
        ws = wb.sheet_by_title("ContinuousInvest");
    }
    catch(xlnt::key_not_found err) {
        found_ContinuousInvest = false;
        cerr << "Cannot find sheet ContinuousInvest, trying sheet ContinuousVariableForcedInvest" << endl;
    }
    if(!found_ContinuousInvest){
        try{
            ws = wb.sheet_by_title("ContinuousVariableForcedInvest");
        }
        catch(xlnt::key_not_found err) {
            cerr << "Cannot find sheet ContinuousInvest or ContinuousVariableForcedInvest, exiting" << endl;
            return -1;
        }
    }
    clog << "Processing ContinuousInvest (Battery investment options at each node)" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    row = *row_it;
    for (auto n: nodes) {
        while (row[1].to_string().compare("ElectricStorage")) {
            row = *(++row_it);
        }
        auto bus = (Bus*)n;
        min_bat = row[3].value<int>();
        unsigned exist = 0;
        if(row[4].value<int>()==1){
            exist = min_bat;
        }
        unsigned max_bat = min_bat;
        if(row[2].value<int>()==0){
            max_bat += max_ident_units-exist;
        }
        bus->_max_batt_cap = min_bat;
        bus->_min_batt_cap = max_bat;
        bus->_existing_batt_cap = row[4].value<double>();
        bus->_existing_batt_age = row[5].value<int>();
        while (row[1].to_string().compare("PV")) {
            row = *(++row_it);
        }
        min_PV = row[3].value<int>();
        if(row[4].value<int>()==1){
            bus->_existing_PV_cap = min_PV;
            min_PV = 0;
        }
        bus->_min_PV_cap = min_PV;
        bus->_max_PV_cap = min_PV;
        if(row[2].value<int>()==0){
            bus->_max_PV_cap += PeakPVEfficiency*(bus->_max_PV_area) ;
        }
        bus->_existing_PV_age = row[5].value<int>();


    }

    ws = wb.sheet_by_title("DEROPT");
    clog << "Processing DEROPT" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW

    unsigned lifetime,type,idx = 1;
    auto first_row = *(ws.rows().begin());
    while (row_it!=ws.rows().end()) {
        double max_p = 0, max_s =  0, capcost = 0, c0 = 0,c1 = 0,c2 = 0,eff = 0,max_ramp_up = 0,max_ramp_down = 0,min_up_time = 0,min_down_time = 0;
        auto row = *row_it++;
        max_p = row[2].value<double>()/bMVA;
        max_s = row[3].value<double>()/bMVA;
        lifetime = row[4].value<int>();
        capcost = row[5].value<double>();
        c0 = row[6].value<double>();
        c1 = row[7].value<double>()*bMVA;
        if (first_row[8].to_string().compare("OMVar2")==0) {
            c2 = row[8].value<double>()*pow(bMVA,2);
        }
        else {
            c2 = c1*bMVA;
        }

        for (auto i = 9; i<first_row.length(); i++) {
            if (first_row[i].to_string().compare("type")==0) {
                type = row[i].value<int>();
            }
            if (first_row[i].to_string().compare("efficiency")==0) {
                eff = row[i].value<double>();
            }
            if (first_row[i].to_string().compare("MaxRampUp")==0) {
                max_ramp_up = row[i].value<double>();
            }
            if (first_row[i].to_string().compare("MaxRampDown")==0) {
                max_ramp_down = row[i].value<double>();
            }
            if (first_row[i].to_string().compare("MinDownTime")==0) {
                min_down_time = row[i].value<int>();
            }
            if (first_row[i].to_string().compare("MinUpTime")==0) {
                min_up_time = row[i].value<int>();
            }
        }
        _all_diesel_gens.push_back(DieselGen("DG"+to_string(idx++), max_p, max_s, lifetime, capcost, c0, c1, c2, type, eff, max_ramp_down, max_ramp_up,min_down_time,min_up_time));
//        _all_diesel_gens.back().print();
    }

    /* Diesel Investment Options */
    bool found_disc_invest = true;
    try{
        ws = wb.sheet_by_title("DiscreteInvest");
    }
    catch(xlnt::key_not_found err) {
        found_disc_invest = false;
        cerr << "Cannot find sheet DiscreteInvest, trying sheet GenConstraints" << endl;
    }
    if(!found_disc_invest){
        try{
            ws = wb.sheet_by_title("GenConstraints");
        }
        catch(xlnt::key_not_found err) {
            cerr << "Cannot find sheet DiscreteInvest or GenConstraints, exiting" << endl;
            return -1;
        }
    }
    clog << "Processing DiscreteInvest (Diesel investment options at each node)" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    unsigned mind,maxd,exist,age;
    for (auto n: nodes) {
        auto bus = (Bus*)n;
        unsigned index = 0;
        auto row = *row_it++;
        while (row[0].to_string().compare((*row_it)[0].to_string())==0) {
            if (!(row[5].value<int>()==1 && row[6].value<int>()==0) || row[7].value<int>()>0) {
                mind = row[6].value<int>();
                exist = 0;
                if(row[7].value<int>()==1){
                    exist = mind;
                    mind = 0;
                }
                maxd = mind;
                if(row[5].value<int>()==0){
                    maxd+=max_ident_units;
                }

                age = row[8].value<int>();
                bus->_diesel_data[index] = DieselData(mind,maxd,exist,age);
                for (unsigned i = 0; i<exist; i++) {
                    auto copy = _all_diesel_gens[index];
                    auto name = copy._name + "," + bus->_name + "," + "slot"+to_string(i);
                    auto gen = new Gen(bus, name, 0, copy._max_p, -copy._max_s, copy._max_s);
                    _existing_diesel_gens.push_back(gen);
                    gens.push_back(gen);
                    bus->_gen.push_back(gen);
                    this->c0.set_val(name,copy._c0);
                    this->c1.set_val(name,copy._c1);
                    this->c2.set_val(name,copy._c2);
                    this->pg_min.set_val(name,0);
                    this->pg_max.set_val(name,copy._max_p);
                    this->qg_min.set_val(name,-copy._max_s);
                    this->qg_max.set_val(name,copy._max_s);
                    this->min_dt.set_val(name,copy._min_down_time);
                    this->min_ut.set_val(name,copy._min_up_time);
                    this->ramp_up.set_val(name,copy._max_ramp_up);
                    this->ramp_down.set_val(name,copy._max_ramp_down);
                    this->gen_eff.set_val(name,copy._eff);
                }
                for (unsigned i = exist; i<exist+maxd; i++) {
                    auto copy = _all_diesel_gens[index];
                    auto name = copy._name + "," + bus->_name + "," + "slot"+to_string(i);
                    auto gen = new Gen(bus, name, 0, copy._max_p, -copy._max_s, copy._max_s);
                    _potential_diesel_gens.push_back(gen);
                    gen->_gen_type = index+1;
                    gens.push_back(gen);
                    bus->_gen.push_back(gen);
                    bus->_pot_gen.push_back(gen);
                    this->min_diesel_invest.set_val(name, mind);
                    this->max_diesel_invest.set_val(name, maxd);
                    this->c0.set_val(name,copy._c0);
                    this->c1.set_val(name,copy._c1);
                    this->c2.set_val(name,copy._c2);
                    this->pg_min.set_val(name,0);
                    this->pg_max.set_val(name,copy._max_p);
                    this->qg_min.set_val(name,-copy._max_s);
                    this->qg_max.set_val(name,copy._max_s);
                    this->gen_capcost.set_val(name,copy._capcost);
                    this->min_dt.set_val(name,copy._min_down_time);
                    this->min_ut.set_val(name,copy._min_up_time);
                    this->ramp_up.set_val(name,copy._max_ramp_up);
                    this->ramp_down.set_val(name,copy._max_ramp_down);
                    this->gen_eff.set_val(name,copy._eff);
                }

            }
//            gen_eff.print(true);
            row = *row_it++;
            index++;
        }
    }

    ws = wb.sheet_by_title("BattInverterParameters");
    clog << "Processing BattInverterParameters (Battery Inverter Properties)" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    vector<double> x_eff;
    vector<double> y_eff;
    idx = 1;
    double max_s = 0, capcost = 0;
    while (row_it!=ws.rows().end()) {
        auto row = *row_it++;
        max_s = row[1].value<double>()/bMVA;
        lifetime = row[2].value<int>();
        capcost = row[3].value<double>();
        auto nb_points = (row.length() - 4)/2;
        x_eff.resize(nb_points);
        y_eff.resize(nb_points);
        for (int i = 0; i<nb_points; i++) {
            x_eff[i] = row[2*i+4].value<double>();
            y_eff[i] = row[2*i+5].value<double>();
        }
        _all_battery_inverters.push_back(BatteryInverter("BI"+to_string(idx++),max_s, lifetime, capcost, x_eff, y_eff));
        Debug("Battery Inverter" << _all_battery_inverters.size() << ": ");
//        _all_battery_inverters.back().print();
    }


    /* Battery Investment Options */
    ws = wb.sheet_by_title("BattInverterForcedInvest");
    clog << "Processing BattInverterForcedInvest (Battery inverter investment options at each node)" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    for (auto n: nodes) {
        auto bus = (Bus*)n;
        unsigned index = 0;
        auto row = *row_it++;
        while (row[0].to_string().compare((*row_it)[0].to_string())==0) {
            if (!(row[2].value<int>()==1 && row[3].value<int>()==0) || row[4].value<int>()>0) {
                min_bat = row[3].value<int>();
                unsigned exist = 0;
                if(row[4].value<int>()==1){
                    exist = min_bat;
                    min_bat = 0;
                }
                unsigned max_bat = min_bat;
                if(row[2].value<int>()==0){
                    max_bat += max_ident_units;
                }
                age = row[5].value<int>();
                bus->_battery_data[index] = BatteryData(min_bat,max_bat,exist,age);
                for (unsigned i = 0; i<exist; i++) {
                    auto copy = new BatteryInverter(_all_battery_inverters[index]);
                    copy->_name += "," + bus->_name + "," + "slot"+to_string(i);
                    _existing_battery_inverters.push_back(copy);
                    _battery_inverters.push_back(copy);
                    bus->_bat.push_back(copy);
                    this->pb_min.set_val(copy->_name, -copy->_max_s);
                    this->pb_max.set_val(copy->_name, copy->_max_s);
                    this->qb_min.set_val(copy->_name, -copy->_max_s);
                    this->qb_max.set_val(copy->_name, copy->_max_s);
                    auto nb_eff_pieces = copy->_x_eff.size() - 1;
                    if(_nb_eff_pieces<nb_eff_pieces){
                        _nb_eff_pieces = nb_eff_pieces;
                    }
                    for(auto p =1; p<= _nb_eff_pieces;p++){
                        auto str = "eff"+to_string(p)+","+copy->_name;
                        if (p > nb_eff_pieces) {
                            eff_a.set_val(str, (copy->_y_eff[nb_eff_pieces] - copy->_y_eff[nb_eff_pieces-1])/(copy->_x_eff[nb_eff_pieces] - copy->_x_eff[nb_eff_pieces-1]));
                            eff_b.set_val(str, copy->_y_eff[nb_eff_pieces] - eff_a.eval()*copy->_x_eff[nb_eff_pieces]);
                        }
                        else{
                            eff_a.set_val(str, (copy->_y_eff[p] - copy->_y_eff[p-1])/(copy->_x_eff[p] - copy->_x_eff[p-1]));
                            eff_b.set_val(str, copy->_y_eff[p] - eff_a.eval()*copy->_x_eff[p]);
                        }
                    }
                }
                for (unsigned i = exist; i<exist+max_bat; i++) {
                    auto copy = new BatteryInverter(_all_battery_inverters[index]);
                    copy->_name += "," + bus->_name + "," + "slot"+to_string(i);
                    copy->_bat_type = index+1;
                    bus->_bat.push_back(copy);
                    bus->_pot_bat.push_back(copy);
                    _potential_battery_inverters.push_back(copy);
                    _battery_inverters.push_back(copy);
                    this->min_batt_invest.set_val(copy->_name, min_bat);
                    this->max_batt_invest.set_val(copy->_name, max_bat);
                    this->inverter_capcost.set_val(copy->_name,copy->_capcost+cb_f.eval());
                    this->pb_min.set_val(copy->_name, -copy->_max_s);
                    this->pb_max.set_val(copy->_name, copy->_max_s);
                    this->qb_min.set_val(copy->_name, -copy->_max_s);
                    this->qb_max.set_val(copy->_name, copy->_max_s);
                    auto nb_eff_pieces = copy->_x_eff.size() - 1;
                    if(_nb_eff_pieces<nb_eff_pieces){
                        _nb_eff_pieces = nb_eff_pieces;
                    }
                    for(auto p =1; p<= _nb_eff_pieces;p++){
                        auto str = "eff"+to_string(p)+","+copy->_name;
                        if (p > nb_eff_pieces) {
                            eff_a.set_val(str, (copy->_y_eff[nb_eff_pieces] - copy->_y_eff[nb_eff_pieces-1])/(copy->_x_eff[nb_eff_pieces] - copy->_x_eff[nb_eff_pieces-1]));
                            eff_b.set_val(str, copy->_y_eff[nb_eff_pieces] - eff_a.eval()*copy->_x_eff[nb_eff_pieces]);
                        }
                        else{
                            eff_a.set_val(str, (copy->_y_eff[p] - copy->_y_eff[p-1])/(copy->_x_eff[p] - copy->_x_eff[p-1]));
                            eff_b.set_val(str, copy->_y_eff[p] - eff_a.eval()*copy->_x_eff[p]);
                        }
                    }
                }
            }
            row = *row_it++;
            index++;
        }
//        bus->print();
    }
    for (auto i = 1; i<=_nb_eff_pieces; i++) {
        _eff_pieces.add("eff"+to_string(i));
    }
        indices months = time("jan","feb","mar","apr","may","jun","jul","aug","sep","oct","nov","dec"); /**< Months */
    bool has_monthseason = true;
    try{
        ws = wb.sheet_by_title("monthseason");
    }
    catch(xlnt::key_not_found err){
        has_monthseason = false;
    }
    unsigned week_days = 0,weekend_days = 0,peak_days = 0;
    if (has_monthseason) {
        clog << "Processing monthseason" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        SeasonType season;
        while (row_it!=ws.rows().end()) {
            auto row = *row_it++;
            name = row[0].to_string();
            if (row[1].value<int>()==1) {
                season = summer_;
            }
            else {
                season = winter_;
            }
            _months_data.push_back(Month(name, season,week_days,weekend_days,peak_days));
        }
    }
    else {
        for (unsigned m = 0; m<12; m++) {
            name = months._indices->at(m);
            _months_data.push_back(Month(name, summer_,week_days,weekend_days,peak_days));
        }
    }

    bool has_nb_days = true;
    try{
    ws = wb.sheet_by_title("numberOfDays");
    }
    catch(xlnt::key_not_found err) {
        has_nb_days = false;
    }
    if(has_nb_days){
        clog << "Processing numberOfDays" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (unsigned m = 0; m<12; m++) {
            auto row = *row_it++;
            week_days = row[2].value<int>();
            weekend_days = row[3].value<int>();
            peak_days = row[1].value<int>();
            _months_data[m]._nb_week_days = week_days;
            _months_data[m]._nb_weekend_days = weekend_days;
            _months_data[m]._nb_peak_days = peak_days;
        }
    }
    else {
        for (unsigned m = 0; m<12; m++) {
            week_days = row[2].value<int>();
            weekend_days = row[3].value<int>();
            peak_days = row[1].value<int>();
            _months_data[m]._nb_week_days = 18;
            _months_data[m]._nb_weekend_days = 9;
            _months_data[m]._nb_peak_days = 3;
        }
    }
    bool found_Wind_Power_Potential = true;
    try{
        ws = wb.sheet_by_title("Wind_Power_Potential");
    }
    catch(xlnt::key_not_found err) {
        found_Wind_Power_Potential = false;
        cerr << "Cannot find sheet found_Wind_Power_Potential, trying sheet WindPowerNormalized" << endl;
    }
    if(!found_Wind_Power_Potential){
        try{
            ws = wb.sheet_by_title("WindPowerNormalized");
        }
        catch(xlnt::key_not_found err) {
            cerr << "Cannot find sheet Wind_Power_Potential or WindPowerNormalized, exiting" << endl;
            return -1;
        }
    }

    clog << "Processing Wind_Power_Potential (Hourly Average for each Month)" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    for (unsigned m = 0; m<12; m++) {
        auto row = *row_it++;
        for (unsigned t=0; t<24; t++) {
            _months_data[m]._wind_average[t] = row[t+1].value<double>()/bMVA;
        }
    }

    ws = wb.sheet_by_title("solarInsolation");
    clog << "Processing solarInsolation (Hourly average radiance for each month)" << std::endl;
    row_it = ws.rows().begin();
    row_it++;//SKIP FRIST ROW
    for (unsigned m = 0; m<12; m++) {
        auto row = *row_it++;
        for (unsigned t=0; t<24; t++) {
            _months_data[m]._solar_average[t] = row[t+1].value<double>();

        }
    }
    bool found_Wind_Power_Variance = true;
    try{
        ws = wb.sheet_by_title("Wind_Power_Variance");
    }
    catch(xlnt::key_not_found err) {
        found_Wind_Power_Potential = false;
        cerr << "Cannot find sheet Wind_Power_Variance, using average values" << endl;
    }
    if(found_Wind_Power_Variance){
        clog << "Processing Wind_Power_Variance (Hourly Variance for each Month)" << std::endl;
        row_it = ws.rows().begin();
        row_it++;//SKIP FRIST ROW
        for (unsigned m = 0; m<12; m++) {
            auto row = *row_it++;
            for (unsigned t=0; t<24; t++) {
                _months_data[m]._wind_variance[t] = row[t+1].value<double>()/bMVA;
            }
        }
    }
    else{
        for (unsigned m = 0; m<12; m++) {
            for (unsigned t=0; t<24; t++) {
                _months_data[m]._wind_variance[t] = _months_data[m]._wind_average[t]/10.;
            }
        }
    }

    ws = wb.sheet_by_title("WindGeneratorPar");
    clog << "Processing WindGeneratorPar (Wind Data)" << std::endl;
    row_it = ws.rows().begin();
    while (row_it!=ws.rows().end())
    {
        auto row = *row_it++;
        name = row[0].to_string();
        if (name.compare("WindRatedCapacity")==0) {
            _wind_data._cap = row[1].value<double>()/bMVA;
            Debug("Wind Farm Rated Capacity = " << to_string(_wind_data._cap) << endl);
        }
        else if (name.compare("WindCapitalCost")==0) {
            _wind_data._cap_cost = row[1].value<double>();
            Debug("Wind Farm Capital Cost = " << to_string(_wind_data._cap_cost) << endl);
        }
        else if (name.compare("WindFixedMaintCost")==0) {
            _wind_data._fixed_cost = row[1].value<double>();
            Debug("Wind Farm Fixed Cost = " << to_string(_wind_data._fixed_cost) << endl);
        }
        else if (name.compare("WindVarMaintCost")==0) {
            _wind_data._var_cost = row[1].value<double>();
            Debug("Wind Farm Variable Cost = " << to_string(_wind_data._var_cost) << endl);
        }
        else if (name.compare("WindLifeTime")==0) {
            _wind_data._lifetime = row[1].value<double>();
            Debug("Wind Farm Life Time = " << to_string(_wind_data._lifetime) << endl);
        }

    }
    string key;

    for (unsigned i = 0; i<nb_nodes; i++) {
        auto b = (Bus*)nodes[i];
        auto potential_PV = b->_max_PV_cap;
        if (potential_PV>0) {
            auto new_pv = new PV(b->_name, b->_min_PV_cap, potential_PV, _PV_data._lifetime, _PV_data._fixed_cost, _PV_data._var_cost);
            auto name = new_pv->_name;
            b->_pv.push_back(new_pv);
            b->_pot_pv.push_back(new_pv);
            _all_PV_gens.push_back(new_pv);
            _potential_PV_gens.push_back(new_pv);
            this->pv_max.set_val(name, potential_PV/bMVA);
            this->pv_min.set_val(name, 0);
            this->pv_capcost.set_val(name,new_pv->_capcost);
            this->pv_varcost.set_val(name,new_pv->_varcost*bMVA);

            for (unsigned m = 0; m<months.size(); m++) {
                for (unsigned t=0; t<24; t++) {
                    auto key = name + "," + months._indices->at(m) + ",week," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                    key = name + "," + months._indices->at(m) + ",peak," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                    key = name + "," + months._indices->at(m) + ",weekend," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                }
            }
        }
        if (b->_existing_PV_cap > 0) {
            auto new_pv = new PV(b->_name+"_exist", b->_existing_PV_cap, b->_existing_PV_cap, _PV_data._lifetime, _PV_data._fixed_cost, _PV_data._var_cost);
            b->_pv.push_back(new_pv);
            auto name = new_pv->_name;
            _existing_PV_gens.push_back(new_pv);
            _all_PV_gens.push_back(new_pv);
            this->pv_max.set_val(name, b->_existing_PV_cap/bMVA);
            this->pv_min.set_val(name, 0);
            for (unsigned m = 0; m<months.size(); m++) {
                for (unsigned t=0; t<24; t++) {
                    auto key = name + "," + months._indices->at(m) + ",week," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                    key = name + "," + months._indices->at(m) + ",peak," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                    key = name + "," + months._indices->at(m) + ",weekend," + to_string(t+1) ;
                    pv_out.set_val(key, _months_data[m]._solar_average[t]);
                }
            }
        }
        for (auto i = 0; i<b->_wind_gens; i++) {
            auto name = b->_name + "," + to_string(i);
            auto wg = new WindGen(name, _wind_data._cap, _wind_data._lifetime, _wind_data._cap_cost);
            b->_wind.push_back(wg);
            _all_wind_gens.push_back(wg);
            for (unsigned m = 0; m<months.size(); m++) {
                for (unsigned t=0; t<24; t++) {
                    auto key = name + "," + months._indices->at(m) + ",week," + to_string(t+1) ;
                    pw_max.set_val(key, _months_data[m]._wind_average[t]);
                    pw_min.set_val(key, 0);
                    key = name + "," + months._indices->at(m) + ",peak," + to_string(t+1) ;
                    pw_max.set_val(key, _months_data[m]._wind_average[t]);
                    pw_min.set_val(key, 0);
                    key = name + "," + months._indices->at(m) + ",weekend," + to_string(t+1) ;
                    pw_max.set_val(key, _months_data[m]._wind_average[t]);
                    pw_min.set_val(key, 0);
                }
            }
        }
        double load_mult = 1;
        b->vbound.min = vmin;
        b->vbound.max = vmax;
        if (b->_enable_load) {
            b->_enable_load = false;
            ws = wb.sheet_by_title("LoadInput_N"+to_string(i+1)+"_P");
            clog << "Processing " << "LoadInput_N"+to_string(i+1)+"_P" << std::endl;
            row_it = ws.rows().begin();
            row_it++;//SKIP FRIST ROW
            for (auto &m: *months._indices) {
                auto row = *row_it++;
                for (unsigned t=1; t<=24; t++) {
                    key = b->_name + "," + m + ",week," + to_string(t);

                    auto pload = row[t+2].value<double>()/bMVA;
                    if (pload!=0) {
                        b->_enable_load = true;
                    }
                    pl.set_val(key, pload);
                    if (pload!=0) {
                        pl_ratio.set_val(key, 1./pload);
                    }
                    ql.set_val(key, (sqrt(1-_power_factor*_power_factor)/_power_factor)*pload);
                }
            }
            for (auto &m: *months._indices) {
                auto row = *row_it++;
                for (unsigned t=1; t<=24; t++) {
                    key = b->_name + "," + m + ",peak," + to_string(t) ;
                    auto pload = row[t+2].value<double>()/bMVA;
                    if (pload!=0) {
                        b->_enable_load = true;
                    }
                    pl.set_val(key, pload);
                    if (pload!=0) {
                        pl_ratio.set_val(key, 1./pload);
                    }
                    ql.set_val(key, (sqrt(1-_power_factor*_power_factor)/_power_factor)*pload);
                }
            }
            for (auto &m: *months._indices) {
                auto row = *row_it++;
                for (unsigned t=1; t<=24; t++) {
                    key = b->_name + "," + m + ",weekend," + to_string(t);
                    auto pload = row[t+2].value<double>()/bMVA;
                    if (pload!=0) {
                        b->_enable_load = true;
                    }
                    pl.set_val(key, pload);
                    if (pload!=0) {
                        pl_ratio.set_val(key, 1./pload);
                    }
                    ql.set_val(key, (sqrt(1-_power_factor*_power_factor)/_power_factor)*pload);
                }
            }
        }
        else {
            for (auto &m: *months._indices) {
                for (unsigned t=1; t<=24; t++) {
                    key = b->_name + "," + m + ",week," + to_string(t);
                    pl.set_val(key, 0);
                    ql.set_val(key, 0);
                }
            }
            for (auto &m: *months._indices) {
                for (unsigned t=1; t<=24; t++) {
                    key = b->_name + "," + m + ",peak," + to_string(t) ;
                    pl.set_val(key, 0);
                    ql.set_val(key, 0);
                }
            }
            for (auto &m: *months._indices) {
                for (unsigned t=1; t<=24; t++) {
                    key = b->_name + "," + m + ",weekend," + to_string(t);
                    pl.set_val(key, 0);
                    ql.set_val(key, 0);
                }
            }
        }
    }
//    clog << "Loads:\n";
//    pl.print(true);
    clog << "Reading excel file complete" << std::endl;
    return 0;
}

int PowerNet::readgrid(const char* fname) {
    double pi = 4.*atan(1.);
    string name;
    double kvb = 0;
//    int id = 0;
    unsigned index = 0;
    cout << "Loading file " << fname << endl;
    ifstream file(fname, std::ifstream::in);
    if(!file.is_open()) {
        auto new_fname = "../" + string(fname);
        file.open(new_fname);
        if(!file.is_open()) {
            throw invalid_argument("Could not open file\n");
        }
    }
    string word;
    while (word.compare("function")) {
        file >> word;
    }

    file.ignore(6);
    file >> word;
    _name = word;

//  cout << _name << endl;
    while (word.compare("mpc.baseMVA")) {
        file >> word;
    }

    file.ignore(3);
    getline(file, word,';');
    bMVA = atoi(word.c_str());
    /* Nodes data */
    while (word.compare("mpc.bus")) {
        file >> word;
    }

    getline(file, word);
    Bus* bus = NULL;
//    Bus* bus_clone= NULL;
    file >> word;
    int status;
    while(word.compare("];")) {
        name = word.c_str();
        file >> ws >> word;
        status = atoi(word.c_str());
        if (status==3) {
            ref_bus = name;
            DebugOff("Ref bus = " << ref_bus << endl);
        }
        file >> ws >> word;
        pl.set_val(name,atof(word.c_str())/bMVA);
        file >> word;
        ql.set_val(name,atof(word.c_str())/bMVA);
        file >> word;
        gs.set_val(name,atof(word.c_str())/bMVA);
        file >> word;
        bs.set_val(name,atof(word.c_str())/bMVA);
        file >> ws >> word >> ws >> word;
        v_s.set_val(name,atof(word.c_str()));
        file >> ws >> word >> ws >> word;
        kvb = atof(word.c_str());
        file >> ws >> word >> ws >> word;
        v_max.set_val(name,atof(word.c_str()));
        getline(file, word,';');
        v_min.set_val(name,atof(word.c_str()));
        w_min.set_val(name,pow(v_min(name).eval(), 2));
        w_max.set_val(name,pow(v_max(name).eval(), 2));
        // single phase

        bus = new Bus(name, pl(name).eval(), ql(name).eval(), gs(name).eval(), bs(name).eval(), v_min(name).eval(), v_max(name).eval(), kvb, 1);
//        bus_clone = new Bus(name, pl(name).eval(), ql(name).eval(), gs(name).eval(), bs(name).eval(), v_min(name).eval(), v_max(name).eval(), kvb, 1);
        bus->vs = v_s(name).eval();
//        bus_clone->vs = v_s(name).eval();
        if (status>=4) {
            bus->_active = false;
//            bus_clone->_active = false;
        }

        this->Net::add_node(bus);
        if (status>=4) {
            DebugOff("INACTIVE NODE!\n" << name << endl);
        }
        file >> word;
    }
//    ref_bus = nodes.front()->_name;
    file.seekg (0, file.beg);


    /* Generator data */
    while (word.compare("mpc.gen")) {
        file >> word;
    }
//    double qmin = 0, qmax = 0, pmin = 0, pmax = 0, ps = 0, qs = 0;
//    int status = 0;
    getline(file, word);


    file >> word;
//    std::vector<bool> gen_status;
    index = 0;
    string bus_name;
    while(word.compare("];")) {
        bus_name = word.c_str();
        // name -> node.
        bus = (Bus*)(Net::get_node(bus_name));
        name = to_string(index);
        file >> word;
        pg_s.set_val(name,atof(word.c_str())/bMVA);
        file >> word;
        qg_s.set_val(name,atof(word.c_str())/bMVA);
        file >> word;
        qg_max.set_val(name,atof(word.c_str())/bMVA);
        file >> word;
        qg_min.set_val(name,atof(word.c_str())/bMVA);

        file >> ws >> word >> ws >> word >> ws >> word;
        status = atoi(word.c_str());
        file >> word;
        pg_max.set_val(name,atof(word.c_str())/bMVA);

        file >> word;
        pg_min.set_val(name,atof(word.c_str())/bMVA);
        getline(file, word,'\n');
//        gen_status.push_back(status==1);


        bus->_has_gen = true;
        /** generator name, ID */
        Gen* g = new Gen(bus, name, pg_min.eval(index), pg_max.eval(index), qg_min.eval(index), qg_max.eval(index));
        g->_id = index;
        g->_ps = pg_s.eval();
        g->_qs = qg_s.eval();
        gens.push_back(g);
        bus->_gen.push_back(g);
        if(status!=1 || !bus->_active) {
            DebugOff("INACTIVE GENERATOR!\n" << name << endl);
            g->_active = false;
        }
        index++;
//        getline(file, word);
        file >> word;
    }


    file.seekg (0, file.beg);

    /* Generator costs */
    while (word.compare("mpc.gencost")) {
        file >> word;
    }
//    double c0 = 0, c1 = 0,c2 = 0;
    getline(file, word);

    int gen_counter = 0;
    for (int i = 0; i < gens.size(); ++i) {
        file >> ws >> word >> ws >> word >> ws >> word >> ws >> word >> ws >> word;
        c2.set_val(to_string(i),atof(word.c_str())*pow(bMVA,2));
        file >> word;
        c1.set_val(to_string(i),atof(word.c_str())*bMVA);
        file >> word;
        c0.set_val(to_string(i),atof(word.c_str()));
//        c2(i) = atof(word.c_str())*pow(bMVA,2);
//        file >> word;
//        c1(i) = atof(word.c_str())*bMVA;
//        file >> word;
//        c0(i) = atof(word.c_str());
        gens[gen_counter++]->set_costs(c0.eval(), c1.eval(), c2.eval());
        getline(file, word);
    }
    file.seekg (0, file.beg);

    /* Lines data */
    while (word.compare("mpc.branch")) {
        file >> word;
    }
    getline(file, word);
    double res = 0;
    set<string> bus_pair_names;
    Line* arc = NULL;
    string src,dest,key;
    file >> word;
    index = 0;
    while(word.compare("];")) {
        src = word;
        file >> dest;
        key = dest+","+src;//Taking care of reversed direction arcs
        if(arcID.find(key)!=arcID.end()) {//Reverse arc direction
            Debug("Adding arc linking " +src+" and "+dest);
            Debug(" with reversed direction, reversing source and destination.\n");
            key = src;
            src = dest;
            dest = key;
        }
        
        arc = new Line(to_string(index) + "," + src + "," + dest); // Name of lines
        arc->_id = index++;
        arc->_src = get_node(src);
        arc->_dest= get_node(dest);

        file >> word;
        arc->r = atof(word.c_str());
        file >> word;
        arc->x = atof(word.c_str());
        res = pow(arc->r,2) + pow(arc->x,2);

        if (res==0) {
            cerr << " line with r = x = 0" << endl;
            exit(-1);
        }
        // define g and b for each conductor.
        arc->g = arc->r/res;
        arc->b = -arc->x/res;
        file >> word;
        arc->ch = atof(word.c_str());
        file >> word;
        arc->limit = atof(word.c_str())/bMVA;

        // skip rate A rate B rate C.
        file >> ws >> word >> ws >> word >> ws >> word;
        if(atof(word.c_str()) == 0)
            arc->tr = 1.0;
        else
            arc->tr = atof(word.c_str());
        file >> ws >> word;
        arc->as = atof(word.c_str())*pi/180.;
        file >> ws >> word;


        arc->cc = arc->tr*cos(arc->as); // Rectangular values for transformer phase shifters
        arc->dd = arc->tr*sin(arc->as);
        arc->status = atoi(word.c_str());
        file >> ws >> word;

        arc->tbound.min = atof(word.c_str())*pi/180.;
//        arc->tbound.min = -30*pi/180;
        m_theta_lb += arc->tbound.min;
        file >>  ws >>word;

        arc->tbound.max = atof(word.c_str())*pi/180.;
        if (arc->tbound.min==0 && arc->tbound.max==0) {
            Debug("Angle bounds are equal to zero. Setting them to -+60");
             arc->tbound.min = -60*pi/180;
            arc->tbound.max = 60*pi/180;
            
        }
//        arc->tbound.max = 30*pi/180;
        m_theta_ub += arc->tbound.max;

        Bus* bus_s = (Bus*)(arc->_src);
        Bus* bus_d = (Bus*)(arc->_dest);

        arc->smax = max(
                        pow(bus_s->vbound.max,2)*(arc->g*arc->g + arc->b*arc->b)*(pow(bus_s->vbound.max,2) + pow(bus_d->vbound.max,2)),
                        pow(bus_d->vbound.max,2)*(arc->g*arc->g+arc->b*arc->b)*(pow(bus_d->vbound.max,2) + pow(bus_s->vbound.max,2))
                    );
        name = arc->_name;
        g.set_val(name,arc->g);
        b.set_val(name,arc->b);
        tr.set_val(name,arc->tr);
        as.set_val(name,arc->as);
        g_ff.set_val(name,arc->g/(pow(arc->cc, 2) + pow(arc->dd, 2)));
        g_ft.set_val(name,(-arc->g*arc->cc + arc->b*arc->dd)/(pow(arc->cc, 2) + pow(arc->dd, 2)));

        g_tt.set_val(name,arc->g);
        g_tf.set_val(name,(-arc->g*arc->cc - arc->b*arc->dd)/(pow(arc->cc, 2) + pow(arc->dd, 2)));


        b_ff.set_val(name,(arc->ch*0.5 + arc->b)/(pow(arc->cc, 2) + pow(arc->dd, 2)));
        b_ft.set_val(name,(-arc->b*arc->cc - arc->g*arc->dd)/(pow(arc->cc, 2) + pow(arc->dd, 2)));

        b_tt.set_val(name,(arc->ch*0.5 + arc->b));
        b_tf.set_val(name,(-arc->b*arc->cc + arc->g*arc->dd)/(pow(arc->cc, 2) + pow(arc->dd, 2)));

        Y.set_val(name,sqrt(arc->g*arc->g + arc->b*arc->b));
        if (arc->g!=0) {
            Y_t.set_val(name,atan(arc->b/arc->g));
            if(arc->b < 0) {
                Y_t.set_val(name,atan(arc->b/arc->g) - pi);
            } else {
                Y_t.set_val(name,atan(arc->b/arc->g) + pi);
            }
        }
        else {
        if(arc->b < 0){
            Y_t.set_val(name,-pi*0.5);
        } else {
            Y_t.set_val(name,pi*0.5);
        }
    }
    
    Y_charge.set_val(name,sqrt(pow(arc->g,2) + pow((arc->b+arc->ch*0.5),2)));
    if(arc->g != 0) {
        Y_charge_t.set_val(name,atan((arc->b+arc->ch*0.5)/arc->g));
    } else {
        if(arc->b < 0) {
            Y_charge_t.set_val(name,-pi*0.5);
        } else {
            Y_charge_t.set_val(name,pi*0.5);
        }
    }
        ch.set_val(name,arc->ch);
        S_max.set_val(name,arc->limit);
//        Debug("charge = " << arc->ch << endl);
//        Debug("as = " << arc->as << endl);
//        Debug("tr = " << arc->tr << endl);


        if(arc->status != 1 || !bus_s->_active || !bus_d->_active) {
            arc->_active = false;
            DebugOff("INACTIVE ARC!\n" << arc->_name << endl);
        }
        arc->connect();
        add_arc(arc);
        /* Switching to bus_pairs keys */
        name = bus_s->_name + "," + bus_d->_name;
        if (arc->_active && bus_pair_names.count(name)==0) {
            _bus_pairs._keys.push_back(new index_pair(index_(bus_s->_name), index_(bus_d->_name), arc->_active));
            bus_pair_names.insert(name);
        }
        if (!arc->_parallel) {
            th_min.set_val(name,arc->tbound.min);
            th_max.set_val(name,arc->tbound.max);
            tan_th_min.set_val(name,tan(arc->tbound.min));
            tan_th_max.set_val(name,tan(arc->tbound.max));
            
        }
        else {
            th_min.set_val(name,max(th_min.eval(name), arc->tbound.min));
            th_max.set_val(name,min(th_max.eval(name), arc->tbound.max));
            tan_th_min.set_val(name,tan(th_min.eval(name)));
            tan_th_max.set_val(name,tan(th_max.eval(name)));
        }
        if (arc->tbound.min >= 0) {
            wr_max.set_val(name,bus_s->vbound.max*bus_d->vbound.max*cos(th_min(name).eval()));
            wr_min.set_val(name,bus_s->vbound.min*bus_d->vbound.min*cos(th_max(name).eval()));
            wi_max.set_val(name,bus_s->vbound.max*bus_d->vbound.max*sin(th_max(name).eval()));
            wi_min.set_val(name,bus_s->vbound.min*bus_d->vbound.min*sin(th_min(name).eval()));
        };
        if (arc->tbound.max <= 0) {
            wr_max.set_val(name,bus_s->vbound.max*bus_d->vbound.max*cos(th_max(name).eval()));
            wr_min.set_val(name,bus_s->vbound.min*bus_d->vbound.min*cos(th_min(name).eval()));
            wi_max.set_val(name,bus_s->vbound.min*bus_d->vbound.min*sin(th_max(name).eval()));
            wi_min.set_val(name,bus_s->vbound.max*bus_d->vbound.max*sin(th_min(name).eval()));
        }
        if (arc->tbound.min < 0 && arc->tbound.max > 0) {
            wr_max.set_val(name,bus_s->vbound.max*bus_d->vbound.max);
            wr_min.set_val(name,bus_s->vbound.min*bus_d->vbound.min*min(cos(th_min(name).eval()), cos(th_max(name).eval())));
            wi_max.set_val(name,bus_s->vbound.max*bus_d->vbound.max*sin(th_max(name).eval()));
            wi_min.set_val(name,bus_s->vbound.max*bus_d->vbound.max*sin(th_min(name).eval()));
        }
        cphi.set_val(name, cos(0.5*(arc->tbound.min+arc->tbound.max)));
        sphi.set_val(name, sin(0.5*(arc->tbound.min+arc->tbound.max)));
        cos_d.set_val(name, cos(0.5*(arc->tbound.max-arc->tbound.min)));
        getline(file, word,'\n');
        file >> word;
    }
    DebugOff(ch.to_str(true) << endl);
    DebugOff(as.to_str(true) << endl);
    DebugOff(tr.to_str(true) << endl);

    file.close();
    if (nodes.size()>2000) {
        add_3d_nlin = false;
    }
    return 0;
}

/* Create imaginary lines, fill bus_pairs_chord, set lower and upper bounds */
void PowerNet::update_net(){
    string name;
    double cos_max_, cos_min_, sin_max_, sin_min_;
    double wr_max_, wr_min_, wi_max_, wi_min_, w_max_, w_min_;
    Node *src, *dest, *n;
    Arc *new_arc;
    int fixed = 1, id_sorted = 0; //id of the current bag in bags_sorted
    Arc *a12, *a13, *a32;
    std::vector<std::vector<Node*>> bags_sorted;

    // bags are cliques in the chordal completion graph
    for(auto& b: _bags){
        for(int i = 0; i < b.size()-1; i++) {
            for(int j = i+1; j < b.size(); j++) {
                Arc* a = get_arc(b[i]->_name,b[j]->_name);
                if (a==nullptr) {
                    src = get_node(b[i]->_name);
                    dest = get_node(b[j]->_name);
                    new_arc = new Line(to_string((int) arcs.size() + 1));
                    new_arc->_id = arcs.size();
                    new_arc->_src = src;
                    new_arc->_dest = dest;
                    new_arc->_active = false;
                    new_arc->_imaginary = true;
                    new_arc->_free = true;
                    new_arc->connect();
                    add_undirected_arc(new_arc);
                }
            }
        }
    }

    while (fixed != 0) {
        fixed = 0;
        DebugOff("\nNew iteration");
        for(auto b_it = _bags.begin(); b_it != _bags.end();) {
            std::vector<Node*> b = *b_it;
            if(b.size() == 3) {
                DebugOff("\nBag: " << b[0]->_name << ", " << b[1]->_name << ", " << b[2]->_name);
                a12 = get_arc(b[0], b[1]);
                a13 = get_arc(b[0], b[2]);
                a32 = get_arc(b[2], b[1]);
                if ((a12->_free && a13->_free) || (a12->_free && a32->_free) || (a13->_free && a32->_free) ||
                    (!a12->_free && !a13->_free && !a32->_free)) { // at least two missing lines or all lines real
                    ++b_it;
                    continue;
                }
                if (a12->_free) {
                    a12->_free = false;
                    DebugOff("\nFixing arc a12 (" << a12->_src->_name << ", " << a12->_dest->_name << "), adding bag #" << id_sorted);
                    fixed++;
                }
                if (a13->_free) {
                    a13->_free = false;
                    DebugOff("\nFixing arc a13 (" << a13->_src->_name << ", " << a13->_dest->_name << "), adding bag #" << id_sorted);
                    fixed++;
                }
                if (a32->_free) {
                    a32->_free = false;
                    DebugOff("\nFixing arc a32 (" << a32->_src->_name << ", " << a32->_dest->_name << "), adding bag #" << id_sorted);
                    fixed++;
                }
                bags_sorted.push_back(b);
                _bags.erase(b_it);
                id_sorted++;
            }
            else{ // Bags with size > 3; todo: leave only this as the general case?
                DebugOff("\nBag with size > 3");

                for(int i = 0; i < b.size()-1; i++) {
                    for (int j = i + 1; j < b.size(); j++) {
                        Arc* a = get_arc(b[i]->_name, b[j]->_name);
                        if (!a->_free) continue;
                        n = a->_src;
                        //by now, all arcs in bags should be created
                        for (auto n1: b) {
                            if(n==n1) continue;
                            Arc* a2 = get_arc(n->_name, n1->_name);
                            if (a2->_free) continue;
                            Arc *a1 = get_arc(a->_dest, n1);
                            if (!a1->_free) {
                                a->_free = false;

                                vector<Node *> bag;
                                bag.push_back(get_node(n->_name));
                                bag.push_back(get_node(a->_dest->_name));
                                bag.push_back(get_node(n1->_name));
//                                sort(bag.begin(), bag.end(),
//                                     [](const Node *a, const Node *b) -> bool { return a->_id < b->_id; });

                                fixed++;
                                sort(bag.begin(), bag.end(), [](const Node* a, const Node* b) -> bool{return a->_id < b->_id;});
                                bags_sorted.push_back(bag);
                                id_sorted++;
                                DebugOff("\nFixing arc in a larger bag (" << a->_src->_name << ", " << a->_dest->_name << ")");
                                break;
                            }
                        }
                    } // j
                } // i
                ++b_it;

            } // size > 3
        } // bags loop
    } // while

    //add all remaining bags to bags_sorted
    for(auto b_it = _bags.begin(); b_it != _bags.end();) {
        std::vector<Node*> b = *b_it;
            if(b.size() > 2) bags_sorted.push_back(b);
            _bags.erase(b_it);
//            id_sorted++;
    }
    _bags = bags_sorted;

    for(auto& a: arcs) {
        if(a->_imaginary) a->_free = true;
    }


    for(auto& k: _bus_pairs._keys){
        _bus_pairs_chord._keys.push_back(new index_pair(*k));
    }

    for(auto& a: arcs){
        if(a->_imaginary){
            Bus* bus_s = (Bus*)(a->_src);
            Bus* bus_d = (Bus*)(a->_dest);

            name = bus_s->_name + "," + bus_d->_name;
            _bus_pairs_chord._keys.push_back(new index_pair(index_(bus_s->_name), index_(bus_d->_name)));

            if (m_theta_lb < -3.14 && m_theta_ub > 3.14) {
                cos_max_ = 1;
                cos_min_ = -1;
            } else if (m_theta_lb < 0 && m_theta_ub > 0){
                cos_max_ = 1;
                cos_min_ = min(cos(m_theta_lb), cos(m_theta_ub));
            } else{
                cos_max_ = max(cos(m_theta_lb),cos(m_theta_ub));
                cos_min_ = min(cos(m_theta_lb), cos(m_theta_ub));
            }
            w_max_ = bus_s->vbound.max*bus_d->vbound.max;
            w_min_ = bus_s->vbound.min*bus_d->vbound.min;

            wr_max_ = cos_max_*w_max_;
            if(cos_min_ < 0) wr_min_ = cos_min_*w_max_;
            else wr_min_ = cos_min_*w_min_;

            if(m_theta_lb < -1.57 && m_theta_ub > 1.57){
                sin_max_ = 1;
                sin_min_ = -1;
            } else{
                sin_max_ = sin(m_theta_ub);
                sin_min_ = sin(m_theta_lb);
            }

            if(sin_max_ > 0) wi_max_ = sin_max_*w_max_;
            else wi_max_ = sin_max_*w_min_;
            if(sin_min_ > 0) wi_min_ = sin_min_*w_min_;
            else wi_min_ = sin_min_*w_max_;

//            cout << "\nImaginary line, bounds: (" << wr_min_ << "," << wr_max_ << "); (" << wi_min_ << "," << wi_max_ << ")";

            wr_max.set_val(name,wr_max_);
            wr_min.set_val(name,wr_min_);
            wi_max.set_val(name,wi_max_);
            wi_min.set_val(name,wi_min_);
        }
    }
    DebugOff("\nBags sorted: " << endl);
    for(auto& b: _bags) {
        DebugOff("bag = {");
        for (int i = 0; i < b.size(); i++) {
            DebugOff(b.at(i)->_name << " ");
        }
        DebugOff("}" << endl);
        if(add_3d_nlin && b.size()==3){
            for(int i = 0; i < 2; i++) {
                for(int j = i+1; j < 3; j++) {
                    Arc* aij = get_arc(b[i],b[j]);
                    aij->_free = false;
                }
            }
        }
    }
}

//void PowerNet::update_status(unique_ptr<Model> model){
//    for (auto &v_p:model->_vars) {
//        if (v_p.second->is_integer() || v_p.second->is_binary()) {
//            auto v = v_p.second;
//            auto new_v = new var<Real>(v_p.second->_name, 0,1);
//            new_v->copy(*v);
//            new_v->_is_relaxed = true;
//            new_v->_val->resize(new_v->get_dim());
//            if (v->get_intype()==integer_) {
//                auto real_var = (var<int>*)v;
//                for (int i = 0; i < real_var->get_dim(); i++) {
//                    new_v->_val->at(i) = real_var->_val->at(i);
//                }
//            }
//            if (v->get_intype()==short_) {
//                auto real_var = (var<short>*)v;
//                for (int i = 0; i < real_var->get_dim(); i++) {
//                    new_v->_val->at(i) = real_var->_val->at(i);
//                }
//            }
//            if (v->get_intype()==binary_) {
//                auto real_var = (var<bool>*)v;
//                _model->_bin_vars[v_p.second->get_vec_id()] = *real_var;
//                for (int i = 0; i < real_var->get_dim(); i++) {
//                    new_v->_val->at(i) = real_var->_val->at(i);
//                }
//            }
//            v_p.second = new_v;
//        }
//    }
//}


shared_ptr<Model> PowerNet::build_ROMDST_contingency(const string& name, PowerModelType pmt, int output, double tol, int max_nb_hours){
    
    
    auto new_Et = Et_opt;
    auto new_Gt = indices(gens,T);
    auto new_Bt = Bt_opt;
    auto new_Bt1 = Bt1_opt; /**< Excluding first hour */
    auto new_PVt = PVt_opt;
    
    /** MODEL DECLARATION */
    shared_ptr<Model> ROMDST(new Model(name));
    /** VARIABLES */
    
    
    /* Diesel power generation variables */
    var<Real> Pg("Pg", pg_min, pg_max);
    var<Real> Qg ("Qg", qg_min, qg_max);
    var<Real> Pg_ ("Pg_", pg_min, pg_max);/**< Active power generation before losses */
    ROMDST->add(Pg.in(new_Gt),Pg_.in(new_Gt),Qg.in(new_Gt));
    
    
    /* Battery power generation variables */
    var<Real> Pb("Pb", pb_min, pb_max);/**< Active power generation outside the battery */
    var<Real> Pb_diff("Pb_diff", pos_);/**< Difference in active power generation outside and inside the battery */
    var<Real> Qb ("Qb", qb_min, qb_max);/**< Reactive power generation outside the battery */
    var<Real> Pb_("Pb_", pb_min, pb_max);/**< Active power generation in the battery */
//    var<Real> Qb_("Qb_", qb_min, qb_max);/**< Reactive power generation in the battery */
    ROMDST->add(Pb.in(new_Bt), Qb.in(new_Bt), Pb_.in(new_Bt), Pb_diff.in(new_Bt));
    
    
    /* PV power generation variables */
    var<Real> Pv("Pv", pos_);
    ROMDST->add(Pv.in(new_PVt));
    
    /* Battery state of charge variables */
    var<Real> Sc("Sc", pos_);
    ROMDST->add(Sc.in(new_Bt));
    
    /* Wind power generation variables */
    var<Real> Pw("Pw", pw_min, pw_max);
    //    pw_max.print(true);
    ROMDST->add(Pw.in(Wt));
    
    /* Power flow variables */
    var<Real> Pij("Pfrom", S_max);
    var<Real> Qij("Qfrom", S_max);
    var<Real> Pji("Pto", S_max);
    var<Real> Qji("Qto", S_max);
    
    var<Real> P_shed("P_shed", pos_);
    var<Real> P_shed_max("P_shed_max", pos_);
//    var<Real> Q_shed("Q_shed", pos_);
    
    ROMDST->add(Pij.in(new_Et),Qij.in(new_Et), P_shed.in(Nt), P_shed_max.in(R(1)));
    if (pmt!=LDISTF) {
        ROMDST->add(Pji.in(new_Et),Qji.in(new_Et));
    }
    
    /** Voltage magnitude (squared) variables */
    var<Real> v("v", vmin, vmax);
    ROMDST->add(v.in(Nt));
    v.initialize_all(0.64);
    
    /** Power loss variables */
    var<Real> loss("loss", pos_);
    if (pmt==DISTF || pmt==CDISTF) {
        ROMDST->add(loss.in(Nt));
    }
    
    /** OBJECTIVE FUNCTION */
    ROMDST->min(100*P_shed_max);
    
    
    /** CONSTRAINTS **/
    
/** Max Load Shed **/
    
    Constraint max_load_shed("max_load_shed");
//    max_load_shed = bMVA*P_shed/(pl*bMVA-1e-5) - P_shed_max;//TODO fix this
    max_load_shed = P_shed*pl_ratio - P_shed_max;
    ROMDST->add(max_load_shed.in(Nt_load)<=0);
//    ROMDST->get_constraint("max_load_shed")->print_expanded();

    Constraint no_load_shed("no_load_shed");
    no_load_shed = P_shed;
    ROMDST->add(no_load_shed.in(Nt_no_load)==0);
//    ROMDST->get_constraint("no_load_shed")->print_expanded();

/** FLOW CONSERVATION **/
    
    /** KCL P and Q */
    Constraint KCL_P("KCL_P");
    Constraint KCL_Q("KCL_Q");
    if (pmt==LDISTF) {
        KCL_P  = sum(Pij.out_arcs()) - sum(Pij.in_arcs()) + pl - sum(Pg.in_gens()) - sum(Pb.in_bats()) - sum(Pw.in_wind()) - sum(Pv.in_pv()) - P_shed;
        //        KCL_P  = sum(Pij.out_arcs()) - sum(Pij.in_arcs()) + pl - sum(Pg.in_gens()) - sum(Pb.in_bats()) - sum(Pw.in_wind());
        KCL_Q  = sum(Qij.out_arcs()) - sum(Qij.in_arcs()) + ql - sum(Qg.in_gens()) - sum(Qb.in_bats());
    }
    else{
        KCL_P  = sum(Pij.out_arcs()) + sum(Pji.in_arcs()) + pl - sum(Pg.in_gens())- sum(Pb.in_bats()) - sum(Pw.in_wind()) - sum(Pv.in_pv()) - P_shed;
        KCL_Q  = sum(Qij.out_arcs()) + sum(Qji.in_arcs()) + ql - sum(Qg.in_gens()) - sum(Qb.in_bats());
    }
    ROMDST->add(KCL_P.in(nodes, T) == 0);
    ROMDST->add(KCL_Q.in(nodes, T) == 0);
    
/**  THERMAL LIMITS **/
    
    
    
    /*  Thermal Limit Constraints */
    Constraint Thermal_Limit_from("Thermal_Limit_from");
    Thermal_Limit_from += power(Pij, 2) + power(Qij, 2);
    Thermal_Limit_from -= power(S_max, 2);
    ROMDST->add(Thermal_Limit_from.in(new_Et) <= 0);
    
    
    
/**  POWER FLOW **/
    
    /** Power Flow Constraints */
    Constraint Flow_P_From("Flow_P_From");
    Flow_P_From = v.from() - 2*(r*Pij + x*Qij) - v.to();
    ROMDST->add(Flow_P_From.in(arcs, T)==0);
    
    
    /**  PV **/
    
    /*  On/Off on newly installed PV */
    Constraint OnOffPV("OnOffPV");
    OnOffPV += Pv - Pv_cap_*pv_out;
    ROMDST->add(OnOffPV.in(new_PVt) <= 0);
    
    
/**  BATTERIES **/
    
    /*  Apparent Power Limit on existing and installed Batteries */
    Constraint Apparent_Limit_Batt_Pot("Apparent_Limit_Batt");
    Apparent_Limit_Batt_Pot += power(Pb, 2) + power(Qb, 2);
    Apparent_Limit_Batt_Pot -= power(pb_max, 2);
    ROMDST->add(Apparent_Limit_Batt_Pot.in(new_Bt) <= 0);
    
    /*  State Of Charge */
    Constraint State_Of_Charge("State_Of_Charge");
    State_Of_Charge = Sc - Sc.prev() + Pb_;
    ROMDST->add(State_Of_Charge.in(new_Bt1) == 0);
    
    /*  State Of Charge 0 */
    auto Bat0 = indices(_battery_inverters,T.start());
    Constraint State_Of_Charge0("State_Of_Charge0");
    State_Of_Charge0 = Sc;
    ROMDST->add(State_Of_Charge0.in(Bat0) == 0);
    Constraint Pb0("Pb0");
    Pb0 = Pb;
    ROMDST->add(Pb0.in(Bat0) == 0);
    
    /*  Efficiencies */
    Constraint DieselEff("DieselEff");
    DieselEff += Pg - gen_eff*Pg_;
    ROMDST->add(DieselEff.in(new_Gt) == 0);
    
    Constraint EfficiencyExist("BatteryEfficiency");
    EfficiencyExist += Pb  - eff_a*Pb_ - eff_b;//TODO without time extending eff_a and eff_b
    ROMDST->add(EfficiencyExist.in(indices(_eff_pieces,new_Bt)) <= 0);
    
//    Constraint PbDiff_P("PbDiff_P");
//    PbDiff_P += Pb - Pb_ - Pb_diff;
//    ROMDST->add(PbDiff_P.in(new_Bt) <= 0);
//    
//    Constraint PbDiff_N("PbDiff_N");
//    PbDiff_N += Pb_ - Pb - Pb_diff;
//    ROMDST->add(PbDiff_N.in(new_Bt) <= 0);
    
//    ROMDST->_obj -= *ROMDST->get_constraint("BatteryEfficiency");

/*  GENERATOR RAMP UP/DOWN */
    Constraint RampDown("RampDown");
    RampDown +=  Pg_ - (Pg_base - ramp_down*pg_max);
    ROMDST->add(RampDown.in(new_Gt) >= 0);
    
    Constraint RampUp("RampUp");
    RampUp +=  Pg_ - (Pg_base + ramp_up*pg_max);
    ROMDST->add(RampUp.in(new_Gt) <= 0);
    
    return ROMDST;
}

void PowerNet::fix_investment(){
    auto vals = w_g.get_vals();
    for (auto i = 0; i<vals->size(); i++) {
        if (!vals->at(i)) {//This generator was not built
            clog << "Excluding Generator: " << _potential_diesel_gens[i]->_name << endl;
            _potential_diesel_gens[i]->_active = false;
        }
    }
    vals = w_b.get_vals();
    for (auto i = 0; i<vals->size(); i++) {
        if (!vals->at(i)) {//This battery was not built
            clog << "Excluding Battery: " << _potential_battery_inverters[i]->_name << endl;
            _potential_battery_inverters[i]->_active = false;
        }
    }
    vals = w_pv.get_vals();
    for (auto i = 0; i<vals->size(); i++) {
        if (!vals->at(i)) {//This PV was not built
            clog << "Excluding PV: " << _potential_PV_gens[i]->_name << endl;
            _potential_PV_gens[i]->_active = false;
        }
    }
    vals = w_e.get_vals();
    for (auto i = 0; i<vals->size(); i++) {
        if (!vals->at(i)) {//This edge was not built
            clog << "Excluding Edge Expansion: " << _potential_expansion[i]->_name << endl;
            _potential_expansion[i]->_active = false;
        }
    }
    Pv_cap_ = Pv_cap;
    for (auto pv:_existing_PV_gens) {
        Pv_cap_(pv->_name) = pv->_max_cap;
    }
    Pg_base = Pg_;
    Et_opt = indices(arcs, T);
    Gt_opt = indices(gens, T);
    Bt_opt = indices(_battery_inverters, T);
    Bt1_opt = indices(_battery_inverters,T.exclude(T.start())); /**< Excluding first hour */
    PVt_opt = indices(_all_PV_gens,T);
}

unique_ptr<Model> PowerNet::build_ODO_model(PowerModelType pmt, int output, double tol, int max_nb_hours){
    /** Indices Sets */
    hours = time(1,max_nb_hours); /**< Hours */
    //        indices months = time("jan","feb","mar","apr","may","jun","jul","aug","sep","oct","nov","dec"); /**< Months */
    //    indices months = time("jan","feb","mar","apr","may","jun"); /**< Months */
    //    months = time("apr", "aug", "dec"); /**< Months */
    indices months = time("jan");
    indices phases = indices("ph0","ph1","ph2");
    //        indices months = time("jan", "feb", "mar","apr","may","jun");
    //    typical_days = time("week","peak","weekend");
    typical_days = time("week");
    T = indices(months,typical_days,hours);
    double nT = T.size();
    DebugOn("number of time periods = " << nT << endl);
    time_expand(T);
    Nt = indices(nodes,phases,T);
    Nt_load._time_extended = true;
    Nt_no_load._time_extended = true;
//    for (auto &nt:*Nt._indices) {
//        if (pl.eval(nt)!=0) {
//            Nt_load.add(nt);
//        }
//        else{
//            Nt_no_load.add(nt);
//        }
//    }
    Et = indices(arcs,phases,T);
    Et_ph = indices(Et_ph,T);
    Gt = indices(gens,phases,T);
    exist_Gt = indices(_existing_diesel_gens,phases,T);
    exist_Bt = indices(_existing_battery_inverters,phases,T);
    exist_Et = indices(_exist_arcs,phases,T);
    pot_Gt = indices(_potential_diesel_gens,phases,T);
    pot_Bt = indices(_potential_battery_inverters,phases,T);
    pot_Et = indices(_potential_expansion,phases,T);
    Bt = indices(_battery_inverters,phases,T);
    auto T1 = T.exclude(T.start());/**< Excluding first time step */
    Bt1 = indices(_battery_inverters,phases,T1);
    Gt1 = indices(gens,phases,T1);
    Wt = indices(_all_wind_gens,phases,T);
    PVt = indices(_all_PV_gens,phases,T);
    PV_pot_t = indices(_potential_PV_gens,phases,T);
    pot_gen = indices(_potential_diesel_gens);
    pot_batt = indices(_potential_battery_inverters);
    pot_edges = indices(_potential_expansion);
    
    
    /** MODEL DECLARATION */
    unique_ptr<Model> ODO(new Model("ODO Model"));
    /** VARIABLES */
    
    
    /* Investment binaries */
    
    //    var<Real> Pv_cap("Pv_cap", 0, pv_max); /**< Real variable indicating the extra capacity of PV to be installed on bus b */
    var<Real> Pv_cap("Pv_cap", pos_); /**< Real variable indicating the extra capacity of PV to be installed on bus b */
    auto pot_pv = indices(_potential_PV_gens);
    ODO->add(Pv_cap.in(pot_pv));
    //        var<Real> w_g("w_g",0,1); /**< Binary variable indicating if generator g is built on bus */
    //        var<Real> w_b("w_b",0,1); /**< Binary variable indicating if battery b is built on bus */
    //        var<Real> w_e("w_e",0,1); /**< Binary variable indicating if expansion is selected for edge e */
    //        var<Real> w_pv("w_pv",0,1); /**< Binary variable indicating if expansion is selected for edge e */
    //        w_g._is_relaxed = true;
    //        w_b._is_relaxed = true;
    //        w_e._is_relaxed = true;
    //        w_pv._is_relaxed = true;
    
    var<bool> w_g("w_g"); /**< Binary variable indicating if generator g is built on bus */
    var<bool> w_b("w_b"); /**< Binary variable indicating if battery b is built on bus */
    var<bool> w_e("w_e"); /**< Binary variable indicating if expansion is selected for edge e */
    var<bool> w_pv("w_pv"); /**< Binary variable indicating if PV is installed on bus b */
    
    
    
    ODO->add(w_g.in(pot_gen),w_b.in(pot_batt),w_e.in(pot_edges),w_pv.in(pot_pv));
    w_g.initialize_all(1);
    w_b.initialize_all(1);
    w_e.initialize_all(1);
    w_pv.initialize_all(1);
    
    this->w_g = w_g;
    this->w_b = w_b;
    this->w_e = w_e;
    this->w_pv = w_pv;
    this->Pv_cap = Pv_cap;
    
    DebugOn("size w_g = " << w_g.get_dim() << endl);
    DebugOn("size w_b = " << w_b.get_dim() << endl);
    DebugOn("size w_e = " << w_e.get_dim() << endl);
    DebugOn("size w_pv = " << w_pv.get_dim() << endl);
    DebugOn("size Pv_cap = " << Pv_cap.get_dim() << endl);
    
    
    /* Diesel power generation variables */
    var<Real> Pg("Pg", pg_min, pg_max);
    var<Real> Qg ("Qg", qg_min, qg_max);
//    var<Real> Pg("Pg");
//    var<Real> Qg ("Qg");
    var<Real> Pg_ ("Pg_", pg_min, pg_max);/**< Active power generation before losses */
    //    var<Real> Pg2("Pg2", 0, power(pg_max,2));/**< Square of Pg */
    var<Real> Pg2("Pg2", pos_);/**< Square of Pg */
    ODO->add(Pg.in(Gt));
    ODO->add(Pg_.in(Gt));
    ODO->add(Qg.in(Gt));
    ODO->add(Pg2.in(pot_Gt));
    DebugOn("size Pg = " << Pg.get_dim() << endl);
    DebugOn("size Pg_ = " << Pg_.get_dim() << endl);
    DebugOn("size Qg = " << Qg.get_dim() << endl);
    DebugOn("size Pg2 = " << Pg2.get_dim() << endl);
    //    Pg.initialize_all(0.001);
    //    Qg.initialize_all(0.0001);
    
    this->Pg_ = Pg_;
    
    /* Battery power generation variables */
    var<Real> Pb("Pb", pb_min, pb_max);/**< Active power generation outside the battery */
    var<Real> Qb ("Qb", qb_min, qb_max);/**< Reactive power generation outside the battery */
    var<Real> Pb_("Pb_", pb_min, pb_max);/**< Active power generation in the battery */
    //    var<Real> Qb_("Qb_", qb_min, qb_max);/**< Reactive power generation in the battery */
    ODO->add(Pb.in(Bt), Qb.in(Bt), Pb_.in(Bt));
    DebugOn("size Pb = " << Pb.get_dim() << endl);
    DebugOn("size Qb = " << Qb.get_dim() << endl);
    
    
    /* PV power generation variables */
    //    var<Real> Pv("Pv", 0,pv_max);
    var<Real> Pv("Pv", pos_);
    ODO->add(Pv.in(PVt));
    DebugOn("size Pv = " << Pv.get_dim() << endl);
    
    /* Battery state of charge variables */
    var<Real> Sc("Sc", pos_);
    ODO->add(Sc.in(Bt));
    DebugOn("size Sc = " << Sc.get_dim() << endl);
    
    /* Wind power generation variables */
    var<Real> Pw("Pw", pw_min, pw_max);
    //    pw_max.print(true);
    ODO->add(Pw.in(Wt));
    DebugOn("size Pw = " << Pw.get_dim() << endl);
    
    /* Power flow variables */
    var<Real> Pij("Pfrom", S_max);
    var<Real> Qij("Qfrom", S_max);
    var<Real> Pji("Pto", S_max);
    var<Real> Qji("Qto", S_max);
    
    ODO->add(Pij.in(Et),Qij.in(Et));
    DebugOn("size Pij = " << Pij.get_dim() << endl);
    if (pmt!=LDISTF) {
        ODO->add(Pji.in(Et),Qji.in(Et));
    }
    
    /** Voltage magnitude (squared) variables */
    var<Real> v("v", v_min, v_max);
    ODO->add(v.in(Nt));
    DebugOn("size v = " << v.get_dim() << endl);
    //    v.initialize_all(0.64);
    
    /** Power loss variables */
    var<Real> loss("loss", pos_);
    if (pmt==DISTF || pmt==CDISTF) {
        ODO->add(loss.in(Nt));
    }
    
    /** OBJECTIVE FUNCTION */
    func_ obj = product(c1.in(exist_Gt), Pg_.in(exist_Gt)) + product(c1.in(pot_Gt), Pg_.in(pot_Gt)) + product(c2.in(exist_Gt), power(Pg_.in(exist_Gt),2)) + product(c2.in(pot_Gt), Pg2.in(pot_Gt)) + sum(c0.in(exist_Gt));
    //    obj *= 12./months.size();
    obj += nT*product(c0.in(pot_gen),w_g.in(pot_gen));
    obj += product(gen_capcost.in(pot_gen), w_g.in(pot_gen));
    obj += product(inverter_capcost.in(pot_batt), w_b.in(pot_batt));
    obj += product(expansion_capcost.in(pot_edges), w_e.in(pot_edges));
    obj += product(pv_capcost.in(pot_pv), w_pv.in(pot_pv));
    obj += product(pv_varcost.in(pot_pv), Pv_cap.in(pot_pv));
    obj *= 1e-3;
    ODO->min(obj);
    obj.print_expanded();
    
    /** CONSTRAINTS **/
    
    /** Voltage at source bus **/
    Constraint fix_voltage("fix_voltage");
    fix_voltage += v - vm_;
    ODO->add(fix_voltage.in(indices(indices(ref_bus),phases,T))==0);
    
    /** FLOW CONSERVATION **/
    
    /** KCL P and Q */
    Constraint KCL_P("KCL_P");
    Constraint KCL_Q("KCL_Q");
    if (pmt==LDISTF) {
        KCL_P  = sum(Pij.out_arcs()) - sum(Pij.in_arcs()) + pl  + gs_*v*v - sum(Pg.in_gens()) - sum(Pb.in_bats()) - sum(Pw.in_wind()) - sum(Pv.in_pv());
        //        KCL_P  = sum(Pij.out_arcs()) - sum(Pij.in_arcs()) + pl - sum(Pg.in_gens()) - sum(Pb.in_bats()) - sum(Pw.in_wind());
        KCL_Q  = sum(Qij.out_arcs()) - sum(Qij.in_arcs()) + ql  - bs_*v*v - sum(Qg.in_gens()) - sum(Qb.in_bats());
    }
    else{
        KCL_P  = sum(Pij.out_arcs()) + sum(Pji.in_arcs()) + pl - sum(Pg.in_gens())- sum(Pb.in_bats()) - sum(Pw.in_wind()) - sum(Pv.in_pv());
        KCL_Q  = sum(Qij.out_arcs()) + sum(Qji.in_arcs()) + ql - sum(Qg.in_gens()) - sum(Qb.in_bats());
    }
    ODO->add(KCL_P.in(nodes, indices(phases,T)) == 0);
    ODO->add(KCL_Q.in(nodes, indices(phases,T)) == 0);
    
    /**  THERMAL LIMITS **/
    
    /*  Thermal Limit Constraints */
    Constraint Thermal_Limit_from("Thermal_Limit_from");
    Thermal_Limit_from += power(Pij, 2) + power(Qij, 2);
    Thermal_Limit_from -= power(S_max, 2);
    ODO->add(Thermal_Limit_from.in(Et_ph) <= 0);

    Constraint Thermal_Limit_to("Thermal_Limit_to");
    Thermal_Limit_to += power(Pji, 2) + power(Qji, 2);
    Thermal_Limit_to -= power(S_max, 2);
//    ODO->add(Thermal_Limit_to.in(exist_Et) <= 0);

    
    /*  Thermal Limit Constraints for expansion edges */
    Constraint Thermal_Limit_from_exp("Thermal_Limit_From_Exp");
    Thermal_Limit_from_exp += power(Pij, 2) + power(Qij, 2);
    Thermal_Limit_from_exp -= power(w_e,2)*power(S_max, 2);
//    ODO->add(Thermal_Limit_from_exp.in(pot_Et) <= 0);
//        ODO->get_constraint("Thermal_Limit_From_Exp")->print_expanded();
    
    /**  GENERATOR INVESTMENT **/
    
    /*  On/Off status */
    Constraint OnOff_maxP("OnOff_maxP");
    OnOff_maxP += Pg_ - pg_max*w_g;
    ODO->add(OnOff_maxP.in(pot_Gt) <= 0);
    
    Constraint Perspective_OnOff("Perspective_OnOff");
    Perspective_OnOff += power(Pg_,2) - Pg2*w_g;
    ODO->add(Perspective_OnOff.in(pot_Gt) <= 0);
    
    Constraint OnOff_maxQ("OnOff_maxQ");
    OnOff_maxQ += Qg - qg_max*w_g;
    ODO->add(OnOff_maxQ.in(pot_Gt) <= 0);
    
    Constraint OnOff_maxQ_N("OnOff_maxQ_N");
    OnOff_maxQ_N += Qg + qg_max*w_g;
    ODO->add(OnOff_maxQ_N.in(pot_Gt) >= 0);
    
    /**  POWER FLOW **/
    
    /** Power Flow Constraints */
    /*
    for(auto a: arcs){
        for (auto i = 0; i<3; i++) {
            if (a->_phases.count(i)>0) {
                for (auto &t:*T._indices) {
                    auto ph = a->_name+",ph"+to_string(i);
                    auto ph_t = ph+","+t;
                    //Constraint Flow_P_From("Flow_P_From_"+ph_t);
                   // Flow_P_From = v(a->_src->_name+",ph"+to_string(i)+","+t) - 2*(r(ph)*Pij(ph_t) + x(ph)*Qij(ph_t)) - v(a->_dest->_name+",ph"+to_string(i)+","+t);
                    //ODO->add(Flow_P_From==0);
                    auto src = a->_src->_name+",ph"+to_string(i);
                    auto dest = a->_dest->_name+",ph"+to_string(i);
                    Constraint Flow_P_From("Flow_P_From_"+ph_t);
                    Flow_P_From = g_fr(v(a->_src->_name+",ph"+to_string(i)+","+t) - 2*(r(ph)*Pij(ph_t) + x(ph)*Qij(ph_t)) - v(a->_dest->_name+",ph"+to_string(i)+","+t);
                    ODO->add(Flow_P_From==0);
                }
            }
        }
    }*/
    /* (g_fr[c]+g[c,c]) * vm_fr[c]^2 +
     sum( g[c,d]*vm_fr[c]*vm_fr[d]*cos(va_fr[c]-va_fr[d]) +
     b[c,d]*vm_fr[c]*vm_fr[d]*sin(va_fr[c]-va_fr[d]) for d in PMs.conductor_ids(pm) if d != c) +
     sum(-g[c,d]*vm_fr[c]*vm_to[d]*cos(va_fr[c]-va_to[d]) +
     -b[c,d]*vm_fr[c]*vm_to[d]*sin(va_fr[c]-va_to[d]) for d in PMs.conductor_ids(pm)) )
     */
//    Constraint Flow_P_From("Flow_P_From");
//    Flow_P_From = v.from() - 2*(r*Pij + x*Qij) - v.to();
//    ODO->add(Flow_P_From.in(_exist_arcs, indices(phases,T))==0);
//    Constraint Flow_P_to("Flow_P_Fto");
//    Flow_P_to = v.to() - 2*(r*Pji + x*Qji) - v.from();
//    ODO->add(Flow_P_to.in(_exist_arcs, indices(phases,T))==0);
//
//    Constraint Flow_P_Expansion_L("Flow_P_Expansion_L");
//    Flow_P_Expansion_L = v.from() - 2*(r*Pij + x*Qij) - v.to() - (1-w_e)*(v_diff_max);
//    ODO->add(Flow_P_Expansion_L.in(_potential_expansion, T)<=0);
//
//    Constraint Flow_P_Expansion_U("Flow_P_Expansion_U");
//    Flow_P_Expansion_U = v.from() - 2*(r*Pij + x*Qij) - v.to() + (1-w_e)*(v_diff_max);
//    ODO->add(Flow_P_Expansion_U.in(_potential_expansion, T)>=0);
    
    
    /**  PV **/
    
    /*  On/Off on Potential PV */
    Constraint OnOffPV("OnOffPV");
    OnOffPV += Pv_cap - w_pv*pv_max;
    ODO->add(OnOffPV.in(indices(_potential_PV_gens)) <= 0);
//        ODO->get_constraint("OnOffPV")->print_expanded();
    
    /*  Max Cap on Potential PV */
    Constraint MaxCapPV("MaxCapPV");
    MaxCapPV += Pv - Pv_cap*pv_out;
    ODO->add(MaxCapPV.in(PV_pot_t) <= 0);
//        ODO->get_constraint("MaxCapPV")->print_expanded();
    
    /*  Existing PV */
    Constraint existPV("existPV");
    existPV += Pv - pv_max*pv_out;
    ODO->add(existPV.in(indices(_existing_PV_gens, T)) <= 0);
//        ODO->get_constraint("existPV")->print_expanded();
    
    
    /**  BATTERIES **/
    
    /*  Apparent Power Limit on Potential Batteries */
    Constraint Apparent_Limit_Batt_Pot("Apparent_Limit_Batt_Potential");
    Apparent_Limit_Batt_Pot += power(Pb, 2) + power(Qb, 2);
    Apparent_Limit_Batt_Pot -= power(w_b,2)*power(pb_max, 2);
    ODO->add(Apparent_Limit_Batt_Pot.in(pot_Bt) <= 0);
    
    /*  Apparent Power Limit on Existing Batteries */
    Constraint Apparent_Limit_Batt("Apparent_Limit_Batt_Existing");
    Apparent_Limit_Batt += power(Pb, 2) + power(Qb, 2);
    Apparent_Limit_Batt -= power(pb_max, 2);
    ODO->add(Apparent_Limit_Batt.in(exist_Bt) <= 0);
    
    
    /*  State Of Charge */
    Constraint State_Of_Charge("State_Of_Charge");
    State_Of_Charge = Sc - Sc.prev() + Pb_;
    ODO->add(State_Of_Charge.in(Bt1) == 0);
    
    /*  State Of Charge 0 */
    auto Bat0 = indices(_battery_inverters,phases,T.start());
    Constraint State_Of_Charge0("State_Of_Charge0");
    State_Of_Charge0 = Sc;
    ODO->add(State_Of_Charge0.in(Bat0) == 0);
    Constraint Pb0("Pb0");
    Pb0 = Pb_;
    ODO->add(Pb0.in(Bat0) == 0);
    
    /*  EFFICIENCIES */
    Constraint DieselEff("DieselEff");
    DieselEff += Pg - gen_eff*Pg_;
//    ODO->add(DieselEff.in(Gt) == 0);
    
    Constraint EfficiencyExist("BatteryEfficiencyExisting");
    EfficiencyExist += Pb  - eff_a*Pb_ - eff_b;//TODO without time extending eff_a and eff_b
    ODO->add(EfficiencyExist.in(indices(_eff_pieces,exist_Bt)) <= 0);
    
    Constraint EfficiencyPot("BatteryEfficiencyPotential");
    EfficiencyPot += Pb  - eff_a*Pb_ - eff_b*w_b;
    ODO->add(EfficiencyPot.in(indices(_eff_pieces,pot_Bt)) <= 0);
    
    
    for (auto n:nodes) {
        auto b = (Bus*)n;
        //        b->print();
        for (auto i = 0; i < b->_pot_gen.size(); i++) {
            auto gen = b->_pot_gen[i];
            if(min_diesel_invest.eval(gen->_name)==max_diesel_invest.eval(gen->_name)){
                Constraint FixedDieselInvest("FixedDieselInvest"+gen->_name);
                FixedDieselInvest += w_g(gen->_name);
                ODO->add(FixedDieselInvest == 1);
                for (auto j = i+1; j < b->_pot_gen.size(); j++) {
                    auto gen2 = b->_pot_gen[j];
                    if (gen2->_gen_type==gen->_gen_type) {
                        Constraint FixedDieselInvest("FixedDieselInvest"+gen2->_name);
                        FixedDieselInvest += w_g(gen2->_name);
                        ODO->add(FixedDieselInvest == 1);
                    }
                }
            }
            else {
                Constraint MinDieselInvest("MinDieselInvest_"+b->_name+"_DG"+to_string(gen->_gen_type));
                MinDieselInvest += w_g(gen->_name);
                for (auto j = i+1; j < b->_pot_gen.size(); j++) {
                    auto gen2 = b->_pot_gen[j];
                    if (gen2->_gen_type==gen->_gen_type) {
                        MinDieselInvest += w_g(gen2->_name);
                    }
                }
                auto rhs = min_diesel_invest.eval(gen->_name);
                if (rhs>0) {
                    ODO->add(MinDieselInvest >= rhs);
                }
            }
        }
        for (auto i = 0; i < b->_pot_bat.size(); i++) {
            auto bat = b->_pot_bat[i];
            if(min_batt_invest.eval(bat->_name)==max_batt_invest.eval(bat->_name)){
                Constraint FixedBattInvest("FixedBattInvest"+bat->_name);
                FixedBattInvest += w_b(bat->_name);
                ODO->add(FixedBattInvest == 1);
                for (auto j = i+1; j < b->_pot_bat.size(); j++) {
                    auto bat2 = b->_pot_bat[j];
                    if (bat2->_bat_type==bat->_bat_type) {
                        Constraint FixedBattInvest("FixedBattInvest"+bat2->_name);
                        FixedBattInvest += w_b(bat2->_name);
                        ODO->add(FixedBattInvest == 1);
                    }
                }
            }
            else {
                Constraint MinBattInvest("MinBattInvest_"+b->_name+"_DG"+to_string(bat->_bat_type));
                MinBattInvest += w_b(bat->_name);
                for (auto j = i+1; j < b->_pot_bat.size(); j++) {
                    auto bat2 = b->_pot_bat[j];
                    if (bat2->_bat_type==bat->_bat_type) {
                        MinBattInvest += w_b(bat2->_name);
                    }
                }
                auto rhs = min_batt_invest.eval(bat->_name);
                if (rhs>0) {
                    ODO->add(MinBattInvest >= rhs);
                }
            }
        }
    }
//        ODO->print_expanded();
    return ODO;
}

unique_ptr<Model> PowerNet::build_ROMDST(PowerModelType pmt, int output, double tol, int max_nb_hours){
    /** Indices Sets */
    hours = time(1,max_nb_hours); /**< Hours */
//        indices months = time("jan","feb","mar","apr","may","jun","jul","aug","sep","oct","nov","dec"); /**< Months */
//    indices months = time("jan","feb","mar","apr","may","jun"); /**< Months */
//    months = time("apr", "aug", "dec"); /**< Months */
        indices months = time("jan");
    //        indices months = time("jan", "feb", "mar","apr","may","jun");
//    typical_days = time("week","peak","weekend");
        typical_days = time("week");
    T = indices(months,typical_days,hours);
    double nT = T.size();
    DebugOn("number of time periods = " << nT << endl);
    time_expand(T);
    Nt = indices(nodes,T);
    Nt_load._time_extended = true;
    Nt_no_load._time_extended = true;
    for (auto &nt:*Nt._indices) {
        if (pl.eval(nt)!=0) {
            Nt_load.add(nt);
        }
        else{
            Nt_no_load.add(nt);
        }
    }
    Et = indices(arcs,T);
    Gt = indices(gens,T);
    exist_Gt = indices(_existing_diesel_gens,T);
    exist_Bt = indices(_existing_battery_inverters,T);
    exist_Et = indices(_exist_arcs,T);
    pot_Gt = indices(_potential_diesel_gens,T);
    pot_Bt = indices(_potential_battery_inverters,T);
    pot_Et = indices(_potential_expansion,T);
    Bt = indices(_battery_inverters,T);
    auto T1 = T.exclude(T.start());/**< Excluding first time step */
    Bt1 = indices(_battery_inverters,T1);
    Gt1 = indices(gens,T1);
    Wt = indices(_all_wind_gens,T);
    PVt = indices(_all_PV_gens,T);
    PV_pot_t = indices(_potential_PV_gens,T);
    pot_gen = indices(_potential_diesel_gens);
    pot_batt = indices(_potential_battery_inverters);
    pot_edges = indices(_potential_expansion);
    
    
    /** MODEL DECLARATION */
    unique_ptr<Model> ROMDST(new Model("ROMDST Model"));
    /** VARIABLES */
    
    
    /* Investment binaries */
    
//    var<Real> Pv_cap("Pv_cap", 0, pv_max); /**< Real variable indicating the extra capacity of PV to be installed on bus b */
    var<Real> Pv_cap("Pv_cap", pos_); /**< Real variable indicating the extra capacity of PV to be installed on bus b */
    auto pot_pv = indices(_potential_PV_gens);
    ROMDST->add(Pv_cap.in(pot_pv));
//        var<Real> w_g("w_g",0,1); /**< Binary variable indicating if generator g is built on bus */
//        var<Real> w_b("w_b",0,1); /**< Binary variable indicating if battery b is built on bus */
//        var<Real> w_e("w_e",0,1); /**< Binary variable indicating if expansion is selected for edge e */
//        var<Real> w_pv("w_pv",0,1); /**< Binary variable indicating if expansion is selected for edge e */
//        w_g._is_relaxed = true;
//        w_b._is_relaxed = true;
//        w_e._is_relaxed = true;
//        w_pv._is_relaxed = true;
    
    var<bool> w_g("w_g"); /**< Binary variable indicating if generator g is built on bus */
    var<bool> w_b("w_b"); /**< Binary variable indicating if battery b is built on bus */
    var<bool> w_e("w_e"); /**< Binary variable indicating if expansion is selected for edge e */
    var<bool> w_pv("w_pv"); /**< Binary variable indicating if PV is installed on bus b */
    
    
    
    ROMDST->add(w_g.in(pot_gen),w_b.in(pot_batt),w_e.in(pot_edges),w_pv.in(pot_pv));
    w_g.initialize_all(1);
    w_b.initialize_all(1);
    w_e.initialize_all(1);
    w_pv.initialize_all(1);
    
    this->w_g = w_g;
    this->w_b = w_b;
    this->w_e = w_e;
    this->w_pv = w_pv;
    this->Pv_cap = Pv_cap;
    
    DebugOff("size w_g = " << w_g.get_dim() << endl);
    DebugOff("size w_b = " << w_b.get_dim() << endl);
    DebugOff("size w_e = " << w_e.get_dim() << endl);
    DebugOff("size w_pv = " << w_pv.get_dim() << endl);
    DebugOff("size Pv_cap = " << Pv_cap.get_dim() << endl);
    
    
    /* Diesel power generation variables */
    var<Real> Pg("Pg", pg_min, pg_max);
    var<Real> Qg ("Qg", qg_min, qg_max);
    var<Real> Pg_ ("Pg_", pg_min, pg_max);/**< Active power generation before losses */
//    var<Real> Pg2("Pg2", 0, power(pg_max,2));/**< Square of Pg */
    var<Real> Pg2("Pg2", pos_);/**< Square of Pg */
    ROMDST->add(Pg.in(Gt),Pg_.in(Gt),Qg.in(Gt),Pg2.in(pot_Gt));
    DebugOff("size Pg = " << Pg.get_dim() << endl);
    DebugOff("size Pg_ = " << Pg_.get_dim() << endl);
    DebugOff("size Qg = " << Qg.get_dim() << endl);
    DebugOff("size Pg2 = " << Pg2.get_dim() << endl);
//    Pg.initialize_all(0.001);
//    Qg.initialize_all(0.0001);
    
    this->Pg_ = Pg_;    
    
    /* Battery power generation variables */
    var<Real> Pb("Pb", pb_min, pb_max);/**< Active power generation outside the battery */
    var<Real> Qb ("Qb", qb_min, qb_max);/**< Reactive power generation outside the battery */
    var<Real> Pb_("Pb_", pb_min, pb_max);/**< Active power generation in the battery */
//    var<Real> Qb_("Qb_", qb_min, qb_max);/**< Reactive power generation in the battery */
    ROMDST->add(Pb.in(Bt), Qb.in(Bt), Pb_.in(Bt));
    DebugOff("size Pb = " << Pb.get_dim() << endl);
    DebugOff("size Qb = " << Qb.get_dim() << endl);
    
    
    /* PV power generation variables */
//    var<Real> Pv("Pv", 0,pv_max);
    var<Real> Pv("Pv", pos_);
    ROMDST->add(Pv.in(PVt));
    DebugOff("size Pv = " << Pv.get_dim() << endl);
    
    /* Battery state of charge variables */
    var<Real> Sc("Sc", pos_);
    ROMDST->add(Sc.in(Bt));
    DebugOff("size Sc = " << Sc.get_dim() << endl);
    
    /* Wind power generation variables */
    var<Real> Pw("Pw", pw_min, pw_max);
    //    pw_max.print(true);
    ROMDST->add(Pw.in(Wt));
    DebugOff("size Pw = " << Pw.get_dim() << endl);
    
    /* Power flow variables */
    var<Real> Pij("Pfrom", S_max);
    var<Real> Qij("Qfrom", S_max);
    var<Real> Pji("Pto", S_max);
    var<Real> Qji("Qto", S_max);
    
    ROMDST->add(Pij.in(Et),Qij.in(Et));
    DebugOff("size Pij = " << Pij.get_dim() << endl);
    if (pmt!=LDISTF) {
        ROMDST->add(Pji.in(Et),Qji.in(Et));
    }
    
    /** Voltage magnitude (squared) variables */
    var<Real> v("v", vmin, vmax);
    ROMDST->add(v.in(Nt));
    DebugOff("size v = " << v.get_dim() << endl);
//    v.initialize_all(0.64);
    
    /** Power loss variables */
    var<Real> loss("loss", pos_);
    if (pmt==DISTF || pmt==CDISTF) {
        ROMDST->add(loss.in(Nt));
    }
    
    /** OBJECTIVE FUNCTION */
    func_ obj = product(c1.in(exist_Gt), Pg_.in(exist_Gt)) + product(c1.in(pot_Gt), Pg_.in(pot_Gt)) + product(c2.in(exist_Gt), power(Pg_.in(exist_Gt),2)) + product(c2.in(pot_Gt), Pg2.in(pot_Gt)) + sum(c0.in(exist_Gt));
//    obj *= 12./months.size();
    obj += nT*product(c0.in(pot_gen),w_g.in(pot_gen));
    obj += product(gen_capcost.in(pot_gen), w_g.in(pot_gen));
    obj += product(inverter_capcost.in(pot_batt), w_b.in(pot_batt));
    obj += product(expansion_capcost.in(pot_edges), w_e.in(pot_edges));
    obj += product(pv_capcost.in(pot_pv), w_pv.in(pot_pv));
    obj += product(pv_varcost.in(pot_pv), Pv_cap.in(pot_pv));
    obj *= 1e-3;
    ROMDST->min(obj);
    
//    obj.print_expanded();
    
    /** CONSTRAINTS **/
    
/** FLOW CONSERVATION **/
    
    /** KCL P and Q */
    Constraint KCL_P("KCL_P");
    Constraint KCL_Q("KCL_Q");
    if (pmt==LDISTF) {
        KCL_P  = sum(Pij.out_arcs()) - sum(Pij.in_arcs()) + pl - sum(Pg.in_gens()) - sum(Pb.in_bats()) - sum(Pw.in_wind()) - sum(Pv.in_pv());
        //        KCL_P  = sum(Pij.out_arcs()) - sum(Pij.in_arcs()) + pl - sum(Pg.in_gens()) - sum(Pb.in_bats()) - sum(Pw.in_wind());
        KCL_Q  = sum(Qij.out_arcs()) - sum(Qij.in_arcs()) + ql - sum(Qg.in_gens()) - sum(Qb.in_bats());
    }
    else{
        KCL_P  = sum(Pij.out_arcs()) + sum(Pji.in_arcs()) + pl - sum(Pg.in_gens())- sum(Pb.in_bats()) - sum(Pw.in_wind()) - sum(Pv.in_pv());
        KCL_Q  = sum(Qij.out_arcs()) + sum(Qji.in_arcs()) + ql - sum(Qg.in_gens()) - sum(Qb.in_bats());
    }
    ROMDST->add(KCL_P.in(nodes, T) == 0);
    ROMDST->add(KCL_Q.in(nodes, T) == 0);
    
/**  THERMAL LIMITS **/
    
    /*  Thermal Limit Constraints */
    Constraint Thermal_Limit_from("Thermal_Limit_from");
    Thermal_Limit_from += power(Pij, 2) + power(Qij, 2);
    Thermal_Limit_from -= power(S_max, 2);
    ROMDST->add(Thermal_Limit_from.in(exist_Et) <= 0);
    
    
    /*  Thermal Limit Constraints for expansion edges */
    Constraint Thermal_Limit_from_exp("Thermal_Limit_From_Exp");
    Thermal_Limit_from_exp += power(Pij, 2) + power(Qij, 2);
    Thermal_Limit_from_exp -= power(w_e,2)*power(S_max, 2);
    ROMDST->add(Thermal_Limit_from_exp.in(pot_Et) <= 0);
//    ROMDST->get_constraint("Thermal_Limit_From_Exp")->print_expanded();
    
/**  GENERATOR INVESTMENT **/
    
    /*  On/Off status */
    Constraint OnOff_maxP("OnOff_maxP");
    OnOff_maxP += Pg_ - pg_max*w_g;
    ROMDST->add(OnOff_maxP.in(pot_Gt) <= 0);
    
    Constraint Perspective_OnOff("Perspective_OnOff");
    Perspective_OnOff += power(Pg_,2) - Pg2*w_g;
    ROMDST->add(Perspective_OnOff.in(pot_Gt) <= 0);
    
    Constraint OnOff_maxQ("OnOff_maxQ");
    OnOff_maxQ += Qg - qg_max*w_g;
    ROMDST->add(OnOff_maxQ.in(pot_Gt) <= 0);

    Constraint OnOff_maxQ_N("OnOff_maxQ_N");
    OnOff_maxQ_N += Qg + qg_max*w_g;
    ROMDST->add(OnOff_maxQ_N.in(pot_Gt) >= 0);

/**  POWER FLOW **/
    
    /** Power Flow Constraints */
    Constraint Flow_P_From("Flow_P_From");
    Flow_P_From = v.from() - 2*(r*Pij + x*Qij) - v.to();
    ROMDST->add(Flow_P_From.in(_exist_arcs, T)==0);
    
    Constraint Flow_P_Expansion_L("Flow_P_Expansion_L");
    Flow_P_Expansion_L = v.from() - 2*(r*Pij + x*Qij) - v.to() - (1-w_e)*(v_diff_max);
    ROMDST->add(Flow_P_Expansion_L.in(_potential_expansion, T)<=0);
    
    Constraint Flow_P_Expansion_U("Flow_P_Expansion_U");
    Flow_P_Expansion_U = v.from() - 2*(r*Pij + x*Qij) - v.to() + (1-w_e)*(v_diff_max);
    ROMDST->add(Flow_P_Expansion_U.in(_potential_expansion, T)>=0);
    
    
/**  PV **/
    
    /*  On/Off on Potential PV */
    Constraint OnOffPV("OnOffPV");
    OnOffPV += Pv_cap - w_pv*pv_max;
    ROMDST->add(OnOffPV.in(indices(_potential_PV_gens)) <= 0);
//    ROMDST->get_constraint("OnOffPV")->print_expanded();
    
    /*  Max Cap on Potential PV */
    Constraint MaxCapPV("MaxCapPV");
    MaxCapPV += Pv - Pv_cap*pv_out;
    ROMDST->add(MaxCapPV.in(PV_pot_t) <= 0);
//    ROMDST->get_constraint("MaxCapPV")->print_expanded();
    
    /*  Existing PV */
    Constraint existPV("existPV");
    existPV += Pv - pv_max*pv_out;
    ROMDST->add(existPV.in(indices(_existing_PV_gens, T)) <= 0);
//    ROMDST->get_constraint("existPV")->print_expanded();
    

/**  BATTERIES **/
    
    /*  Apparent Power Limit on Potential Batteries */
    Constraint Apparent_Limit_Batt_Pot("Apparent_Limit_Batt_Potential");
    Apparent_Limit_Batt_Pot += power(Pb, 2) + power(Qb, 2);
    Apparent_Limit_Batt_Pot -= power(w_b,2)*power(pb_max, 2);
    ROMDST->add(Apparent_Limit_Batt_Pot.in(pot_Bt) <= 0);
    
    /*  Apparent Power Limit on Existing Batteries */
    Constraint Apparent_Limit_Batt("Apparent_Limit_Batt_Existing");
    Apparent_Limit_Batt += power(Pb, 2) + power(Qb, 2);
    Apparent_Limit_Batt -= power(pb_max, 2);
    ROMDST->add(Apparent_Limit_Batt.in(exist_Bt) <= 0);
    
    
    /*  State Of Charge */
    Constraint State_Of_Charge("State_Of_Charge");
    State_Of_Charge = Sc - Sc.prev() + Pb_;
    ROMDST->add(State_Of_Charge.in(Bt1) == 0);
    
    /*  State Of Charge 0 */
    auto Bat0 = indices(_battery_inverters,T.start());
    Constraint State_Of_Charge0("State_Of_Charge0");
    State_Of_Charge0 = Sc;
    ROMDST->add(State_Of_Charge0.in(Bat0) == 0);
    Constraint Pb0("Pb0");
    Pb0 = Pb_;
    ROMDST->add(Pb0.in(Bat0) == 0);
    
/*  EFFICIENCIES */
    Constraint DieselEff("DieselEff");
    DieselEff += Pg - gen_eff*Pg_;
    ROMDST->add(DieselEff.in(Gt) == 0);
    
    Constraint EfficiencyExist("BatteryEfficiencyExisting");
    EfficiencyExist += Pb  - eff_a*Pb_ - eff_b;//TODO without time extending eff_a and eff_b
    ROMDST->add(EfficiencyExist.in(indices(_eff_pieces,exist_Bt)) <= 0);
    
    Constraint EfficiencyPot("BatteryEfficiencyPotential");
    EfficiencyPot += Pb  - eff_a*Pb_ - eff_b*w_b;
    ROMDST->add(EfficiencyPot.in(indices(_eff_pieces,pot_Bt)) <= 0);
    
    
    for (auto n:nodes) {
        auto b = (Bus*)n;
//        b->print();
        for (auto i = 0; i < b->_pot_gen.size(); i++) {
            auto gen = b->_pot_gen[i];
            if(min_diesel_invest.eval(gen->_name)==max_diesel_invest.eval(gen->_name)){
                Constraint FixedDieselInvest("FixedDieselInvest"+gen->_name);
                FixedDieselInvest += w_g(gen->_name);
                ROMDST->add(FixedDieselInvest == 1);
                for (auto j = i+1; j < b->_pot_gen.size(); j++) {
                    auto gen2 = b->_pot_gen[j];
                    if (gen2->_gen_type==gen->_gen_type) {
                        Constraint FixedDieselInvest("FixedDieselInvest"+gen2->_name);
                        FixedDieselInvest += w_g(gen2->_name);
                        ROMDST->add(FixedDieselInvest == 1);
                    }
                }
            }
            else {
                Constraint MinDieselInvest("MinDieselInvest_"+b->_name+"_DG"+to_string(gen->_gen_type));
                MinDieselInvest += w_g(gen->_name);
                for (auto j = i+1; j < b->_pot_gen.size(); j++) {
                    auto gen2 = b->_pot_gen[j];
                    if (gen2->_gen_type==gen->_gen_type) {
                        MinDieselInvest += w_g(gen2->_name);
                    }
                }
                auto rhs = min_diesel_invest.eval(gen->_name);
                if (rhs>0) {
                    ROMDST->add(MinDieselInvest >= rhs);
                }
            }
        }
        for (auto i = 0; i < b->_pot_bat.size(); i++) {
            auto bat = b->_pot_bat[i];
            if(min_batt_invest.eval(bat->_name)==max_batt_invest.eval(bat->_name)){
                Constraint FixedBattInvest("FixedBattInvest"+bat->_name);
                FixedBattInvest += w_b(bat->_name);
                ROMDST->add(FixedBattInvest == 1);
                for (auto j = i+1; j < b->_pot_bat.size(); j++) {
                    auto bat2 = b->_pot_bat[j];
                    if (bat2->_bat_type==bat->_bat_type) {
                        Constraint FixedBattInvest("FixedBattInvest"+bat2->_name);
                        FixedBattInvest += w_b(bat2->_name);
                        ROMDST->add(FixedBattInvest == 1);
                    }
                }
            }
            else {
                Constraint MinBattInvest("MinBattInvest_"+b->_name+"_DG"+to_string(bat->_bat_type));
                MinBattInvest += w_b(bat->_name);
                for (auto j = i+1; j < b->_pot_bat.size(); j++) {
                    auto bat2 = b->_pot_bat[j];
                    if (bat2->_bat_type==bat->_bat_type) {
                        MinBattInvest += w_b(bat2->_name);
                    }
                }
                auto rhs = min_batt_invest.eval(bat->_name);
                if (rhs>0) {
                    ROMDST->add(MinBattInvest >= rhs);
                }
            }
        }
    }
//    ROMDST->print_expanded();
    return ROMDST;
}

double PowerNet::solve_acopf(PowerModelType pmt, int output, double tol){
    
    bool polar = (pmt==ACPOL);
    if (polar) {
        Debug("Using polar model\n");
    }
    else {
        Debug("Using rectangular model\n");
    }
    Model ACOPF("AC-OPF Model");
    /** Variables */
    /* Power generation variables */
    var<Real> Pg("Pg", pg_min, pg_max);
    var<Real> Qg ("Qg", qg_min, qg_max);
    ACOPF.add_var(Pg.in(gens));
    ACOPF.add_var(Qg.in(gens));
    
    /* Power flow variables */
    var<Real> Pf_from("Pf_from", S_max);
    var<Real> Qf_from("Qf_from", S_max);
    var<Real> Pf_to("Pf_to", S_max);
    var<Real> Qf_to("Qf_to", S_max);
    
    ACOPF.add_var(Pf_from.in(arcs));
    ACOPF.add_var(Qf_from.in(arcs));
    ACOPF.add_var(Pf_to.in(arcs));
    ACOPF.add_var(Qf_to.in(arcs));
    
    /** Voltage related variables */
    var<Real> theta("theta");
    var<Real> v("|V|", v_min, v_max);
    //    var<Real> vr("vr");
    //    var<Real> vi("vi");
    var<Real> vr("vr", v_max);
    var<Real> vi("vi", v_max);
    
    if (polar) {
        ACOPF.add_var(v.in(nodes));
        ACOPF.add_var(theta.in(nodes));
        v.initialize_all(1);
    }
    else {
        ACOPF.add_var(vr.in(nodes));
        ACOPF.add_var(vi.in(nodes));
        vr.initialize_all(1.0);
    }
    
    /** Construct the objective function */
    func_ obj = product(c1, Pg) + product(c2, power(Pg,2)) + sum(c0);
    ACOPF.min(obj.in(gens));
    
    /** Define constraints */
    
    /* REF BUS */
    Constraint Ref_Bus("Ref_Bus");
    if (polar) {
        Ref_Bus = theta(get_ref_bus());
    }
    else {
        Ref_Bus = vi(get_ref_bus());
    }
    ACOPF.add_constraint(Ref_Bus == 0);
    
    /** KCL Flow conservation */
    Constraint KCL_P("KCL_P");
    Constraint KCL_Q("KCL_Q");
    KCL_P  = sum(Pf_from.out_arcs()) + sum(Pf_to.in_arcs()) + pl - sum(Pg.in_gens());
    KCL_Q  = sum(Qf_from.out_arcs()) + sum(Qf_to.in_arcs()) + ql - sum(Qg.in_gens());
    /* Shunts */
    if (polar) {
        KCL_P +=  gs*power(v,2);
        KCL_Q -=  bs*power(v,2);
    }
    else {
        KCL_P +=  gs*(power(vr,2)+power(vi,2));
        KCL_Q -=  bs*(power(vr,2)+power(vi,2));
    }
    ACOPF.add_constraint(KCL_P.in(nodes) == 0);
    ACOPF.add_constraint(KCL_Q.in(nodes) == 0);
    
    /** AC Power Flows */
    /** TODO write the constraints in Complex form */
    Constraint Flow_P_From("Flow_P_From");
    Flow_P_From += Pf_from;
    if (polar) {
        Flow_P_From -= g/power(tr,2)*power(v.from(),2);
        Flow_P_From += g/tr*(v.from()*v.to()*cos(theta.from() - theta.to() - as));
        Flow_P_From += b/tr*(v.from()*v.to()*sin(theta.from() - theta.to() - as));
    }
    else {
        Flow_P_From -= g_ff*(power(vr.from(), 2) + power(vi.from(), 2));
        Flow_P_From -= g_ft*(vr.from()*vr.to() + vi.from()*vi.to());
        Flow_P_From -= b_ft*(vi.from()*vr.to() - vr.from()*vi.to());
    }
    ACOPF.add_constraint(Flow_P_From.in(arcs)==0);
    
    Constraint Flow_P_To("Flow_P_To");
    Flow_P_To += Pf_to;
    if (polar) {
        Flow_P_To -= g*power(v.to(), 2);
        Flow_P_To += g/tr*(v.from()*v.to()*cos(theta.to() - theta.from() + as));
        Flow_P_To += b/tr*(v.from()*v.to()*sin(theta.to() - theta.from() + as));
    }
    else {
        Flow_P_To -= g_tt*(power(vr.to(), 2) + power(vi.to(), 2));
        Flow_P_To -= g_tf*(vr.from()*vr.to() + vi.from()*vi.to());
        Flow_P_To -= b_tf*(vi.to()*vr.from() - vr.to()*vi.from());
    }
    ACOPF.add_constraint(Flow_P_To.in(arcs)==0);
    
    Constraint Flow_Q_From("Flow_Q_From");
    Flow_Q_From += Qf_from;
    if (polar) {
        Flow_Q_From += (0.5*ch+b)/power(tr,2)*power(v.from(),2);
        Flow_Q_From -= b/tr*(v.from()*v.to()*cos(theta.from() - theta.to() - as));
        Flow_Q_From += g/tr*(v.from()*v.to()*sin(theta.from() - theta.to() - as));
    }
    else {
        Flow_Q_From += b_ff*(power(vr.from(), 2) + power(vi.from(), 2));
        Flow_Q_From += b_ft*(vr.from()*vr.to() + vi.from()*vi.to());
        Flow_Q_From -= g_ft*(vi.from()*vr.to() - vr.from()*vi.to());
    }
    ACOPF.add_constraint(Flow_Q_From.in(arcs)==0);
    Constraint Flow_Q_To("Flow_Q_To");
    Flow_Q_To += Qf_to;
    if (polar) {
        Flow_Q_To += (0.5*ch+b)*power(v.to(),2);
        Flow_Q_To -= b/tr*(v.from()*v.to()*cos(theta.to() - theta.from() + as));
        Flow_Q_To += g/tr*(v.from()*v.to()*sin(theta.to() - theta.from() + as));
    }
    else {
        Flow_Q_To += b_tt*(power(vr.to(), 2) + power(vi.to(), 2));
        Flow_Q_To += b_tf*(vr.from()*vr.to() + vi.from()*vi.to());
        Flow_Q_To -= g_tf*(vi.to()*vr.from() - vr.to()*vi.from());
    }
    ACOPF.add_constraint(Flow_Q_To.in(arcs)==0);
    
    /** AC voltage limit constraints. */
    if (!polar) {
        Constraint Vol_limit_UB("Vol_limit_UB");
        Vol_limit_UB = power(vr, 2) + power(vi, 2);
        Vol_limit_UB -= power(v_max, 2);
        ACOPF.add_constraint(Vol_limit_UB.in(nodes) <= 0);
        
        Constraint Vol_limit_LB("Vol_limit_LB");
        Vol_limit_LB = power(vr, 2) + power(vi, 2);
        Vol_limit_LB -= power(v_min,2);
        ACOPF.add_constraint(Vol_limit_LB.in(nodes) >= 0);
        DebugOff(v_min.to_str(true) << endl);
        DebugOff(v_max.to_str(true) << endl);
    }
    
    
    /* Phase Angle Bounds constraints */
    Constraint PAD_UB("PAD_UB");
    Constraint PAD_LB("PAD_LB");
    auto bus_pairs = get_bus_pairs();
    if (polar) {
        PAD_UB = theta.from() - theta.to();
        PAD_UB -= th_max;
        PAD_LB = theta.from() - theta.to();
        PAD_LB -= th_min;
        DebugOff(th_min.to_str(true) << endl);
        DebugOff(th_max.to_str(true) << endl);
    }
    else {
        DebugOff("Number of bus_pairs = " << bus_pairs.size() << endl);
        PAD_UB = vi.from()*vr.to() - vr.from()*vi.to();
        PAD_UB -= tan_th_max*(vr.from()*vr.to() + vi.from()*vi.to());
        
        PAD_LB = vi.from()*vr.to() - vr.from()*vi.to();
        PAD_LB -= tan_th_min*(vr.from()*vr.to() + vi.from()*vi.to());
        DebugOff(th_min.to_str(true) << endl);
        DebugOff(th_max.to_str(true) << endl);
    }
    ACOPF.add_constraint(PAD_UB.in(bus_pairs) <= 0);
    ACOPF.add_constraint(PAD_LB.in(bus_pairs) >= 0);
    
    
    /*  Thermal Limit Constraints */
    Constraint Thermal_Limit_from("Thermal_Limit_from");
    Thermal_Limit_from += power(Pf_from, 2) + power(Qf_from, 2);
    Thermal_Limit_from -= power(S_max, 2);
    ACOPF.add_constraint(Thermal_Limit_from.in(arcs) <= 0);
    
    Constraint Thermal_Limit_to("Thermal_Limit_to");
    Thermal_Limit_to += power(Pf_to, 2) + power(Qf_to, 2);
    Thermal_Limit_to -= power(S_max,2);
    ACOPF.add_constraint(Thermal_Limit_to.in(arcs) <= 0);
    DebugOff(S_max.in(arcs).to_str(true) << endl);
    bool relax;
    solver OPF(ACOPF,ipopt);
    auto mipgap = 1e-6;
    OPF.run(output, relax = false, tol = 1e-6, mipgap, "ma27");
    return ACOPF._obj_val;
}



