//
//  PowerNet.cpp
//
//
//  Created by Hassan Hijazi on 03/06/2017.
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
#include <armadillo>

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


indices PowerNet::out_arcs_per_node_time() const{
    auto ids = Et;
    ids._name = "out_arcs_per_node_time";
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Nt.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Nt._keys) {
        auto pos = key.find_last_of(",");
        auto name = key.substr(pos+1);
        key = key.substr(0,pos);
        pos = key.find_last_of(",");
        auto ph = key.substr(pos+1);
        time_stamp = key.substr(0,pos);
        auto n = get_node(name);
        if (!n->_active) {
            continue;
        }
        for (auto a:n->get_out()) {
            if (!a->_active || !a->has_phase(ph)) {
                continue;
            }
            key = time_stamp+","+ph+","+a->_name;
            auto it1 = ids._keys_map->find(key);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function out_arcs_per_node(), unknown key.");
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::in_arcs_per_node_time() const{
    auto ids = Et;
    ids._name = "in_arcs_per_node_time";
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Nt.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Nt._keys) {
        auto pos = key.find_last_of(",");
        auto name = key.substr(pos+1);
        key = key.substr(0,pos);
        pos = key.find_last_of(",");
        auto ph = key.substr(pos+1);
        time_stamp = key.substr(0,pos);
        auto n = get_node(name);
        if (!n->_active) {
            continue;
        }
        for (auto a:n->get_in()) {
            if (!a->_active || !a->has_phase(ph)) {
                continue;
            }
            key = time_stamp+","+ph+","+a->_name;
            auto it1 = ids._keys_map->find(key);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function in_arcs_per_node_time(), unknown key.");
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::to_branch_phase(unsigned ph) const{
    indices Ei;
    string phi;
    switch (ph) {
        case 1:
            Ei = Et1;
            phi = "ph1";
            break;
        case 2:
            Ei = Et2;
            phi = "ph2";
            break;
        case 3:
            Ei = Et3;
            phi = "ph3";
            break;
        default:
            break;
    }
    indices ids(Nt);
    ids._name = "to_branch_phase"+to_string(ph);
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Ei.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Ei._keys) {
        auto key_dest = key.substr(key.find_last_of(",")+1,key.size());
        auto key_arc = key.substr(0, key.find_last_of(","));
        key_arc = key_arc.substr(0, key_arc.find_last_of(","));
        key_arc = key.substr(key_arc.find_last_of(",")+1,key.size());
        time_stamp = key.substr(0, key.find_last_of(","));//dest
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//src
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//arcid
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//phase
        auto dest = get_node(key_dest);
        auto arc = arcMap.at(key_arc);
        if (!arc->_active) {
            throw invalid_argument("inactive arc in E_phi");
        }
        for (auto ph2:arc->_phases) {
            if(ph2==ph){
                continue;
            }
            if(dest->_phases.count(ph2)==0){
                throw invalid_argument("In function to_branch_phase(), destination bus " + key_dest + " is missing phase " + to_string(ph2)+", but arc " + key_arc + " has it");
            }
            key = time_stamp+",ph"+to_string(ph2)+","+key_dest;
            auto it1 = ids._keys_map->find(key);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function to_branch_phase(), unknown key.");
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::fixed_to_branch_phase(unsigned ph) const{
    indices Ei;
    string phi;
    switch (ph) {
        case 1:
            Ei = Et1;
            phi = "ph1";
            break;
        case 2:
            Ei = Et2;
            phi = "ph2";
            break;
        case 3:
            Ei = Et3;
            phi = "ph3";
            break;
        default:
            break;
    }
    indices ids(Nt);
    ids._name = "ref_to_phase"+to_string(ph);
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Ei.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Ei._keys) {
        auto key_dest = key.substr(key.find_last_of(",")+1,key.size());
        auto key_arc = key.substr(0, key.find_last_of(","));
        key_arc = key_arc.substr(0, key_arc.find_last_of(","));
        key_arc = key.substr(key_arc.find_last_of(",")+1,key.size());
        time_stamp = key.substr(0, key.find_last_of(","));//dest
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//src
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//arcid
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//phase
        auto arc = arcMap.at(key_arc);
        if (!arc->_active) {
            throw invalid_argument("inactive arc in E_phi");
        }
        for (auto ph2:arc->_phases) {
            if(ph2==ph){
                continue;
            }
            key = time_stamp+","+phi+","+key_dest;
            auto it1 = ids._keys_map->find(key);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function fixed_to_branch_phase(), unknown key.");
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::fixed_from_branch_phase(unsigned ph) const{
    indices Ei;
    string phi;
    switch (ph) {
        case 1:
            Ei = Et1;
            phi = "ph1";
            break;
        case 2:
            Ei = Et2;
            phi = "ph2";
            break;
        case 3:
            Ei = Et3;
            phi = "ph3";
            break;
        default:
            break;
    }
    indices ids(Nt);
    ids._name = "ref_from_branch_phase"+to_string(ph);
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Ei.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Ei._keys) {
        auto key_src = key.substr(0, key.find_last_of(","));
        key_src = key_src.substr(key_src.find_last_of(",")+1,key_src.size());
        auto key_arc = key.substr(0, key.find_last_of(","));
        key_arc = key_arc.substr(0, key_arc.find_last_of(","));
        key_arc = key.substr(key_arc.find_last_of(",")+1,key.size());
        time_stamp = key.substr(0, key.find_last_of(","));//dest
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//src
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//arcid
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//phase
        auto arc = arcMap.at(key_arc);
        if (!arc->_active) {
            throw invalid_argument("inactive arc in E_phi");
        }
        for (auto ph2:arc->_phases) {
            if(ph2==ph){
                continue;
            }
            key = time_stamp+","+phi+","+key_src;
            auto it1 = ids._keys_map->find(key);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function fixed_from_branch_phase(), unknown key.");
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::from_branch_phase(unsigned ph) const{
    indices Ei;
    string phi;
    switch (ph) {
        case 1:
            Ei = Et1;
            phi = "ph1";
            break;
        case 2:
            Ei = Et2;
            phi = "ph2";
            break;
        case 3:
            Ei = Et3;
            phi = "ph3";
            break;
        default:
            break;
    }
    indices ids(Nt);
    ids._name = "from_branch_phase"+to_string(ph);
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Ei.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Ei._keys) {
        auto key_src = key.substr(0, key.find_last_of(","));
        key_src = key_src.substr(key_src.find_last_of(",")+1,key_src.size());
        auto key_arc = key.substr(0, key.find_last_of(","));
        key_arc = key_arc.substr(0, key_arc.find_last_of(","));
        key_arc = key.substr(key_arc.find_last_of(",")+1,key.size());
        time_stamp = key.substr(0, key.find_last_of(","));//dest
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//src
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//arcid
        time_stamp = time_stamp.substr(0, time_stamp.find_last_of(","));//phase
        auto src = get_node(key_src);
        auto arc = arcMap.at(key_arc);
        if (!arc->_active) {
            throw invalid_argument("inactive arc in E_phi");
        }
        for (auto ph2:arc->_phases) {
            if(ph2==ph){
                continue;
            }
            if(src->_phases.count(ph2)==0){
                throw invalid_argument("In function from_branch_phase(), source bus " + key_src + " is missing phase " + to_string(ph2)+", but arc " + key_arc + " has it");
            }
            key = time_stamp+",ph"+to_string(ph2)+","+key_src;
            auto it1 = ids._keys_map->find(key);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function from_branch_phase(), unknown key.");
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::get_branch_id_phase(unsigned ph) const{
    indices ids(cross_phase);
    ids._name = "branch_phase_id"+to_string(ph);
    ids._ids = make_shared<vector<vector<size_t>>>();
    indices Ei;
    string phi;
    switch (ph) {
        case 1:
            Ei = E_ph1;
            phi = "ph1";
            break;
        case 2:
            Ei = E_ph2;
            phi = "ph2";
            break;
        case 3:
            Ei = E_ph3;
            phi = "ph3";
            break;
        default:
            break;
    }
    ids._ids->resize(1);
    string key, time_stamp;
    for (auto key: *Ei._keys) {
        auto key_arc = key.substr(0, key.find_last_of(","));
        key_arc = key_arc.substr(0, key_arc.find_last_of(","));
        key_arc = key.substr(key_arc.find_last_of(",")+1,key.size());
        auto arc = arcMap.at(key_arc);
        if (!arc->_active) {
            throw invalid_argument("inactive arc in E_phi");
        }
        key = phi+","+phi+","+arc->_name;
        auto it1 = ids._keys_map->find(key);
        if (it1 == ids._keys_map->end()){
            throw invalid_argument("In function get_branch_phase(), unknown key: " + key);
        }
        ids._ids->at(0).push_back(it1->second);
    }
    return ids;
}

indices PowerNet::get_branch_phase(unsigned ph) const{
    indices ids(cross_phase);
    ids._name = "branch_phase"+to_string(ph);
    ids._ids = make_shared<vector<vector<size_t>>>();
    indices Ei;
    string phi;
    switch (ph) {
        case 1:
            Ei = E_ph1;
            phi = "ph1";
            break;
        case 2:
            Ei = E_ph2;
            phi = "ph2";
            break;
        case 3:
            Ei = E_ph3;
            phi = "ph3";
            break;
        default:
            break;
    }
    ids._ids->resize(Ei.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Ei._keys) {
        auto key_arc = key.substr(0, key.find_last_of(","));
        key_arc = key_arc.substr(0, key_arc.find_last_of(","));
        key_arc = key.substr(key_arc.find_last_of(",")+1,key.size());
        auto arc = arcMap.at(key_arc);
        if (!arc->_active) {
            throw invalid_argument("inactive arc in E_phi");
        }
        for (auto ph2:arc->_phases) {
            if(ph2==ph){
                continue;
            }
            key = phi+",ph"+to_string(ph2)+","+arc->_name;
            auto it1 = ids._keys_map->find(key);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function get_branch_phase(), unknown key.");
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::Batt_per_node_time() const{
    auto ids = Bt;
    ids._name = "batt_per_node_time";
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Nt.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Nt._keys) {
        auto pos = key.find_last_of(",");
        auto name = key.substr(pos+1);
        key = key.substr(0,pos);
        pos = key.find_last_of(",");
        auto ph = key.substr(pos+1);
        key = key.substr(0,pos);
        auto n = get_node(name);
        if (!n->_active) {
            continue;
        }
        for (auto g:((Bus*)n)->_bat) {
            if (!g->_active || !g->has_phase(ph)) {
                continue;
            }
            auto gname = key+","+ph+","+g->_name;
            auto it1 = ids._keys_map->find(gname);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function batt_per_node_time(), unknown key: " + gname);
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::PV_per_node_time() const{
    auto ids = PVt;
    ids._name = "PV_per_node_time";
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Nt.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Nt._keys) {
        auto pos = key.find_last_of(",");
        auto name = key.substr(pos+1);
        key = key.substr(0,pos);
        pos = key.find_last_of(",");
        auto ph = key.substr(pos+1);
        key = key.substr(0,pos);
        auto n = get_node(name);
        if (!n->_active) {
            continue;
        }
        for (auto g:((Bus*)n)->_pv) {
            if (!g->_active || !g->has_phase(ph)) {
                continue;
            }
            auto gname = key+","+ph+","+g->_name;
            auto it1 = ids._keys_map->find(gname);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function pv_per_node_time(), unknown key: " + gname);
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::Wind_per_node_time() const{
    auto ids = Wt;
    ids._name = "Wind_per_node_time";
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Nt.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Nt._keys) {
        auto pos = key.find_last_of(",");
        auto name = key.substr(pos+1);
        key = key.substr(0,pos);
        pos = key.find_last_of(",");
        auto ph = key.substr(pos+1);
        key = key.substr(0,pos);
        auto n = get_node(name);
        if (!n->_active) {
            continue;
        }
        for (auto g:((Bus*)n)->_wind) {
            if (!g->_active || !g->has_phase(ph)) {
                continue;
            }
            auto gname = key+","+ph+","+g->_name;
            auto it1 = ids._keys_map->find(gname);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function wind_per_node_time(), unknown key: " + gname);
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}


indices PowerNet::gens_per_node_time() const{
    auto ids = Gt;
    ids._name = "gens_per_node_time";
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(Nt.size());
    string key, time_stamp;
    size_t inst = 0;
    for (auto key: *Nt._keys) {
        auto pos = key.find_last_of(",");
        auto name = key.substr(pos+1);
        key = key.substr(0,pos);
        pos = key.find_last_of(",");
        auto ph = key.substr(pos+1);
        key = key.substr(0,pos);
        auto n = get_node(name);
        if (!n->_active) {
            continue;
        }
        for (auto g:((Bus*)n)->_gen) {
            if (!g->_active || !g->has_phase(ph)) {
                continue;
            }
            auto gname = key+","+ph+","+g->_name;
            auto it1 = ids._keys_map->find(gname);
            if (it1 == ids._keys_map->end()){
                throw invalid_argument("In function gens_per_node_time(), unknown key: " + gname);
            }
            ids._ids->at(inst).push_back(it1->second);
        }
        inst++;
    }
    return ids;
}

indices PowerNet::gens_per_node() const{
    indices ids("gens_per_node");
    ids = indices(gens);
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(get_nb_active_nodes());
    string key;
    size_t inst = 0;
    for (auto n: nodes) {
        if (n->_active) {
            for(auto g: ((Bus*)n)->_gen){
                if (!g->_active) {
                    continue;
                }
                key = g->_name;
                auto it1 = ids._keys_map->find(key);
                if (it1 == ids._keys_map->end()){
                    throw invalid_argument("In function gen_ids(), unknown key.");
                }
                ids._ids->at(inst).push_back(it1->second);
            }
            inst++;
        }
    }
    return ids;
}

indices PowerNet::out_arcs_per_node() const{
    auto ids = indices(arcs);
    ids._name = "out_arcs_per_node";
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(get_nb_active_nodes());
    string key;
    size_t inst = 0;
    for (auto n: nodes) {
        if (n->_active) {
            for (auto a:n->get_out()) {
                if (!a->_active) {
                    continue;
                }
                key = a->_name;
                auto it1 = ids._keys_map->find(key);
                if (it1 == ids._keys_map->end()){
                    throw invalid_argument("In function out_arcs_per_node(), unknown key.");
                }
                ids._ids->at(inst).push_back(it1->second);
            }
            inst++;
        }
    }
    return ids;
}

indices PowerNet::in_arcs_per_node() const{
    auto ids = indices(arcs);
    ids._name = "in_arcs_per_node";
    ids._ids = make_shared<vector<vector<size_t>>>();
    ids._ids->resize(get_nb_active_nodes());
    string key;
    size_t inst = 0;
    for (auto n: nodes) {
        if (n->_active) {
            for (auto a:n->get_in()) {
                if (!a->_active) {
                    continue;
                }
                key = a->_name;
                auto it1 = ids._keys_map->find(key);
                if (it1 == ids._keys_map->end()){
                    throw invalid_argument("In function in_arcs_per_node(), unknown key.");
                }
                ids._ids->at(inst).push_back(it1->second);
            }
            inst++;
        }
    }
    return ids;
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
        if (bp->_active) {
            nb++;
        }
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
    vector<string> r; /* Matrix of resistance */
    vector<string> x; /* Matrix of reactance */
    vector<double> cost; /* Vector of expansion costs */
    vector<double> Smax; /* Vector of thermal limits */
    vector<string> phase_list;
    while (row_it!=ws.rows().end()) {
        auto row = *row_it++;
        cost.push_back(row[2].value<double>());
        r.push_back(row[3].to_string());
        x.push_back(row[4].to_string());
        Smax.push_back(row[5].value<double>()/bMVA);
        phase_list.push_back(row[6].to_string());
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
                        auto arc = new Line("Type"+to_string(b_id) + "," + src + "," + dest); // Name of lines
                        arc->_expansion = true;
                        arc->_active = true;
                        arc->b_type = b_id;
                        arc->smax = Smax[b_id-1];
                        arc->cost = cost[b_id-1];
                        arc->set_phases(phase_list[b_id-1]);
                        arc->_id = index++;
                        arc->_src = get_node(src);
                        arc->_dest= get_node(dest);
                        arc->connect();
                        add_arc(arc);
                        expansion_capcost.add_val(arc->_name, arc->cost);
                        _potential_expansion.push_back(arc);
                        auto r_str = r[b_id-1];
                        auto x_str = x[b_id-1];
                        auto ymat = arma::cx_mat(3,3);
                        for (auto i = 0; i<3; i++) {
                            if(arc->_phases.count(i+1)!=0 && arc->_src->_phases.count(i+1)!=0 && arc->_dest->_phases.count(i+1)!=0){
                                auto ph_key = "ph"+to_string(i+1)+","+arc->_name;
                                E_ph.insert(ph_key);
                                pot_E_ph.insert(ph_key);
                                this->S_max.add_val(ph_key, arc->smax);
                                if(i==0){
                                    E_ph1.insert(ph_key);
                                }
                                else if(i==1){
                                    E_ph2.insert(ph_key);
                                }
                                else {
                                    E_ph3.insert(ph_key);
                                }
                                for (auto j= 0; j<3; j++) {
                                    if(arc->_phases.count(i+1)!=0 && arc->_src->_phases.count(i+1)!=0 && arc->_dest->_phases.count(i+1)!=0){
                                        auto key = "ph"+to_string(i+1)+",ph"+to_string(j+1)+","+arc->_name;
                                        r_str = r_str.substr(0,r_str.find_first_of(","));
                                        x_str = x_str.substr(0,x_str.find_first_of(","));
                                        br_r_.add_val(key, stod(r_str));
                                        br_x_.add_val(key, stod(x_str));
                                        ymat(i,j) = complex<double>(br_r_.eval(),br_x_.eval());
//                                            b.add_val(key, ymat_inv(i,j).imag());
                                    }
                                }
                                b_fr_.add_val(ph_key, 0);
                                b_to_.add_val(ph_key, 0);
                                g_fr_.add_val(ph_key, 0);
                                g_to_.add_val(ph_key, 0);
                                shift_.add_val(ph_key, 0);
                                tap_.add_val(ph_key, 1);
                            }
                        }
                        arma::cx_mat ymat_inv = arma::pinv(ymat);
                        for (auto i= 0; i<3; i++) {
                            if(arc->_phases.count(i+1)==0)
                                continue;
                            for (auto j= 0; j<3; j++) {
                                if(arc->_phases.count(j+1)==0)
                                    continue;
                                auto key = "ph"+to_string(i+1)+",ph"+to_string(j+1)+","+arc->_name;
                                cross_phase.insert(key);
                                g.add_val(key, ymat_inv(i,j).real());
                                b.add_val(key, ymat_inv(i,j).imag());
                            }
                        }
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
        row_it++;//SKIP SECOND ROW
        vector<string> pv_phase_list;
        auto row = *row_it;
        for (auto i = 0; i<nodes.size(); i++) {
            auto bname = row[0].to_string();
            bname = bname.substr(4);//remove the node string
            auto bus = (Bus*)get_node(bname);
            min_PV = row[1].value<double>();
            max_PV = row[2].value<double>();
            exist = row[3].value<double>();
            bus->_min_PV_cap = min_PV;
            bus->_max_PV_cap = max_PV;
            bus->_existing_PV_cap = exist;
            if (max_PV>0 || exist >0) {
                pv_phase_list.push_back(row[5].to_string());
                if(exist>0){
                    auto name = "exist,"+bus->_name;
                    bus->_pv.push_back(new PV(name,min_PV,max_PV));
                    bus->_pv.back()->set_phases(pv_phase_list.back());
                    DebugOn("existing PV cap at bus" << bus->_name << " = " << exist << endl);
                    DebugOn("On phases: " << pv_phase_list.back() << endl);
                    for (int ph = 1; ph<=3; ph++) {
                        if(bus->_phases.count(ph)==0 || bus->_pv.back()->_phases.count(ph)==0){
                            continue;
                        }
                        exist_PV_ph.insert("ph"+to_string(ph)+","+name);
                        PV_ph.insert("ph"+to_string(ph)+","+name);
                    }
                }
                if(max_PV-exist>0){
                    auto name = "potential,"+bus->_name;
                    bus->_pv.push_back(new PV(name,min_PV,max_PV));
                    bus->_pv.back()->set_phases(pv_phase_list.back());
                    DebugOn("min PV cap at bus" << bus->_name << " = " << min_PV << endl);
                    DebugOn("max PV cap at bus" << bus->_name << " = " << max_PV << endl);
                    DebugOn("Potential PV cap at bus" << bus->_name << " = " << max_PV-exist << endl);
                    DebugOn("On phases: " << pv_phase_list.back() << endl);
                    for (int ph = 1; ph<=3; ph++) {
                        if(bus->_phases.count(ph)==0 || bus->_pv.back()->_phases.count(ph)==0){
                            continue;
                        }
                        pot_PV_ph.insert("ph"+to_string(ph)+","+name);
                        PV_ph.insert("ph"+to_string(ph)+","+name);
                    }
                }
            }
            row_it++;
            row = *row_it;
        }
        assert(row[0].value<string>()=="Wind");
        row_it++;
        row_it++;//SKIP SECOND ROW
        row = *row_it;
        for (auto i = 0; i<nodes.size(); i++) {
            auto bname = row[0].to_string();
            bname = bname.substr(4);//remove the node string
            auto bus = (Bus*)get_node(bname);
            min_Wind = row[1].value<double>();
            max_Wind = row[2].value<double>();
            exist = row[3].value<double>();
            bus->_min_Wind_cap = min_Wind;
            bus->_max_Wind_cap = max_Wind;
            bus->_existing_Wind_cap = exist;
            if(exist>0){
                auto name = "exist,"+bus->_name;
                bus->_wind.push_back(new WindGen(name,min_Wind,max_Wind));
                DebugOn("min Wind cap at bus" << bus->_name << " = " << min_Wind << endl);
                DebugOn("max Wind cap at bus" << bus->_name << " = " << max_Wind << endl);
                DebugOn("existing Wind cap at bus" << bus->_name << " = " << exist << endl);
                for (int ph = 1; ph<=3; ph++) {
                    if(bus->_phases.count(ph)==0){
                        continue;
                    }
                    exist_Wind_ph.insert("ph"+to_string(ph)+","+name);
                    Wind_ph.insert("ph"+to_string(ph)+","+name);
                }
            }
            if(max_Wind-exist>0){
                auto name = "potential,"+bus->_name;
                bus->_wind.push_back(new WindGen(name,min_Wind,max_Wind));
                DebugOn("min Wind cap at bus" << bus->_name << " = " << min_Wind << endl);
                DebugOn("max Wind cap at bus" << bus->_name << " = " << max_Wind << endl);
                DebugOn("potential Wind cap at bus" << bus->_name << " = " << max_Wind-exist << endl);
                for (int ph = 1; ph<=3; ph++) {
                    if(bus->_phases.count(ph)==0){
                        continue;
                    }
                    pot_Wind_ph.insert("ph"+to_string(ph)+","+name);
                    Wind_ph.insert("ph"+to_string(ph)+","+name);
                }
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
    vector<string> gen_phase_list;
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
            if (first_row[i].to_string().compare("phases (1,2,3)")==0) {
                gen_phase_list.push_back(row[i].to_string());
            }
        }
        _all_diesel_gens.push_back(DieselGen("DG"+to_string(idx++), max_p, max_s, lifetime, capcost, c0, c1, c2, type, eff, max_ramp_down, max_ramp_up,min_down_time,min_up_time));
        _all_diesel_gens.back().set_phases(gen_phase_list.back());
        _all_diesel_gens.back().print();//check phases
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
        for (auto i = 0; i<nodes.size(); i++) {
            
            auto row = *row_it++;
            auto bname = row[0].to_string();
            bname = bname.substr(4);//remove the node string
            auto bus = (Bus*)get_node(bname);
            unsigned index = 0;
            while (row[0].to_string().compare((*row_it)[0].to_string())==0) {
                max_d = row[6].value<int>();
                exist = row[7].value<int>();
                if (max_d>0 || exist>0) {
                    min_d = row[5].value<int>();
                    assert(max_d>=min_d);
                    age = row[8].value<int>();
                    bus->_diesel_data[index] = DieselData(min_d,max_d,exist,age);
                    auto copy = _all_diesel_gens[index];
                    for (auto i = 0; i<exist; i++) {
                            auto name = copy._name + "," + bus->_name + "," + "slot"+to_string(i);
                            auto gen = new Gen(bus, name, 0, copy._max_p, -copy._max_s, copy._max_s);
                            gen->set_costs(copy._c0, copy._c1, copy._c2);
                            gen->_phases = copy._phases;
                            _existing_diesel_gens.push_back(gen);
                            gens.push_back(gen);
                            bus->_gen.push_back(gen);
                            this->c0.add_val(name,copy._c0);
                            this->c1.add_val(name,copy._c1);
                            this->c2.add_val(name,copy._c2);
                            this->pg_min.add_val(name,0);
                            this->pg_max.add_val(name,copy._max_p);
                            this->qg_min.add_val(name,-copy._max_s);
                            this->qg_max.add_val(name,copy._max_s);
                            this->min_dt.add_val(name,copy._min_down_time);
                            this->min_ut.add_val(name,copy._min_up_time);
                            this->ramp_up.add_val(name,copy._max_ramp_up);
                            this->ramp_down.add_val(name,copy._max_ramp_down);
                            this->gen_eff.add_val(name,copy._eff);
                        for (int ph = 1; ph<=3; ph++) {
                            if(bus->_phases.count(ph)==0 || _all_diesel_gens[index]._phases.count(ph)==0){
                                continue;
                            }
                            G_ph.insert("ph"+to_string(ph)+","+name);
                            exist_G_ph.insert("ph"+to_string(ph)+","+name);
                        }
                    }
                    for (auto i = exist; i<exist+max_d; i++) {
                        auto copy = _all_diesel_gens[index];
                        auto name = copy._name + "," + bus->_name + "," + "slot"+to_string(i);
                        auto gen = new Gen(bus, name, 0, copy._max_p, -copy._max_s, copy._max_s);
                        gen->set_costs(copy._c0, copy._c1, copy._c2);
                        _potential_diesel_gens.push_back(gen);
                        gen->_gen_type = index+1;
                        gen->_phases = copy._phases;
                        gens.push_back(gen);
                        bus->_gen.push_back(gen);
                        bus->_pot_gen.push_back(gen);
                        this->min_diesel_invest.add_val(name, min_d);
                        this->max_diesel_invest.add_val(name, max_d);
                        this->c0.add_val(name,copy._c0);
                        this->c1.add_val(name,copy._c1);
                        this->c2.add_val(name,copy._c2);
                        this->pg_min.add_val(name,0);
                        this->pg_max.add_val(name,copy._max_p);
                        this->qg_min.add_val(name,-copy._max_s);
                        this->qg_max.add_val(name,copy._max_s);
                        this->gen_capcost.add_val(name,copy._capcost);
                        this->min_dt.add_val(name,copy._min_down_time);
                        this->min_ut.add_val(name,copy._min_up_time);
                        this->ramp_up.add_val(name,copy._max_ramp_up);
                        this->ramp_down.add_val(name,copy._max_ramp_down);
                        this->gen_eff.add_val(name,copy._eff);
                        for (int ph = 1; ph<=3; ph++) {
                            if(bus->_phases.count(ph)==0 || _all_diesel_gens[index]._phases.count(ph)==0){
                                continue;
                            }
                            G_ph.insert("ph"+to_string(ph)+","+name);
                            pot_G_ph.insert("ph"+to_string(ph)+","+name);
                        }
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
    vector<string> batt_phase_list;
    idx = 1;
    double max_s = 0, capcost = 0;
    while (row_it!=ws.rows().end()) {
        auto row = *row_it++;
        max_s = row[1].value<double>()/bMVA;
        lifetime = row[2].value<int>();
        capcost = row[3].value<double>();
        auto nb_points = (row.length() - 5)/2;
        x_eff.resize(nb_points);
        y_eff.resize(nb_points);
        for (int i = 0; i<nb_points; i++) {
            x_eff[i] = row[2*i+4].value<double>();
            y_eff[i] = row[2*i+5].value<double>();
        }
        _all_battery_inverters.push_back(BatteryInverter("BI"+to_string(idx++),max_s, lifetime, capcost, x_eff, y_eff));
        Debug("Battery " << _all_battery_inverters.size() << ": ");
        batt_phase_list.push_back(row[row.length()-1].to_string());
        _all_battery_inverters.back().set_phases(batt_phase_list.back());
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
        
         for (auto i = 0; i<nodes.size(); i++) {
            unsigned index = 0;
            auto row = *row_it++;
             auto bname = row[0].to_string();
             bname = bname.substr(4);//remove the node string
             auto bus = (Bus*)get_node(bname);

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
                        this->pb_min.add_val(copy->_name, -copy->_max_s/bMVA);
                        this->pb_max.add_val(copy->_name, copy->_max_s/bMVA);
                        this->qb_min.add_val(copy->_name, -copy->_max_s/bMVA);
                        this->qb_max.add_val(copy->_name, copy->_max_s/bMVA);
                        auto nb_eff_pieces = copy->_x_eff.size() - 1;
                        if(_nb_eff_pieces<nb_eff_pieces){
                            _nb_eff_pieces = nb_eff_pieces;
                        }
                        for (int ph = 1; ph<=3; ph++) {
                            if(bus->_phases.count(ph)==0 || _all_battery_inverters[index]._phases.count(ph)==0){
                                continue;
                            }
                            B_ph.insert("ph"+to_string(ph)+","+copy->_name);
                            exist_B_ph.insert("ph"+to_string(ph)+","+copy->_name);
                            for(auto p =1; p<= _nb_eff_pieces;p++){
                                auto str = "ph"+to_string(ph)+","+copy->_name+",eff"+to_string(p);
                                if (p > nb_eff_pieces) {
                                    eff_a.add_val(str, (copy->_y_eff[nb_eff_pieces] - copy->_y_eff[nb_eff_pieces-1])/(copy->_x_eff[nb_eff_pieces] - copy->_x_eff[nb_eff_pieces-1]));
                                    eff_b.add_val(str, copy->_y_eff[nb_eff_pieces] - eff_a.eval()*copy->_x_eff[nb_eff_pieces]);
                                }
                                else{
                                    eff_a.add_val(str, (copy->_y_eff[p] - copy->_y_eff[p-1])/(copy->_x_eff[p] - copy->_x_eff[p-1]));
                                    eff_b.add_val(str, copy->_y_eff[p] - eff_a.eval()*copy->_x_eff[p]);
                                }
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
                        
                        this->min_batt_invest.add_val(copy->_name, min_bat);
                        this->max_batt_invest.add_val(copy->_name, max_bat);
                        this->inverter_capcost.add_val(copy->_name,copy->_capcost);
                        this->pb_min.add_val(copy->_name, -copy->_max_s/bMVA);
                        this->pb_max.add_val(copy->_name, copy->_max_s/bMVA);
                        this->qb_min.add_val(copy->_name, -copy->_max_s/bMVA);
                        this->qb_max.add_val(copy->_name, copy->_max_s/bMVA);
                        auto nb_eff_pieces = copy->_x_eff.size() - 1;
                        if(_nb_eff_pieces<nb_eff_pieces){
                            _nb_eff_pieces = nb_eff_pieces;
                        }
                        for (int ph = 1; ph<=3; ph++) {
                            if(bus->_phases.count(ph)==0 || _all_battery_inverters[index]._phases.count(ph)==0){
                                continue;
                            }
                            B_ph.insert("ph"+to_string(ph)+","+copy->_name);
                            pot_B_ph.insert("ph"+to_string(ph)+","+copy->_name);
                            for(auto p =1; p<= _nb_eff_pieces;p++){
                                auto str = "ph"+to_string(ph)+","+copy->_name+",eff"+to_string(p);
                                if (p > nb_eff_pieces) {
                                    eff_a.add_val(str, (copy->_y_eff[nb_eff_pieces] - copy->_y_eff[nb_eff_pieces-1])/(copy->_x_eff[nb_eff_pieces] - copy->_x_eff[nb_eff_pieces-1]));
                                    eff_b.add_val(str, copy->_y_eff[nb_eff_pieces] - eff_a.eval()*copy->_x_eff[nb_eff_pieces]);
                                }
                                else{
                                    eff_a.add_val(str, (copy->_y_eff[p] - copy->_y_eff[p-1])/(copy->_x_eff[p] - copy->_x_eff[p-1]));
                                    eff_b.add_val(str, copy->_y_eff[p] - eff_a.eval()*copy->_x_eff[p]);
                                }
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
            _eff_pieces.insert("eff"+to_string(i));
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
            name = months._keys->at(m);
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
                _months_data[m]._wind_average[t] = 0.0006*0.42*row[t+1].value<double>()/bMVA;
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
                _months_data[m]._wind_variance[t] = 0.0006*0.42*row[t+1].value<double>()/bMVA;
            }
        }
    }
    else{
        for (unsigned m = 0; m<12; m++) {
            for (unsigned t=0; t<24; t++) {
                _months_data[m]._wind_variance[t] = 0.0006*0.42*_months_data[m]._wind_average[t]/10.;
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
            auto new_pv = b->_pv.back();
            auto name = new_pv->_name;
            this->pv_max.add_val(name, potential_PV/bMVA);
            this->pv_min.add_val(name, 0);
            this->pv_capcost.add_val(name,_pv_cap_cost);
            this->pv_varcost.add_val(name,_pv_cap_cost/bMVA);
            for (unsigned y = 0; y<_nb_years; y++) {
                for (unsigned m = 0; m<this->months.size(); m++) {
                    for (unsigned t=0; t<_nb_hours; t++) {
                        for (auto &ph: new_pv->_phases){
                            auto key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",week," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pv_out.add_val(key, _months_data[m]._solar_average[t]);
                            key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",peak," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pv_out.add_val(key, _months_data[m]._solar_average[t] + _months_data[m]._solar_variance[t]);
                            key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",weekend," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pv_out.add_val(key, _months_data[m]._solar_average[t]);
                        }
                            
                    }
                }
            }
        }
        if (b->_existing_PV_cap > 0) {
            auto new_pv = b->_pv.front();
            auto name = new_pv->_name;
            this->pv_max.add_val(name, b->_existing_PV_cap/bMVA);
            this->pv_min.add_val(name, 0);
            for (unsigned y = 0; y<_nb_years; y++) {
                for (unsigned m = 0; m<this->months.size(); m++) {
                    for (unsigned t=0; t<_nb_hours; t++) {
                        for (auto &ph: new_pv->_phases){
                            auto key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",week," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pv_out.add_val(key, _months_data[m]._solar_average[t]);
                            key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",peak," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pv_out.add_val(key, _months_data[m]._solar_average[t] + _months_data[m]._solar_variance[t]);
                            key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",weekend," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pv_out.add_val(key, _months_data[m]._solar_average[t]);
                        }
                    }
                }
            }
        }
        if (potential_wind>0) {
            auto new_wind = b->_wind.back();
            auto name = new_wind->_name;
            for (unsigned y = 0; y<_nb_years; y++) {
                for (unsigned m = 0; m<this->months.size(); m++) {
                    for (unsigned t=0; t<_nb_hours; t++) {
                        for (auto &ph: {1,2,3}){
                            auto key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",week," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pw_max.add_val(key, _months_data[m]._wind_average[t]);
                            key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",peak," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pw_max.add_val(key, _months_data[m]._wind_average[t] +  _months_data[m]._wind_variance[t]);
                            key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",weekend," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pw_max.add_val(key, _months_data[m]._wind_average[t] +  _months_data[m]._wind_variance[t]);
                        }
                    }
                }
            }
        }
        if (b->_existing_Wind_cap>0) {
            auto new_wind = b->_wind.front();
            auto name = new_wind->_name;
            for (unsigned y = 0; y<_nb_years; y++) {
                for (unsigned m = 0; m<this->months.size(); m++) {
                    for (unsigned t=0; t<_nb_hours; t++) {
                        for (auto &ph: {1,2,3}){
                            auto key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",week," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pw_max.add_val(key, _months_data[m]._wind_average[t]);
                            key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",peak," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pw_max.add_val(key, _months_data[m]._wind_average[t] +  _months_data[m]._wind_variance[t]);
                            key = "year" + to_string(y+1) + "," + this->months._keys->at(m) + ",weekend," + to_string(t+1) + ",ph" + to_string(ph) + "," + name;
                            pw_max.add_val(key, _months_data[m]._wind_average[t] +  _months_data[m]._wind_variance[t]);
                        }
                    }
                }
            }
        }
    }
    //    clog << "Loads:\n";
    //    pl.print(true);
    clog << "Reading excel file complete" << std::endl;
    return 0;
}


PowerNet* PowerNet::clone(int net_id) const{
    PowerNet* copy_net = new PowerNet();
    Bus* node = NULL;
    
    for (int i=0; i<nodes.size(); i++) {
        node = (Bus*)this->nodes[i];
        if(node->_net_id==net_id){
            copy_net->add_node(new Bus(*node));
        }
    }
    
    Line* arc = NULL;
    for (int i=0; i < arcs.size(); i++) {
        
        arc = (Line*)arcs[i];
        if(arc->_tie_line){
            continue;
        }
        arc = new Line(*arc);
        /* Update the source and destination to the new nodes in copy_net */
        arc->_src = copy_net->get_node(arc->_src->_name);
        arc->_dest = copy_net->get_node(arc->_dest->_name);
        
        /* Add the new arc to the list of arcs */
        copy_net->add_arc(arc);
        
        /* Connects it to its source and destination */
        arc->connect();
    }
    return copy_net;
}


vector<shared_ptr<PowerNet>> PowerNet::get_separate_microgrids() const{
    
    DebugOn("Number of Microgrids detected = " << _microgrid_ids.size() << endl);
    vector<shared_ptr<PowerNet>> res;
    for (auto net_id: _microgrid_ids) {
        res.push_back(shared_ptr<PowerNet>(this->clone(net_id)));
    }
    return res;
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
    nb_nodes = buses.MemberCount();
    DebugOn("nb_nodes = " << nb_nodes << endl);
    for (auto& v : buses.GetObject()) {
        Value& list = v.value;
        auto btype = list["bus_type"].GetInt();
        Debug("btype = " << btype << endl);
        auto index = list["index"].GetInt();
        auto net_id = list["microgrid_id"].GetInt();
        auto name = string(list["name"].GetString());
        auto status =  list["status"].GetInt();
        auto va =  list["va"].GetArray();
        auto vm =  list["vm"].GetArray();
        auto vmax =  list["vmax"].GetArray();
        auto vmin =  list["vmin"].GetArray();
        auto bus = new Bus(to_string(index));
        bus->vbound.min = vmin[0].GetDouble();
        bus->vbound.max = vmax[0].GetDouble();
        bus->_net_id = net_id;
        _microgrid_ids.insert(net_id);
//        vm_s_.add_val("ph1,"+bus->_name, 0.9999742573517363);
//        vm_s_.add_val("ph2,"+bus->_name, 1.0000038391442767);
//        vm_s_.add_val("ph3,"+bus->_name, 0.9999622416976457);
        vm_s_.add_val("ph1,"+bus->_name, vm[0].GetDouble());
        vm_s_.add_val("ph2,"+bus->_name, vm[1].GetDouble());
        vm_s_.add_val("ph3,"+bus->_name, vm[2].GetDouble());
        //        theta_s_.add_val(bus->_name+",ph0", va[0].GetDouble());
        //        theta_s_.add_val(bus->_name+",ph1", va[1].GetDouble());
        //        theta_s_.add_val(bus->_name+",ph2", va[2].GetDouble());
        //        theta_s_.add_val(bus->_name+",ph0", 0);
        //        theta_s_.add_val(bus->_name+",ph1", -2.094395);
        //        theta_s_.add_val(bus->_name+",ph2", 2.094395);
//        theta_s_.add_val("ph1,"+bus->_name, 30.*pi/180.);
//        theta_s_.add_val("ph2,"+bus->_name, -90.*pi/180.);
//        theta_s_.add_val("ph3,"+bus->_name, 150.*pi/180.);
        for(auto i = 0;i<3;i++){
            auto ph_key = "ph"+to_string(i+1)+","+bus->_name;
            if(vm[i].GetDouble()!=0){
                bus->_phases.insert(i+1);
                v_min.add_val(ph_key, vmin[i].GetDouble());
                v_max.add_val(ph_key, vmax[i].GetDouble());
                N_ph.insert(ph_key);
                if(i==0){
                    N_ph1.insert(ph_key);
                }
                else if(i==1){
                    N_ph2.insert(ph_key);
                }
                else {
                    N_ph3.insert(ph_key);
                }
                gs_.add_val(ph_key, 0);
                bs_.add_val(ph_key, 0);
                pl.add_val(ph_key, 0);
                ql.add_val(ph_key, 0);
            }
            else {
                DebugOn("excluding bus: " << bus->_name << " on phase " << i+1 << endl);
            }
        }
        
        
//        theta_.add_val(bus->_name+",ph0", va[0].GetDouble());
//        theta_.add_val(bus->_name+",ph1", va[1].GetDouble());
//        theta_.add_val(bus->_name+",ph2", va[2].GetDouble());
        bus->_active = status;
        bus->_id = nodes.size();
        bus->_type = btype;
        if (btype==3) {
            ref_bus = bus->_name;
//            theta_s_.add_val("ph1,"+bus->_name, std::tan(0));
//            theta_s_.add_val("ph2,"+bus->_name, std::tan(-2*pi/3));
//            theta_s_.add_val("ph3,"+bus->_name, std::tan(2*pi/3));
            theta_s_.add_val("ph1,"+bus->_name, 0);
            theta_s_.add_val("ph2,"+bus->_name, (-2*pi/3));
            theta_s_.add_val("ph3,"+bus->_name, (2*pi/3));

        }
        assert(bus->_id<nb_nodes);
        if (!nodeID.insert(pair<string,Node*>(bus->_name, bus)).second) {
            throw invalid_argument("ERROR: adding the same bus twice!");
        }
        nodes.push_back(bus);
        if (!bus->_active) {
            DebugOn("INACTIVE NODE: " << name << endl);
        }
    }
    Value& branches = d["branch"];
    nb_branches = branches.MemberCount();
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
        auto tr = string(list["transformer"].GetString());
        
        auto arc = new Line(to_string(index) + "," + to_string(f_bus)+","+to_string(t_bus)); // Name of lines
        arc->_id = index-1;
        arc->_src = get_node(to_string(f_bus));
        arc->_dest= get_node(to_string(t_bus));
        arc->_tie_line = arc->_src->_net_id!=arc->_dest->_net_id;
        if(arc->_tie_line){
            DebugOn("Tie line: " << arc->_name << endl);
        }
        arc->status = status;
        arc->_len = length;
        arc->_is_transformer = (tr=="true");
        arc->connect();
        add_arc(arc);
        _exist_arcs.push_back(arc);
        auto ymat = arma::cx_mat(3,3);
        for (auto i = 0; i<3; i++) {
            auto ph_key = "ph"+to_string(i+1)+","+arc->_name;
            auto src = arc->_src->_name;
            auto dest = arc->_dest->_name;
            if (arc->_src->_phases.count(i+1)!=0 && arc->_dest->_phases.count(i+1)!=0) {
                exist_E_ph.insert(ph_key);
                E_ph.insert(ph_key);
                if(i==0){
                    E_ph1.insert(ph_key);
                }
                else if(i==1){
                    E_ph2.insert(ph_key);
                }
                else {
                    E_ph3.insert(ph_key);
                }
                arc->_phases.insert(i+1);
            }
            else {
                DebugOn("excluding arc: " << arc->_name << " on phase " << i+1 << endl);
                continue;
            }
            r.add_val(ph_key, br_r[i][i].GetDouble());
            x.add_val(ph_key, br_x[i][i].GetDouble());
            arc->r = std::max(arc->r, r.eval());
            arc->x = std::max(arc->x, x.eval());
            for (auto j = 0; j<3; j++) {
                br_r_.add_val(arc->_name+","+to_string(i)+","+to_string(j), br_r[i][j].GetDouble());
                br_x_.add_val(arc->_name+","+to_string(i)+","+to_string(j), br_x[i][j].GetDouble());
                ymat(i,j) = complex<double>(br_r_.eval(),br_x_.eval());
            }
            
            b_fr_.add_val(ph_key, b_fr[i].GetDouble());
            b_to_.add_val(ph_key, b_to[i].GetDouble());
            g_fr_.add_val(ph_key, g_fr[i].GetDouble());
            g_to_.add_val(ph_key, g_to[i].GetDouble());
            shift_.add_val(ph_key, shift[i].GetDouble());
            tap_.add_val(ph_key, tap[i].GetDouble());
            this->S_max.add_val(ph_key, rating[i].GetDouble());
        }
//        br_r_.print();
        arma::cx_mat ymat_inv = arma::pinv(ymat);
        if(arc->_phases.count(1)>0){
            Yr11.add_val("ph1,"+arc->_name, ymat_inv(0,0).real()/3.);
            Yi11.add_val("ph1,"+arc->_name, -ymat_inv(0,0).imag()/3.);
            Yr12.add_val("ph1,"+arc->_name, ymat_inv(0,1).real()/3.);
            Yi12.add_val("ph1,"+arc->_name, -ymat_inv(0,1).imag()/3.);
            Yr13.add_val("ph1,"+arc->_name, ymat_inv(0,2).real()/3.);
            Yi13.add_val("ph1,"+arc->_name, -ymat_inv(0,2).imag()/3.);
        }
        if(arc->_phases.count(2)>0){
            Yr21.add_val("ph2,"+arc->_name, ymat_inv(1,0).real()/3.);
            Yi21.add_val("ph2,"+arc->_name, -ymat_inv(1,0).imag()/3.);
            Yr22.add_val("ph2,"+arc->_name, ymat_inv(1,1).real()/3.);
            Yi22.add_val("ph2,"+arc->_name, -ymat_inv(1,1).imag()/3.);
            Yr23.add_val("ph2,"+arc->_name, ymat_inv(1,2).real()/3.);
            Yi23.add_val("ph2,"+arc->_name, -ymat_inv(1,2).imag()/3.);
        }
        if(arc->_phases.count(3)>0){
            Yr31.add_val("ph3,"+arc->_name, ymat_inv(2,0).real()/3.);
            Yi31.add_val("ph3,"+arc->_name, -ymat_inv(2,0).imag()/3.);
            Yr32.add_val("ph3,"+arc->_name, ymat_inv(2,1).real()/3.);
            Yi32.add_val("ph3,"+arc->_name, -ymat_inv(2,1).imag()/3.);
            Yr33.add_val("ph3,"+arc->_name, ymat_inv(2,2).real()/3.);
            Yi33.add_val("ph3,"+arc->_name, -ymat_inv(2,2).imag()/3.);
        }
        
        for (auto i= 0; i<3; i++) {
            if(arc->_phases.count(i+1)==0)
                continue;
            for (auto j= 0; j<3; j++) {
                if(arc->_phases.count(j+1)==0)
                    continue;
                auto key = "ph"+to_string(i+1)+",ph"+to_string(j+1)+","+arc->_name;
                cross_phase.insert(key);
                g.add_val(key, ymat_inv(i,j).real());
                b.add_val(key, ymat_inv(i,j).imag());
            }
        }
//        g.print(true);
//        b.print(true);
        
    }
    
    Value& generatos = d["generator"];
    nb_gens = generatos.MemberCount();
    DebugOn("nb_gens = " << nb_gens << endl);
    for (auto& g : generatos.GetObject()) {
        Value& list = g.value;
        auto gbus = list["gen_bus"].GetInt();
        Debug("bus = " << gbus << endl);
        auto index = list["index"].GetInt();
        auto name = string(list["name"].GetString());
        auto status =  list["status"].GetInt();
        auto cost =  list["cost"].GetArray();
        auto pg =  list["pg"].GetArray();
        auto qg =  list["qg"].GetArray();
        auto qgmax =  list["qmax"].GetArray();
        auto qgmin =  list["qmin"].GetArray();
        auto pgmax =  list["pmax"].GetArray();
        auto pgmin =  list["pmin"].GetArray();
        auto bus = (Bus*)get_node(to_string(gbus));
        bus->_has_gen = true;
        name = "Existing_Gen," + bus->_name + "," + "slot"+to_string(index-1);
        Gen* gen = new Gen(bus, name, 0, 0, 0, 0);
        gen->_id = index-1;
        gen->_active = status;
        gen->_phases = {1,2,3};
        gens.push_back(gen);
        bus->_gen.push_back(gen);
        _existing_diesel_gens.push_back(gen);
        
        if(!bus->_active) {
            DebugOff("INACTIVE GENERATOR GIVEN INACTIVE BUS: " << gen->_name << endl);
            gen->_active = false;
        }
        else if (!gen->_active) {
            DebugOn("INACTIVE GENERATOR: " << name << endl);
        }
        this->c0.add_val(name,cost[0].GetDouble());
        this->c1.add_val(name,cost[1].GetDouble());
        this->c2.add_val(name,cost[2].GetDouble());
        this->gen_eff.add_val(gen->_name,1);
        if(bus->_type==3){
            pg_min.add_val(gen->_name, 0);
            pg_max.add_val(gen->_name, numeric_limits<double>::max());
            qg_min.add_val(gen->_name, numeric_limits<double>::lowest());
            qg_max.add_val(gen->_name, numeric_limits<double>::max());
        }
        else {
            pg_min.add_val(gen->_name, pgmin[0].GetDouble());
            pg_max.add_val(gen->_name, pgmax[0].GetDouble());
            qg_min.add_val(gen->_name, qgmin[0].GetDouble());
            qg_max.add_val(gen->_name, qgmax[0].GetDouble());
        }
        for(auto i=0; i<3; i++){
            auto ph_key = "ph"+to_string(i+1)+","+gen->_name;
            G_ph.insert(ph_key);
            exist_G_ph.insert(ph_key);
            pg_.add_val(ph_key, pg[i].GetDouble());
            qg_.add_val(ph_key, qg[i].GetDouble());
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
        auto bus = (Bus*)get_node(to_string(lbus));
        auto name = bus->_name;
        for(auto i=0; i<3; i++){
            auto ph_key = "ph"+to_string(i+1)+","+name;
            if(bus->has_phase(i+1)){
                bus->_cond[i]->_pl = pd[i].GetDouble();
                bus->_cond[i]->_ql = qd[i].GetDouble();
                if(status){
                    pl.set_val(ph_key, pd[i].GetDouble());
                    ql.set_val(ph_key, qd[i].GetDouble());
                }
                else {
                    pl.set_val(ph_key, 0);
                    ql.set_val(ph_key, 0);
                }
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
        auto bus = (Bus*)get_node(to_string(bus_id));        
        for (auto i = 0; i<3; i++) {
            auto ph_key = "ph"+to_string(i+1)+","+bus->_name;
            bus->_cond[i]->_bs = bs[i].GetDouble();
            bus->_cond[i]->_gs = gs[i].GetDouble();
            gs_.add_val(ph_key, bus->_cond[i]->_gs);
            bs_.add_val(ph_key, bus->_cond[i]->_bs);
            //            gs_.add_val(bus->_name+",ph"+to_string(i+1), 0);
            //            bs_.add_val(bus->_name+",ph"+to_string(i+1), 0);
        }
        
    }
}

void PowerNet::time_expand(const indices& T) {
    //    c0.time_expand(T);
    //    c1.time_expand(T);
    //    c2.time_expand(T);
//    pl._time_extended = true;
//    pl_ratio._time_extended = true;
//    ql._time_extended = true;
//    pv_out._time_extended = true;
//    pw_min._time_extended = true;
//    pw_max._time_extended = true;
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

shared_ptr<Model<>> PowerNet::build_ODO_model(PowerModelType pmt, int output, double tol, int max_nb_hours, bool networked){
    
   
    /* Grid Parameters */
    _nb_hours = max_nb_hours;
    
    /** Indices Sets */
    hours = time(1,max_nb_hours); /**< Hours */
    hours._name = "hours";
//            indices months = time("jan","feb","mar","apr","may","jun","jul","aug","sep","oct","nov","dec"); /**< Months */
    //    indices months = time("jan","feb","mar","apr","may","jun"); /**< Months */
    //    months = time("apr", "aug", "dec"); /**< Months */
//    indices months = time("jan", "feb");
//    indices years = time("year1", "year2", "year3");
    indices years = time("year1");
    years._name = "years";
//    indices months = time("summer", "spring", "autumn", "winter");
    months._name = "months";
    indices phases = indices("ph1","ph2","ph3");
    phases._name = "phases";
//    typical_days = time("week","peak","weekend");
    typical_days = time("week");
    typical_days._name = "typical_days";
    T = indices(years,months,typical_days,hours);
    double nT = T.size();
    DebugOn("number of time periods = " << nT << endl);
    Nt = indices(T,N_ph);
    Et = indices(T,E_ph);
    Et1 = indices(T,E_ph1);
    Et2 = indices(T,E_ph2);
    Et3 = indices(T,E_ph3);
    Gt = indices(T,G_ph);
    PVt = indices(T,PV_ph);
    Wt = indices(T,Wind_ph);
    exist_Gt = indices(T,exist_G_ph);
    exist_Bt = indices(T,exist_B_ph);
    exist_Et = indices(T,exist_E_ph);
    exist_PVt = indices(T,exist_PV_ph);
    exist_Windt = indices(T,exist_Wind_ph);
    pot_Gt = indices(T,pot_G_ph);
    pot_Bt = indices(T,pot_B_ph);
    pot_Et = indices(T,pot_E_ph);
    pot_PVt = indices(T,pot_PV_ph);
    pot_Windt = indices(T,pot_Wind_ph);
    Bt = indices(T,B_ph);
    /** Sets */
    auto bus_pairs = this->get_bus_pairs();
    auto gen_nodes = this->gens_per_node_time();
    auto batt_nodes = this->Batt_per_node_time();
    auto PV_nodes = this->PV_per_node_time();
    auto Wind_nodes = this->Wind_per_node_time();
    auto out_arcs = this->out_arcs_per_node_time();
    auto in_arcs = this->in_arcs_per_node_time();
    
    /** MODEL DECLARATION */
    shared_ptr<Model<>> ODO(new Model<>("ODO Model"));
    /** VARIABLES */


    /* Investment binaries */

    var<> Pv_cap("Pv_cap", 0, pv_max); /**< Real variable indicating the extra capacity of PV to be installed on bus b */
    ODO->add(Pv_cap.in(pot_PV_ph));
    var<int> w_g("w_g",0,1); /**< Binary variable indicating if generator g is built on bus */
    var<int> w_b("w_b",0,1); /**< Binary variable indicating if battery b is built on bus */
    var<int> w_e("w_e",0,1); /**< Binary variable indicating if expansion is selected for edge e */
    var<int> w_pv("w_pv",0,1); /**< Binary variable indicating if PV is installed on bus b */
    var<int> w_wind("w_wind",0,1); /**< Binary variable indicating if Wind is installed on bus b */
    ODO->add(w_g.in(pot_G_ph),w_b.in(pot_B_ph),w_e.in(pot_E_ph),w_pv.in(pot_PV_ph),w_wind.in(pot_Wind_ph));
    w_g.initialize_all(1);
    w_b.initialize_all(1);
    w_e.initialize_all(1);
    w_pv.initialize_all(1);
    w_wind.initialize_all(1);

    this->w_g = w_g;
    this->w_b = w_b;
    this->w_e = w_e;
    this->w_pv = w_pv;
    this->w_wind = w_wind;
    this->Pv_cap = Pv_cap;

    DebugOff("size w_g = " << w_g.get_dim() << endl);
    DebugOff("size w_b = " << w_b.get_dim() << endl);
    DebugOff("size w_e = " << w_e.get_dim() << endl);
    DebugOff("size w_pv = " << w_pv.get_dim() << endl);
    DebugOff("size w_wind = " << w_wind.get_dim() << endl);
    DebugOff("size Pv_cap = " << Pv_cap.get_dim() << endl);


    /* Diesel power generation variables */
    var<> Pg("Pg", pg_min.in(Gt), pg_max.in(Gt));
    var<> Qg ("Qg", qg_min.in(Gt), qg_max.in(Gt));
    var<> Pg_ ("Pg_", pg_min.in(Gt), pg_max.in(Gt));/**< Active power generation before losses */
    var<> Pg2("Pg2", 0, pow(pg_max.in(pot_Gt),2));/**< Square of Pg */
    ODO->add(Pg.in(Gt));
    ODO->add(Pg_.in(Gt));
    ODO->add(Qg.in(Gt));
    ODO->add(Pg2.in(pot_Gt));
    DebugOff("size Pg = " << Pg.get_dim() << endl);
    DebugOff("size Pg_ = " << Pg_.get_dim() << endl);
    DebugOff("size Qg = " << Qg.get_dim() << endl);
    DebugOff("size Pg2 = " << Pg2.get_dim() << endl);

    this->Pg_ = Pg_;

    /* Battery power generation variables */
    var<> Pb("Pb", pb_min.in(Bt), pb_max.in(Bt));/**< Active power generation outside the battery */
    var<> Qb ("Qb", qb_min.in(Bt), qb_max.in(Bt));/**< Reactive power generation outside the battery */
    var<> Pb_("Pb_", pb_min.in(Bt), pb_max.in(Bt));/**< Active power generation in the battery */
    ODO->add(Pb.in(Bt), Qb.in(Bt), Pb_.in(Bt));
    DebugOff("size Pb = " << Pb.get_dim() << endl);
    DebugOff("size Qb = " << Qb.get_dim() << endl);


    /* PV power generation variables */
    var<> Pv("Pv", 0,pv_max.in(PVt));
    ODO->add(Pv.in(PVt));
    DebugOff("size Pv = " << Pv.get_dim() << endl);

    /* Battery state of charge variables */
    var<> Sc("Sc", pos_);
    ODO->add(Sc.in(Bt));
    DebugOff("size Sc = " << Sc.get_dim() << endl);

    /* Wind power generation variables */
    var<> Pw("Pw", 0, pw_max.in(Wt));
    ODO->add(Pw.in(Wt));
    DebugOff("size Pw = " << Pw.get_dim() << endl);

    /* Power flow variables */
    var<> Pij("Pfrom", -1*S_max.in(Et), S_max.in(Et));
    var<> Qij("Qfrom", -1*S_max.in(Et), S_max.in(Et));
    var<> Pji("Pto", -1*S_max.in(Et), S_max.in(Et));
    var<> Qji("Qto", -1*S_max.in(Et), S_max.in(Et));

    ODO->add(Pij.in(Et),Pji.in(Et),Qij.in(Et),Qji.in(Et));
    DebugOff("size Pij = " << Pij.get_dim() << endl);
    if (pmt!=LDISTF) {
        ODO->add(Pji.in(Et),Qji.in(Et));
    }

    /** Voltage magnitude (squared) variables */
    var<> v("v", v_min.in(Nt), v_max.in(Nt));
    var<> theta("𝛉");
    var<> vr("vr", -1*v_max.in(Nt),v_max.in(Nt));
    var<> vi("vi", -1*v_max.in(Nt),v_max.in(Nt));
    
    var<> v_fr, v_to, theta_fr, theta_to;
    var<> v_fr1, v_to1, theta_fr1, theta_to1;
    var<> v_fr2, v_to2, theta_fr2, theta_to2;
    var<> v_fr3, v_to3, theta_fr3, theta_to3;
    var<> vr_fr, vr_to, vi_fr, vi_to;
    var<> vr_fr1,vr_fr2,vr_fr3,vi_fr1,vi_fr2,vi_fr3;
    var<> vr_to1,vr_to2,vr_to3,vi_to1,vi_to2,vi_to3;
    
    if (pmt==ACPOL) {
        ODO->add(v.in(Nt));
        ODO->add(theta.in(Nt));
        Debug("size v = " << v.get_dim() << endl);
        v.initialize_all(1);
        v_fr = v.from(Et);
        v_to = v.to(Et);
        theta_fr = theta.from(Et);
        theta_to = theta.to(Et);
        /* Indexing the voltage variables */
        v_fr1 = v.from(Et1); theta_fr1 = theta.from(Et1);
        v_fr2 = v.from(Et2); theta_fr2 = theta.from(Et2);
        v_fr3 = v.from(Et3); theta_fr3 = theta.from(Et3);
        v_to1 = v.to(Et1); theta_to1 = theta.to(Et1);
        v_to2 = v.to(Et2); theta_to2 = theta.to(Et2);
        v_to3 = v.to(Et3); theta_to3 = theta.to(Et3);
    }
    else if (pmt==ACRECT) {
        ODO->add(vr.in(Nt));
        ODO->add(vi.in(Nt));
        vr.initialize_all(1);
        vr_fr = vr.from(Et);
        vr_to = vr.to(Et);
        vi_fr = vi.from(Et);
        vi_to = vi.to(Et);
        
        /* Indexing the voltage variables */
        vr_fr1 = vr.from(Et1); vi_fr1 = vi.from(Et1);
        vr_fr2 = vr.from(Et2); vi_fr2 = vi.from(Et2);
        vr_fr3 = vr.from(Et3); vi_fr3 = vi.from(Et3);
        vr_to1 = vr.to(Et1); vi_to1 = vi.to(Et1);
        vr_to2 = vr.to(Et2); vi_to2 = vi.to(Et2);
        vr_to3 = vr.to(Et3); vi_to3 = vi.to(Et3);
        
    }
    auto Pij1 = Pij.in(Et1);auto Pij2 = Pij.in(Et2);auto Pij3 = Pij.in(Et3);
    auto Pji1 = Pji.in(Et1);auto Pji2 = Pji.in(Et2);auto Pji3 = Pji.in(Et3);
    auto Qij1 = Qij.in(Et1);auto Qij2 = Qij.in(Et2);auto Qij3 = Qij.in(Et3);
    auto Qji1 = Qji.in(Et1);auto Qji2 = Qji.in(Et2);auto Qji3 = Qji.in(Et3);
    /** Indices */
    auto branch_id_ph1 = get_branch_id_phase(1);
    branch_id_ph1.extend(T);
    auto branch_id_ph2 = get_branch_id_phase(2);
    branch_id_ph2.extend(T);
    auto branch_id_ph3 = get_branch_id_phase(3);
    branch_id_ph3.extend(T);
    auto branch_ph1 = get_branch_phase(1);
    branch_ph1.extend(T);
    auto branch_ph2 = get_branch_phase(2);
    branch_ph2.extend(T);
    auto branch_ph3 = get_branch_phase(3);
    branch_ph3.extend(T);
    auto ref_from_ph1 = fixed_from_branch_phase(1);
    auto ref_from_ph2 = fixed_from_branch_phase(2);
    auto ref_from_ph3 = fixed_from_branch_phase(3);
    auto ref_to_ph1 = fixed_to_branch_phase(1);
    auto ref_to_ph2 = fixed_to_branch_phase(2);
    auto ref_to_ph3 = fixed_to_branch_phase(3);
    auto from_ph1 = from_branch_phase(1);
    auto from_ph2 = from_branch_phase(2);
    auto from_ph3 = from_branch_phase(3);
    auto to_ph1 = to_branch_phase(1);
    auto to_ph2 = to_branch_phase(2);
    auto to_ph3 = to_branch_phase(3);
    
    /** Power Flows */
    param<Cpx> Y0("Y0"), Y1("Y1"), Y2("Y2"), Y3("Y3");
    param<Cpx> Yc_fr("Yc_fr"), Yc_to("Yc_to");/* Line charging */
    var<Cpx> Vfr("Vfr"), Vto("Vto");
    var<Cpx> Sij("Sij"), Sji("Sji"), Vi("Vi"), Vj("Vj"), Vi1("Vi1"), Vi2("Vi2"), Vi3("Vi3"), Vj1("Vj1"), Vj2("Vj2"), Vj3("Vj3");
    /* Phase 1 */
    Yc_fr.real_imag(g_fr_.in(Et1),b_fr_.in(Et1));
    Yc_to.real_imag(g_to_.in(Et1),b_to_.in(Et1));
    Y0.real_imag(g.in(branch_id_ph1),b.in(branch_id_ph1));
    Y1.real_imag(g.in(branch_ph1),b.in(branch_ph1));
    if(pmt==ACRECT){
        Vfr.real_imag(vr_fr1,vi_fr1);
        Vto.real_imag(vr_to1,vi_to1);
        Vi.real_imag(vr.in(ref_from_ph1),vi.in(ref_from_ph1));
        Vj.real_imag(vr.in(ref_to_ph1),vi.in(ref_to_ph1));
        Vi1.real_imag(vr.in(from_ph1),vi.in(from_ph1));
        Vj1.real_imag(vr.in(to_ph1),vi.in(to_ph1));
    }
    else if(pmt==ACPOL){
        Vfr.mag_ang(v_fr1,theta_fr1);
        Vto.mag_ang(v_to1,theta_to1);
        Vi.mag_ang(v.in(ref_from_ph1),theta.in(ref_from_ph1));
        Vj.mag_ang(v.in(ref_to_ph1),theta.in(ref_to_ph1));
        Vi1.mag_ang(v.in(from_ph1),theta.in(from_ph1));
        Vj1.mag_ang(v.in(to_ph1),theta.in(to_ph1));
    }
    Sij.real_imag(Pij1,Qij1);
    Sji.real_imag(Pji1,Qji1);
    
    
    Constraint<Cpx> S_fr1("S_fr1"), S_to1("S_to1");
    S_fr1 = Sij - (conj(Y0)+conj(Yc_fr))*Vfr*conj(Vfr) + conj(Y0)*Vfr*conj(Vto) - (conj(Y1)*Vi)*(conj(Vi1) - conj(Vj1));
    S_to1 = Sji - (conj(Y0)+conj(Yc_to))*Vto*conj(Vto) + conj(Y0)*Vto*conj(Vfr) - (conj(Y1)*Vj)*(conj(Vj1) - conj(Vi1));
    ODO->add(S_fr1.in(Et1)==0);
    ODO->add(S_to1.in(Et1)==0);
    /* Phase 2 */
    Yc_fr.real_imag(g_fr_.in(Et2),b_fr_.in(Et2));
    Yc_to.real_imag(g_to_.in(Et2),b_to_.in(Et2));
    Y0.real_imag(g.in(branch_id_ph2),b.in(branch_id_ph2));
    Y2.real_imag(g.in(branch_ph2),b.in(branch_ph2));
    
    if(pmt==ACRECT){
        Vfr.real_imag(vr_fr2,vi_fr2);
        Vto.real_imag(vr_to2,vi_to2);
        Vi.real_imag(vr.in(ref_from_ph2),vi.in(ref_from_ph2));
        Vj.real_imag(vr.in(ref_to_ph2),vi.in(ref_to_ph2));
        Vi2.real_imag(vr.in(from_ph2),vi.in(from_ph2));
        Vj2.real_imag(vr.in(to_ph2),vi.in(to_ph2));
    }
    else if(pmt==ACPOL){
        Vfr.mag_ang(v_fr2,theta_fr2);
        Vto.mag_ang(v_to2,theta_to2);
        Vi.mag_ang(v.in(ref_from_ph2),theta.in(ref_from_ph2));
        Vj.mag_ang(v.in(ref_to_ph2),theta.in(ref_to_ph2));
        Vi2.mag_ang(v.in(from_ph2),theta.in(from_ph2));
        Vj2.mag_ang(v.in(to_ph2),theta.in(to_ph2));
    }
    
    Sij.real_imag(Pij2,Qij2);
    Sji.real_imag(Pji2,Qji2);
    Constraint<Cpx> S_fr2("S_fr2"), S_to2("S_to2");
    S_fr2 = Sij - (conj(Y0)+conj(Yc_fr))*Vfr*conj(Vfr) + conj(Y0)*Vfr*conj(Vto) - (conj(Y2)*Vi)*(conj(Vi2) - conj(Vj2));
    S_to2 = Sji - (conj(Y0)+conj(Yc_to))*Vto*conj(Vto) + conj(Y0)*Vto*conj(Vfr) - (conj(Y2)*Vj)*(conj(Vj2) - conj(Vi2));
    ODO->add(S_fr2.in(Et2)==0);
    ODO->add(S_to2.in(Et2)==0);
    /* Phase 3 */
    Yc_fr.real_imag(g_fr_.in(Et3),b_fr_.in(Et3));
    Yc_to.real_imag(g_to_.in(Et3),b_to_.in(Et3));
    Y0.real_imag(g.in(branch_id_ph3),b.in(branch_id_ph3));
    Y3.real_imag(g.in(branch_ph3),b.in(branch_ph3));
    if(pmt==ACRECT){
        Vfr.real_imag(vr_fr3,vi_fr3);
        Vto.real_imag(vr_to3,vi_to3);
        Vi.real_imag(vr.in(ref_from_ph3),vi.in(ref_from_ph3));
        Vj.real_imag(vr.in(ref_to_ph3),vi.in(ref_to_ph3));
        Vi3.real_imag(vr.in(from_ph3),vi.in(from_ph3));
        Vj3.real_imag(vr.in(to_ph3),vi.in(to_ph3));
    }
    else if(pmt==ACPOL){
        Vfr.mag_ang(v_fr3,theta_fr3);
        Vto.mag_ang(v_to3,theta_to3);
        Vi.mag_ang(v.in(ref_from_ph3),theta.in(ref_from_ph3));
        Vj.mag_ang(v.in(ref_to_ph3),theta.in(ref_to_ph3));
        Vi3.mag_ang(v.in(from_ph3),theta.in(from_ph3));
        Vj3.mag_ang(v.in(to_ph3),theta.in(to_ph3));
    }
    
    Sij.real_imag(Pij3,Qij3);
    Sji.real_imag(Pji3,Qji3);
    Constraint<Cpx> S_fr3("S_fr3"), S_to3("S_to3");
    S_fr3 = Sij - (conj(Y0)+conj(Yc_fr))*Vfr*conj(Vfr) + conj(Y0)*Vfr*conj(Vto) - (conj(Y3)*Vi)*(conj(Vi3) - conj(Vj3));
    S_to3 = Sji - (conj(Y0)+conj(Yc_to))*Vto*conj(Vto) + conj(Y0)*Vto*conj(Vfr) - (conj(Y3)*Vj)*(conj(Vj3) - conj(Vi3));
    ODO->add(S_fr3.in(Et3)==0);
    ODO->add(S_to3.in(Et3)==0);
//        ODO->print();
//        exit(-1);
//
//    /** Power loss variables */
//    var<Real> ploss("ploss");
//    var<Real> qloss("qloss");
//    if (pmt==DISTF || pmt==CDISTF) {
//        ODO->add(ploss.in(Et));
//        ODO->add(qloss.in(Et));
//    }
//
    /** Loss constraint */
//    Constraint<> PLosses1("PLosses1");
//    PLosses1 = Pij + Pji;
//    ODO->add(PLosses1.in(Et) >= 0.004149/nb_branches*Pij);
//
//    Constraint<> PLosses2("PLosses2");
//    PLosses2 = Pij + Pji;
//    ODO->add(PLosses2.in(Et) >= 0.004149/nb_branches*Pji);
//
//    Constraint<> QLosses("QLosses");
//    QLosses = Qij + Qji;
//    ODO->add(QLosses.in(Et) >= 0);
//
    /** OBJECTIVE FUNCTION */
    //check pot_gen
    func<> obj = product(c1.in(exist_Gt), Pg_.in(exist_Gt)) + product(c1.in(pot_Gt), Pg_.in(pot_Gt)) + product(c2.in(exist_Gt), pow(Pg_.in(exist_Gt),2)) + product(c2.in(pot_Gt), Pg2.in(pot_Gt)) + sum(c0.in(exist_Gt));
    //    obj *= 12./months.size();
//    obj += nT*product(c0.in(pot_G_ph),w_g);
    obj += 1e-3*product(gen_capcost.in(pot_G_ph), w_g);
    obj += 1e-3*product(inverter_capcost.in(pot_B_ph), w_b);
    obj += 1e-3*product(expansion_capcost.in(pot_E_ph), w_e);
    obj += 1e-3*product(pv_capcost.in(pot_PV_ph), w_pv);
    obj += 1e-3*product(pv_varcost.in(pot_PV_ph), Pv_cap);
    ODO->min(obj);
//    obj += sum(Pg);
//    ODO->min(sum(Pg));
//    func<> obj = product(c2.in(Gt), pow(Pg.in(Gt),2));
//    ODO->min(obj);

    /** CONSTRAINTS **/

    /** Voltage magnitude at source bus **/
    indices ref_id("ref_id");
    ref_id.insert(ref_bus);
    ref_id = indices(T,phases,ref_id);
    Constraint<> fix_voltage_mag("fix_voltage_mag");
    if(pmt==ACPOL){
        fix_voltage_mag += v.in(ref_id) - vm_s_;
    }
    else {
        fix_voltage_mag += pow(vr.in(ref_id),2) + pow(vi.in(ref_id),2) - pow(vm_s_.in(ref_id),2);
    }
//    ODO->add(fix_voltage_mag.in(ref_id)==0);


    /** Voltage angle at source bus **/
    Constraint<> fix_voltage_ang("fix_voltage_ang");
    if(pmt==ACPOL){
        fix_voltage_ang += theta.in(ref_id) - theta_s_.in(ref_id);
    }
    else {
        fix_voltage_ang += vi.in(ref_id) - theta_s_.in(ref_id)*vr.in(ref_id);
    }
//    ODO->add(fix_voltage_ang.in(ref_id)==0);

    /** FLOW CONSERVATION **/
    
    /** KCL Flow conservation */
    Constraint<> KCL_P("KCL_P");
    Constraint<> KCL_Q("KCL_Q");
    KCL_P  = sum(Pij, out_arcs) + sum(Pji, in_arcs) + pl.in(Nt) - sum(Pg, gen_nodes) - sum(Pv, PV_nodes) - sum(Pb, batt_nodes) - sum(Pw, Wind_nodes);
    KCL_Q  = sum(Qij, out_arcs) + sum(Qji, in_arcs) + ql.in(Nt) - sum(Qg, gen_nodes);
    if(pmt==ACPOL){
        KCL_P += gs_.in(Nt)*pow(v.in(Nt),2);
        KCL_Q -= bs_.in(Nt)*pow(v.in(Nt),2);
    }
    else if(pmt==ACRECT){
        KCL_P += gs_.in(Nt)*(pow(vr.in(Nt),2)+pow(vi.in(Nt),2));
        KCL_Q -= bs_.in(Nt)*(pow(vr.in(Nt),2)+pow(vi.in(Nt),2));
    }
    ODO->add(KCL_P.in(Nt) == 0);
    ODO->add(KCL_Q.in(Nt) == 0);

    /**  THERMAL LIMITS **/

    /*  Thermal Limit Constraints for existing lines */
    Constraint<> Thermal_Limit_from("Thermal_Limit_from");
    Thermal_Limit_from += pow(Pij.in(exist_Et), 2) + pow(Qij.in(exist_Et), 2);
    Thermal_Limit_from -= pow(S_max.in(exist_Et), 2);
    ODO->add(Thermal_Limit_from.in(exist_Et) <= 0);

    Constraint<> Thermal_Limit_to("Thermal_Limit_to");
    Thermal_Limit_to += pow(Pji.in(exist_Et), 2) + pow(Qji.in(exist_Et), 2);
    Thermal_Limit_to -= pow(S_max.in(exist_Et), 2);
    ODO->add(Thermal_Limit_to.in(exist_Et) <= 0);


    /*  Thermal Limit Constraints for expansion edges */
    Constraint<> Thermal_Limit_from_exp("Thermal_Limit_From_Exp");
    Thermal_Limit_from_exp += pow(Pij.in(pot_Et), 2) + pow(Qij.in(pot_Et), 2);
    Thermal_Limit_from_exp -= pow(w_e.in(pot_Et),2)*pow(S_max.in(pot_Et), 2);
    ODO->add(Thermal_Limit_from_exp.in(pot_Et) <= 0);
    
    Constraint<> Thermal_Limit_to_exp("Thermal_Limit_to_Exp");
    Thermal_Limit_to_exp += pow(Pji.in(pot_Et), 2) + pow(Qji.in(pot_Et), 2);
    Thermal_Limit_to_exp -= pow(w_e.in(pot_Et),2)*pow(S_max.in(pot_Et), 2);
    ODO->add(Thermal_Limit_to_exp.in(pot_Et) <= 0);

    /** AC voltage limit constraints. */
    if (pmt==ACRECT) {
        Constraint<> Vol_limit_UB("Vol_limit_UB");
        Vol_limit_UB = pow(vr.in(Nt), 2) + pow(vi.in(Nt), 2);
        Vol_limit_UB -= pow(v_max.in(Nt), 2);
        ODO->add(Vol_limit_UB.in(Nt) <= 0);
        
        Constraint<> Vol_limit_LB("Vol_limit_LB");
        Vol_limit_LB = pow(vr.in(Nt), 2) + pow(vi.in(Nt), 2);
        Vol_limit_LB -= pow(v_min.in(Nt),2);
        ODO->add(Vol_limit_LB.in(Nt) >= 0);
    }
    
    /**  GENERATOR INVESTMENT **/

    /*  On/Off status */
    Constraint<> OnOff_maxP("OnOff_maxP");
    OnOff_maxP += Pg_.in(pot_Gt) - pg_max.in(pot_Gt)*w_g.in(pot_Gt);
    ODO->add(OnOff_maxP.in(pot_Gt) <= 0);

    Constraint<> Perspective_OnOff("Perspective_OnOff");
    Perspective_OnOff += pow(Pg_.in(pot_Gt),2) - Pg2.in(pot_Gt)*w_g.in(pot_Gt);
    ODO->add(Perspective_OnOff.in(pot_Gt) <= 0);

    Constraint<> OnOff_maxQ("OnOff_maxQ");
    OnOff_maxQ += Qg.in(pot_Gt) - qg_max.in(pot_Gt)*w_g.in(pot_Gt);
    ODO->add(OnOff_maxQ.in(pot_Gt) <= 0);

    Constraint<> OnOff_maxQ_N("OnOff_maxQ_N");
    OnOff_maxQ_N += Qg.in(pot_Gt) - qg_min.in(pot_Gt)*w_g.in(pot_Gt);
    ODO->add(OnOff_maxQ_N.in(pot_Gt) >= 0);

     /**  PV **/

    /*  On/Off on Potential PV */
    Constraint<> OnOffPV("OnOffPV");
    OnOffPV += Pv_cap.in(pot_PV_ph) - w_pv*pv_max.in(pot_PV_ph);
    ODO->add(OnOffPV.in(pot_PV_ph) <= 0);

    /*  Max Cap on Potential PV */
    Constraint<> MaxCapPV("MaxCapPV");
    MaxCapPV += Pv.in(pot_PVt) - Pv_cap.in(pot_PVt)*pv_out.in(pot_PVt);
    ODO->add(MaxCapPV.in(pot_PVt) <= 0);

    /*  Existing PV */
    Constraint<> existPV("existPV");
    existPV += Pv.in(exist_PVt) - pv_max.in(exist_PVt)*pv_out.in(exist_PVt);
    ODO->add(existPV.in(exist_PVt) <= 0);


    /**  BATTERIES **/

    /*  Apparent Power Limit on Potential Batteries */
    Constraint<> Apparent_Limit_Batt_Pot("Apparent_Limit_Batt_Potential");
    Apparent_Limit_Batt_Pot += pow(Pb.in(pot_Bt), 2) + pow(Qb.in(pot_Bt), 2);
    Apparent_Limit_Batt_Pot -= pow(w_b.in(pot_Bt),2)*pow(pb_max.in(pot_Bt), 2);
    ODO->add(Apparent_Limit_Batt_Pot.in(pot_Bt) <= 0);

    /*  Apparent Power Limit on Existing Batteries */
    Constraint<> Apparent_Limit_Batt("Apparent_Limit_Batt_Existing");
    Apparent_Limit_Batt += pow(Pb.in(exist_Bt), 2) + pow(Qb.in(exist_Bt), 2);
    Apparent_Limit_Batt -= pow(pb_max.in(exist_Bt), 2);
    ODO->add(Apparent_Limit_Batt.in(exist_Bt) <= 0);


    /*  State Of Charge */
    auto T1 = T.exclude(T.first());/**< Excluding first time step */
    auto Tn = T.exclude(T.last());/**< Excluding last time step */
    Bt1 = indices(T1,B_ph);
    Btn = indices(Tn,B_ph);
    Constraint<> State_Of_Charge("State_Of_Charge");
    State_Of_Charge = Sc.in(Bt1) - Sc.in(Btn) + Pb_.in(Bt1);
    ODO->add(State_Of_Charge.in(Bt1) == 0);

    /*  State Of Charge 0 */
    auto Bat0 = indices(indices(T.first()),B_ph);
    Constraint<> State_Of_Charge0("State_Of_Charge0");
    State_Of_Charge0 = Sc.in(Bat0);
    ODO->add(State_Of_Charge0.in(Bat0) == 0);
    Constraint<> Pb0("Pb0");
    Pb0 = Pb_.in(Bat0);
    ODO->add(Pb0.in(Bat0) == 0);

    /*  EFFICIENCIES */
    Constraint<> DieselEff("DieselEff");
    DieselEff += Pg - gen_eff.in(Gt)*Pg_;
    ODO->add(DieselEff.in(Gt) == 0);

    auto exist_batt_eff = indices(exist_Bt,_eff_pieces);
    auto pot_batt_eff = indices(pot_Bt,_eff_pieces);
    Constraint<> EfficiencyExist("BatteryEfficiencyExisting");
    EfficiencyExist += Pb.in(exist_batt_eff)  - eff_a.in(exist_batt_eff)*Pb_.in(exist_batt_eff) - eff_b.in(exist_batt_eff);
    ODO->add(EfficiencyExist.in(exist_batt_eff) <= 0);

    Constraint<> EfficiencyPot("BatteryEfficiencyPotential");
    EfficiencyPot += Pb.in(pot_batt_eff)  - eff_a.in(pot_batt_eff)*Pb_.in(pot_batt_eff) - eff_b.in(pot_batt_eff)*w_b.in(pot_batt_eff);
    ODO->add(EfficiencyPot.in(pot_batt_eff) <= 0);
//
//
//    for (auto n:nodes) {
//        auto b = (Bus*)n;
//        //        b->print();
//        for (auto i = 0; i < b->_pot_gen.size(); i++) {
//            auto gen = b->_pot_gen[i];
//            if(min_diesel_invest.eval(gen->_name)==max_diesel_invest.eval(gen->_name)){
//                Constraint FixedDieselInvest("FixedDieselInvest"+gen->_name);
//                FixedDieselInvest += w_g(gen->_name);
//                ODO->add(FixedDieselInvest == 1);
//                for (auto j = i+1; j < b->_pot_gen.size(); j++) {
//                    auto gen2 = b->_pot_gen[j];
//                    if (gen2->_gen_type==gen->_gen_type) {
//                        Constraint FixedDieselInvest("FixedDieselInvest"+gen2->_name);
//                        FixedDieselInvest += w_g(gen2->_name);
//                        ODO->add(FixedDieselInvest == 1);
//                    }
//                }
//            }
//            else {
//                Constraint MinDieselInvest("MinDieselInvest_"+b->_name+"_DG"+to_string(gen->_gen_type));
//                MinDieselInvest += w_g(gen->_name);
//                for (auto j = i+1; j < b->_pot_gen.size(); j++) {
//                    auto gen2 = b->_pot_gen[j];
//                    if (gen2->_gen_type==gen->_gen_type) {
//                        MinDieselInvest += w_g(gen2->_name);
//                    }
//                }
//                auto rhs = min_diesel_invest.eval(gen->_name);
//                if (rhs>0) {
//                    ODO->add(MinDieselInvest >= rhs);
//                }
//            }
//        }
//        for (auto i = 0; i < b->_pot_bat.size(); i++) {
//            auto bat = b->_pot_bat[i];
//            if(min_batt_invest.eval(bat->_name)==max_batt_invest.eval(bat->_name)){
//                Constraint FixedBattInvest("FixedBattInvest"+bat->_name);
//                FixedBattInvest += w_b(bat->_name);
//                ODO->add(FixedBattInvest == 1);
//                for (auto j = i+1; j < b->_pot_bat.size(); j++) {
//                    auto bat2 = b->_pot_bat[j];
//                    if (bat2->_bat_type==bat->_bat_type) {
//                        Constraint FixedBattInvest("FixedBattInvest"+bat2->_name);
//                        FixedBattInvest += w_b(bat2->_name);
//                        ODO->add(FixedBattInvest == 1);
//                    }
//                }
//            }
//            else {
//                Constraint MinBattInvest("MinBattInvest_"+b->_name+"_DG"+to_string(bat->_bat_type));
//                MinBattInvest += w_b(bat->_name);
//                for (auto j = i+1; j < b->_pot_bat.size(); j++) {
//                    auto bat2 = b->_pot_bat[j];
//                    if (bat2->_bat_type==bat->_bat_type) {
//                        MinBattInvest += w_b(bat2->_name);
//                    }
//                }
//                auto rhs = min_batt_invest.eval(bat->_name);
//                if (rhs>0) {
//                    ODO->add(MinBattInvest >= rhs);
//                }
//            }
//        }
//    }
//    ODO->print();
    return ODO;
}

int PowerNet::readgrid(const string& fname, bool reverse_arcs) {
    double pi = 4.*atan(1.);
    string name;
    double kvb = 0;
//    int id = 0;
    unsigned index = 0;
    cout << "Loading file " << fname << endl;
    ifstream file(fname.c_str(), std::ifstream::in);
    if(!file.is_open()) {
        throw invalid_argument("Could not open file " + fname);
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
    double total_p_load = 0, total_q_load = 0;
    while(word.compare("];")) {
        name = word.c_str();
        file >> ws >> word;
        status = atoi(word.c_str());
        if (status==3) {
            ref_bus = name;
            DebugOn("Ref bus = " << ref_bus << endl);
        }
        file >> ws >> word;
        pl.add_val(name,atof(word.c_str())/bMVA);
        file >> word;
        ql.add_val(name,atof(word.c_str())/bMVA);
        file >> word;
        gs_.add_val(name,atof(word.c_str())/bMVA);
        file >> word;
        bs_.add_val(name,atof(word.c_str())/bMVA);
        file >> ws >> word >> ws >> word;
        v_s.add_val(name,atof(word.c_str()));
        file >> ws >> word >> ws >> word;
        kvb = atof(word.c_str());
        file >> ws >> word >> ws >> word;
        v_max.add_val(name,atof(word.c_str()));
        getline(file, word,';');
        v_min.add_val(name,atof(word.c_str()));
        w_min.add_val(name,pow(v_min.eval(), 2));
        w_max.add_val(name,pow(v_max.eval(), 2));
        // single phase

        bus = new Bus(name, pl.eval(), ql.eval(), gs_.eval(), bs_.eval(), v_min.eval(), v_max.eval(), kvb, 1);
//        bus_clone = new Bus(name, pl.eval(), ql.eval(), gs.eval(), bs.eval(), v_min.eval(), v_max.eval(), kvb, 1);
        total_p_load += pl.eval();
        total_q_load += ql.eval();
        bus->vs = v_s.eval();
        if (status>=4) {
            bus->_active = false;
//            bus_clone->_active = false;
        }

        this->Net::add_node(bus);
        if (status>=4) {
            DebugOn("INACTIVE NODE!\n" << name << endl);
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
        pg_s.add_val(name,atof(word.c_str())/bMVA);
        file >> word;
        qg_s.add_val(name,atof(word.c_str())/bMVA);
        file >> word;
        qg_max.add_val(name,atof(word.c_str())/bMVA);
        file >> word;
        qg_min.add_val(name,atof(word.c_str())/bMVA);

        file >> ws >> word >> ws >> word >> ws >> word;
        status = atoi(word.c_str());
        file >> word;
        pg_max.add_val(name,atof(word.c_str())/bMVA);

        
        file >> word;
        pg_min.add_val(name,atof(word.c_str())/bMVA);
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
    for (size_t i = 0; i < gens.size(); ++i) {
        file >> ws >> word >> ws >> word >> ws >> word >> ws >> word >> ws >> word;
        c2.add_val(to_string(i),atof(word.c_str())*pow(bMVA,2));
        file >> word;
        c1.add_val(to_string(i),atof(word.c_str())*bMVA);
        file >> word;
        c0.add_val(to_string(i),atof(word.c_str()));
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
    bool reversed = false;
    while(word.compare("];")) {
        src = word;
        file >> dest;
        key = dest+","+src;//Taking care of reversed direction arcs
        reversed = false;
//        if(get_node(src)->_id > get_node(dest)->_id) {//Reverse arc direction
        if((reverse_arcs && get_node(src)->_id > get_node(dest)->_id) || arcID.find(key)!=arcID.end()) {//Reverse arc direction
            Warning("Adding arc linking " +src+" and "+dest);
            Warning(" with reversed direction, reversing source and destination.\n");
            reversed = true;
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
        arc->as = (atof(word.c_str())*pi)/180.;
        file >> ws >> word;
        
        
        
        arc->status = atoi(word.c_str());
        file >> ws >> word;
        
        arc->tbound.min = atof(word.c_str())*pi/180.;
        //        arc->tbound.min = -30*pi/180;
        m_theta_lb += arc->tbound.min;
        file >>  ws >>word;
        
        arc->tbound.max = atof(word.c_str())*pi/180.;
        if (arc->tbound.min==0 && arc->tbound.max==0) {
            DebugOn("Angle bounds are equal to zero. Setting them to -+60");
            arc->tbound.min = -60.*pi/180.;
            arc->tbound.max = 60.*pi/180.;
            
        }
        if (reversed && reverse_arcs) {
            arc->g /= pow(arc->tr,2);
            arc->b /= pow(arc->tr,2);
            arc->ch /= pow(arc->tr,2);
            arc->tr = 1./arc->tr;
            arc->as *= -1.;
            auto temp = arc->tbound.max;
            arc->tbound.max = -1.*arc->tbound.min;
            arc->tbound.min = -1.*temp;
        }
        arc->cc = arc->tr*cos(arc->as); // Rectangular values for transformer phase shifters
        arc->dd = arc->tr*sin(arc->as);
        //        arc->tbound.max = 30*pi/180;
        m_theta_ub += arc->tbound.max;
        
        Bus* bus_s = (Bus*)(arc->_src);
        Bus* bus_d = (Bus*)(arc->_dest);
        
        arc->smax = gravity::max(
                        pow(bus_s->vbound.max,2)*(arc->g*arc->g + arc->b*arc->b)*(pow(bus_s->vbound.max,2) + pow(bus_d->vbound.max,2)),
                        pow(bus_d->vbound.max,2)*(arc->g*arc->g+arc->b*arc->b)*(pow(bus_d->vbound.max,2) + pow(bus_s->vbound.max,2))
                        );
        name = arc->_name;
        g.add_val(name,arc->g);
        b.add_val(name,arc->b);
        tr.add_val(name,arc->tr);
        as.add_val(name,arc->as);
        //(g+g_fr)/tm^2
        g_ff.add_val(name,arc->g/(pow(arc->cc, 2) + pow(arc->dd, 2)));
        g_ft.add_val(name,(-arc->g*arc->cc + arc->b*arc->dd)/(pow(arc->cc, 2) + pow(arc->dd, 2)));
        
        g_tt.add_val(name,arc->g);
        g_tf.add_val(name,(-arc->g*arc->cc - arc->b*arc->dd)/(pow(arc->cc, 2) + pow(arc->dd, 2)));
        
        
        b_ff.add_val(name,(arc->ch*0.5 + arc->b)/(pow(arc->cc, 2) + pow(arc->dd, 2)));
        b_ft.add_val(name,(-arc->b*arc->cc - arc->g*arc->dd)/(pow(arc->cc, 2) + pow(arc->dd, 2)));
        
        b_tt.add_val(name,(arc->ch*0.5 + arc->b));
        b_tf.add_val(name,(-arc->b*arc->cc + arc->g*arc->dd)/(pow(arc->cc, 2) + pow(arc->dd, 2)));
        
        ch.add_val(name,arc->ch);
//        S_max.add_val(name,gravity::min(arc->limit,max(2.*total_p_load, 2.*total_q_load)));
        S_max.add_val(name,arc->limit);
        
        
        if(arc->status != 1 || !bus_s->_active || !bus_d->_active) {
            arc->_active = false;
            DebugOn("INACTIVE ARC!\n" << arc->_name << endl);
        }
        arc->connect();
        add_arc(arc);
        /* Switching to bus_pairs keys */
        name = bus_s->_name + "," + bus_d->_name;
        if (arc->_active && bus_pair_names.count(name)==0) {
//            _bus_pairs._keys.push_back(new index_pair(index_(bus_s->_name), index_(bus_d->_name), arc->_active));
            bus_pair_names.insert(name);
        }
        if (!arc->_parallel) {
            th_min.add_val(name,arc->tbound.min);
            th_max.add_val(name,arc->tbound.max);
            tan_th_min.add_val(name,tan(arc->tbound.min));
            tan_th_max.add_val(name,tan(arc->tbound.max));
            
        }
        else {
            th_min.add_val(name,gravity::max(th_min.eval(name), arc->tbound.min));
            th_max.add_val(name,gravity::min(th_max.eval(name), arc->tbound.max));
            tan_th_min.add_val(name,tan(th_min.eval(name)));
            tan_th_max.add_val(name,tan(th_max.eval(name)));
        }
        if (arc->tbound.min >= 0) {
            wr_max.add_val(name,bus_s->vbound.max*bus_d->vbound.max*cos(th_min.eval(name)));
            wr_min.add_val(name,bus_s->vbound.min*bus_d->vbound.min*cos(th_max.eval(name)));
            wi_max.add_val(name,bus_s->vbound.max*bus_d->vbound.max*sin(th_max.eval(name)));
            wi_min.add_val(name,bus_s->vbound.min*bus_d->vbound.min*sin(th_min.eval(name)));
        };
        if (arc->tbound.max <= 0) {
            wr_max.add_val(name,bus_s->vbound.max*bus_d->vbound.max*cos(th_max.eval(name)));
            wr_min.add_val(name,bus_s->vbound.min*bus_d->vbound.min*cos(th_min.eval(name)));
            wi_max.add_val(name,bus_s->vbound.min*bus_d->vbound.min*sin(th_max.eval(name)));
            wi_min.add_val(name,bus_s->vbound.max*bus_d->vbound.max*sin(th_min.eval(name)));
        }
        if (arc->tbound.min < 0 && arc->tbound.max > 0) {
            wr_max.add_val(name,bus_s->vbound.max*bus_d->vbound.max);
            wr_min.add_val(name,bus_s->vbound.min*bus_d->vbound.min*gravity::min(cos(th_min.eval(name)), cos(th_max.eval(name))));
            wi_max.add_val(name,bus_s->vbound.max*bus_d->vbound.max*sin(th_max.eval(name)));
            wi_min.add_val(name,bus_s->vbound.max*bus_d->vbound.max*sin(th_min.eval(name)));
        }
        cphi.add_val(name, cos(0.5*(arc->tbound.min+arc->tbound.max)));
        sphi.add_val(name, sin(0.5*(arc->tbound.min+arc->tbound.max)));
        cos_d.add_val(name, cos(0.5*(arc->tbound.max-arc->tbound.min)));
        getline(file, word,'\n');
        file >> word;
    }
    DebugOff(ch.to_str(true) << endl);
    DebugOff(as.to_str(true) << endl);
    DebugOff(tr.to_str(true) << endl);
    
    file.close();
//    if (nodes.size()>1000) {
//        add_3d_nlin = false;
//    }
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
            if(b.size() >= 2) bags_sorted.push_back(b);
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
                cos_min_ = gravity::min(cos(m_theta_lb), cos(m_theta_ub));
            } else{
                cos_max_ = gravity::max(cos(m_theta_lb),cos(m_theta_ub));
                cos_min_ = gravity::min(cos(m_theta_lb), cos(m_theta_ub));
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

            wr_max.add_val(name,wr_max_);
            wr_min.add_val(name,wr_min_);
            wi_max.add_val(name,wi_max_);
            wi_min.add_val(name,wi_min_);
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



shared_ptr<Model<>> PowerNet::build_SCOPF(PowerModelType pmt, int output, double tol){
    auto bus_pairs = get_bus_pairs();
    /** MODEL DECLARATION */
    shared_ptr<Model<>> SOCPF(new Model<>("SCOPF Model"));
    /** Variables */
    /* power generation variables */
    var<double> Pg("Pg", pg_min, pg_max);
    var<double> Qg ("Qg", qg_min, qg_max);
    SOCPF->add(Pg.in(gens));
    SOCPF->add(Qg.in(gens));


    /* power flow variables */
    var<double> Pf_from("Pf_from", -1*S_max,S_max);
    var<double> Qf_from("Qf_from", -1*S_max,S_max);
    var<double> Pf_to("Pf_to", -1*S_max,S_max);
    var<double> Qf_to("Qf_to", -1*S_max,S_max);
    SOCPF->add(Pf_from.in(arcs));
    SOCPF->add(Qf_from.in(arcs));
    SOCPF->add(Pf_to.in(arcs));
    SOCPF->add(Qf_to.in(arcs));

    /* Real part of Wij = ViVj */
    var<double>  R_Wij("R_Wij", wr_min, wr_max);
    /* Imaginary part of Wij = ViVj */
    var<double>  Im_Wij("Im_Wij", wi_min, wi_max);
    /* Magnitude of Wii = Vi^2 */
    var<double>  Wii("Wii", w_min, w_max);
    SOCPF->add(Wii.in(nodes));
    SOCPF->add(R_Wij.in(bus_pairs));
    SOCPF->add(Im_Wij.in(bus_pairs));

    /* Initialize variables */
    R_Wij.initialize_all(1.0);
    Wii.initialize_all(1.001);
    
    /** Sets */
    auto gens = gens_per_node();
    auto out_arcs = out_arcs_per_node();
    auto in_arcs = in_arcs_per_node();

    /**  Objective */
    auto obj = c1.tr()*Pg + c2.tr()*pow(Pg,2) + sum(c0);
    SOCPF->min(obj);

    /** Constraints */
    /* Second-order cone constraints */
    Constraint<> SOC("SOC");
    SOC = pow(R_Wij, 2) + pow(Im_Wij, 2) - Wii.from(bus_pairs)*Wii.to(bus_pairs);
    SOCPF->add(SOC.in(bus_pairs) <= 0);

    /* Flow conservation */
    Constraint<> KCL_P("KCL_P");
    KCL_P  = sum(Pf_from, out_arcs) + sum(Pf_to, in_arcs) + pl - sum(Pg, gens) + gs_*Wii;
    SOCPF->add(KCL_P.in(nodes) == 0);

    Constraint<> KCL_Q("KCL_Q");
    KCL_Q  = sum(Qf_from, out_arcs) + sum(Qf_to, in_arcs) + ql - sum(Qg, gens) - bs_*Wii;
    SOCPF->add(KCL_Q.in(nodes) == 0);

    /* AC Power Flow */
    Constraint<> Flow_P_From("Flow_P_From");
    Flow_P_From = Pf_from - (g_ff*Wii.from(arcs) + g_ft*R_Wij + b_ft*Im_Wij);
    SOCPF->add(Flow_P_From.in(arcs) == 0);

    Constraint<> Flow_P_To("Flow_P_To");
    Flow_P_To = Pf_to - (g_tt*Wii.to(arcs) + g_tf*R_Wij - b_tf*Im_Wij);
    SOCPF->add(Flow_P_To.in(arcs) == 0);

    Constraint<> Flow_Q_From("Flow_Q_From");
    Flow_Q_From = Qf_from - (g_ft*Im_Wij - b_ff*Wii.from(arcs) - b_ft*R_Wij);
    SOCPF->add(Flow_Q_From.in(arcs) == 0);

    Constraint<> Flow_Q_To("Flow_Q_To");
    Flow_Q_To = Qf_to + (b_tt*Wii.to(arcs) + b_tf*R_Wij + g_tf*Im_Wij);
    SOCPF->add(Flow_Q_To.in(arcs) == 0);

    /* Phase Angle Bounds constraints */
    Constraint<> PAD_UB("PAD_UB");
    PAD_UB = Im_Wij;
    PAD_UB <= tan_th_max*R_Wij;
    SOCPF->add(PAD_UB);

    Constraint<> PAD_LB("PAD_LB");
    PAD_LB =  Im_Wij;
    PAD_LB >= tan_th_min*R_Wij;
    SOCPF->add(PAD_LB);

    /* Thermal Limit Constraints */
    Constraint<> Thermal_Limit_from("Thermal_Limit_from");
    Thermal_Limit_from = pow(Pf_from, 2) + pow(Qf_from, 2);
    Thermal_Limit_from <= pow(S_max,2);
    SOCPF->add(Thermal_Limit_from);


    Constraint<> Thermal_Limit_to("Thermal_Limit_to");
    Thermal_Limit_to = pow(Pf_to, 2) + pow(Qf_to, 2);
    Thermal_Limit_to <= pow(S_max,2);
    SOCPF->add(Thermal_Limit_to);

    /* Lifted Nonlinear Cuts */
    Constraint<> LNC1("LNC1");
    LNC1 += (v_min.from(bus_pairs)+v_max.from(bus_pairs))*(v_min.to(bus_pairs)+v_max.to(bus_pairs))*(sphi*Im_Wij + cphi*R_Wij);
    LNC1 -= v_max.to(bus_pairs)*cos_d*(v_min.to(bus_pairs)+v_max.to(bus_pairs))*Wii.from(bus_pairs);
    LNC1 -= v_max.from(bus_pairs)*cos_d*(v_min.from(bus_pairs)+v_max.from(bus_pairs))*Wii.to(bus_pairs);
    LNC1 -= v_max.from(bus_pairs)*v_max.to(bus_pairs)*cos_d*(v_min.from(bus_pairs)*v_min.to(bus_pairs) - v_max.from(bus_pairs)*v_max.to(bus_pairs));
    SOCPF->add(LNC1.in(bus_pairs) >= 0);

    Constraint<> LNC2("LNC2");
    LNC2 += (v_min.from(bus_pairs)+v_max.from(bus_pairs))*(v_min.to(bus_pairs)+v_max.to(bus_pairs))*(sphi*Im_Wij + cphi*R_Wij);
    LNC2 -= v_min.to(bus_pairs)*cos_d*(v_min.to(bus_pairs)+v_max.to(bus_pairs))*Wii.from(bus_pairs);
    LNC2 -= v_min.from(bus_pairs)*cos_d*(v_min.from(bus_pairs)+v_max.from(bus_pairs))*Wii.to(bus_pairs);
    LNC2 += v_min.from(bus_pairs)*v_min.to(bus_pairs)*cos_d*(v_min.from(bus_pairs)*v_min.to(bus_pairs) - v_max.from(bus_pairs)*v_max.to(bus_pairs));
    SOCPF->add(LNC2.in(bus_pairs) >= 0);
    return SOCPF;
}



shared_ptr<Model<>> build_ACOPF(PowerNet& grid, PowerModelType pmt, int output, double tol){
    /** Sets */
    auto bus_pairs = grid.get_bus_pairs();
    auto nodes = indices(grid.nodes);
    auto arcs = indices(grid.arcs);
    auto gens = indices(grid.gens);
    auto gen_nodes = grid.gens_per_node();
    auto out_arcs = grid.out_arcs_per_node();
    auto in_arcs = grid.in_arcs_per_node();
    
    /* Grid Parameters */
    auto pg_min = grid.pg_min.in(gens);
    auto pg_max = grid.pg_max.in(gens);
    auto qg_min = grid.qg_min.in(gens);
    auto qg_max = grid.qg_max.in(gens);
    auto c1 = grid.c1.in(gens);
    auto c2 = grid.c2.in(gens);
    auto c0 = grid.c0.in(gens);
    auto pl = grid.pl.in(nodes);
    auto ql = grid.ql.in(nodes);
    auto gs = grid.gs_.in(nodes);
    auto bs = grid.bs_.in(nodes);
    auto b = grid.b.in(arcs);
    auto g = grid.g.in(arcs);
    auto as = grid.as.in(arcs);
    auto ch = grid.ch.in(arcs);
    auto tr = grid.tr.in(arcs);
    auto th_min = grid.th_min.in(bus_pairs);
    auto th_max = grid.th_max.in(bus_pairs);
    auto g_ft = grid.g_ft.in(arcs);
    auto g_ff = grid.g_ff.in(arcs);
    auto g_tt = grid.g_tt.in(arcs);
    auto g_tf = grid.g_tf.in(arcs);
    auto b_ft = grid.b_ft.in(arcs);
    auto b_ff = grid.b_ff.in(arcs);
    auto b_tf = grid.b_tf.in(arcs);
    auto b_tt = grid.b_tt.in(arcs);
    auto S_max = grid.S_max.in(arcs);
    auto v_max = grid.v_max.in(nodes);
    auto v_min = grid.v_min.in(nodes);
    auto tan_th_min = grid.tan_th_min.in(bus_pairs);
    auto tan_th_max = grid.tan_th_max.in(bus_pairs);
    
    bool polar = (pmt==ACPOL);
    if (polar) {
        DebugOn("Using polar model\n");
    }
    else {
        DebugOn("Using rectangular model\n");
    }
    auto ACOPF = make_shared<Model<>>("AC-OPF Model");
    /** Variables */
    /* Power generation variables */
    var<> Pg("Pg", pg_min, pg_max);
    var<> Qg ("Qg", qg_min, qg_max);
    ACOPF->add(Pg.in(gens),Qg.in(gens));
    //    Pg.copy_vals(grid.pg_s);
    //    Pg.initialize_av();
    //    Qg.initialize_uniform();
    /* Power flow variables */
    var<> Pf_from("Pf_from", -1.*S_max,S_max);
    var<> Qf_from("Qf_from", -1.*S_max,S_max);
    var<> Pf_to("Pf_to", -1.*S_max,S_max);
    var<> Qf_to("Qf_to", -1.*S_max,S_max);
    ACOPF->add(Pf_from.in(arcs), Qf_from.in(arcs),Pf_to.in(arcs),Qf_to.in(arcs));
    
    /** Voltage related variables */
    var<> theta("theta");
    var<> v("|V|", v_min, v_max);
    var<> vr("vr", -1.*v_max,v_max);
    var<> vi("vi", -1.*v_max,v_max);
    
    var<> v_from, v_to, theta_from, theta_to;
    var<> vr_from, vr_to, vi_from, vi_to;
    if (polar) {
        ACOPF->add(v.in(nodes));
        ACOPF->add(theta.in(nodes));
        v.initialize_all(1.0);
        v_from = v.from(arcs);
        v_to = v.to(arcs);
        theta_from = theta.from(arcs);
        theta_to = theta.to(arcs);
        
    }
    else {
        ACOPF->add(vr.in(nodes));
        ACOPF->add(vi.in(nodes));
        vr.initialize_all(1);
        vr_from = vr.from(arcs);
        vr_to = vr.to(arcs);
        vi_from = vi.from(arcs);
        vi_to = vi.to(arcs);
        //        vr.initialize_uniform(0.99,1.01);
    }
    
    /** Construct the objective function */
    /**  Objective */
    auto obj = product(c1,Pg) + product(c2,pow(Pg,2)) + sum(c0);
    ACOPF->min(obj);
    
    /** Define constraints */
    
    /* REF BUS */
    Constraint<> Ref_Bus("Ref_Bus");
    if (polar) {
        Ref_Bus = theta(grid.ref_bus);
    }
    else {
        Ref_Bus = vi(grid.ref_bus);
    }
    ACOPF->add(Ref_Bus == 0);
    
    /** KCL Flow conservation */
    Constraint<> KCL_P("KCL_P");
    Constraint<> KCL_Q("KCL_Q");
    KCL_P  = sum(Pf_from, out_arcs) + sum(Pf_to, in_arcs) + pl - sum(Pg, gen_nodes);
    KCL_Q  = sum(Qf_from, out_arcs) + sum(Qf_to, in_arcs) + ql - sum(Qg, gen_nodes);
    /* Shunts */
    if (polar) {
        KCL_P +=  gs*pow(v,2);
        KCL_Q -=  bs*pow(v,2);
    }
    else {
        KCL_P +=  gs*(pow(vr,2)+pow(vi,2));
        KCL_Q -=  bs*(pow(vr,2)+pow(vi,2));
    }
    ACOPF->add(KCL_P.in(nodes) == 0);
    ACOPF->add(KCL_Q.in(nodes) == 0);
    
    /** AC Power Flows */
    /** TODO write the constraints in Complex form */
    Constraint<> Flow_P_From("Flow_P_From");
    Flow_P_From += Pf_from;
    if (polar) {
        Flow_P_From -= g/pow(tr,2)*pow(v_from,2);
        Flow_P_From += g/tr*(v_from*v_to*cos(theta_from - theta_to - as));
        Flow_P_From += b/tr*(v_from*v_to*sin(theta_from - theta_to - as));
    }
    else {
        Flow_P_From -= g_ff*(pow(vr_from, 2) + pow(vi_from, 2));
        Flow_P_From -= g_ft*(vr_from*vr_to + vi_from*vi_to);
        Flow_P_From -= b_ft*(vi_from*vr_to - vr_from*vi_to);
    }
    ACOPF->add(Flow_P_From.in(arcs)==0);
    
    Constraint<> Flow_P_To("Flow_P_To");
    Flow_P_To += Pf_to;
    if (polar) {
        Flow_P_To -= g*pow(v_to, 2);
        Flow_P_To += g/tr*(v_from*v_to*cos(theta_to - theta_from + as));
        Flow_P_To += b/tr*(v_from*v_to*sin(theta_to - theta_from + as));
    }
    else {
        Flow_P_To -= g_tt*(pow(vr_to, 2) + pow(vi_to, 2));
        Flow_P_To -= g_tf*(vr_from*vr_to + vi_from*vi_to);
        Flow_P_To -= b_tf*(vi_to*vr_from - vr_to*vi_from);
    }
    ACOPF->add(Flow_P_To.in(arcs)==0);
    
    Constraint<> Flow_Q_From("Flow_Q_From");
    Flow_Q_From += Qf_from;
    if (polar) {
        Flow_Q_From += (0.5*ch+b)/pow(tr,2)*pow(v_from,2);
        Flow_Q_From -= b/tr*(v_from*v_to*cos(theta_from - theta_to - as));
        Flow_Q_From += g/tr*(v_from*v_to*sin(theta_from - theta_to - as));
    }
    else {
        Flow_Q_From += b_ff*(pow(vr_from, 2) + pow(vi_from, 2));
        Flow_Q_From += b_ft*(vr_from*vr_to + vi_from*vi_to);
        Flow_Q_From -= g_ft*(vi_from*vr_to - vr_from*vi_to);
    }
    ACOPF->add(Flow_Q_From.in(arcs)==0);
    
    Constraint<> Flow_Q_To("Flow_Q_To");
    Flow_Q_To += Qf_to;
    if (polar) {
        Flow_Q_To += (0.5*ch+b)*pow(v_to,2);
        Flow_Q_To -= b/tr*(v_from*v_to*cos(theta_to - theta_from + as));
        Flow_Q_To += g/tr*(v_from*v_to*sin(theta_to - theta_from + as));
    }
    else {
        Flow_Q_To += b_tt*(pow(vr_to, 2) + pow(vi_to, 2));
        Flow_Q_To += b_tf*(vr_from*vr_to + vi_from*vi_to);
        Flow_Q_To -= g_tf*(vi_to*vr_from - vr_to*vi_from);
    }
    ACOPF->add(Flow_Q_To.in(arcs)==0);
    
    /** AC voltage limit constraints. */
    if (!polar) {
        Constraint<> Vol_limit_UB("Vol_limit_UB");
        Vol_limit_UB = pow(vr, 2) + pow(vi, 2);
        Vol_limit_UB -= pow(v_max, 2);
        ACOPF->add(Vol_limit_UB.in(nodes) <= 0);
        
        Constraint<> Vol_limit_LB("Vol_limit_LB");
        Vol_limit_LB = pow(vr, 2) + pow(vi, 2);
        Vol_limit_LB -= pow(v_min,2);
        ACOPF->add(Vol_limit_LB.in(nodes) >= 0);
    }
    
    
    /* Phase Angle Bounds constraints */
    Constraint<> PAD_UB("PAD_UB");
    Constraint<> PAD_LB("PAD_LB");
    if (polar) {
        PAD_UB = theta.from(bus_pairs) - theta.to(bus_pairs);
        PAD_UB -= th_max;
        PAD_LB = theta.from(bus_pairs) - theta.to(bus_pairs);
        PAD_LB -= th_min;
    }
    else {
        DebugOff("Number of bus_pairs = " << bus_pairs.size() << endl);
        PAD_UB = vi.from(bus_pairs)*vr.to(bus_pairs) - vr.from(bus_pairs)*vi.to(bus_pairs);
        PAD_UB -= tan_th_max*(vr.from(bus_pairs)*vr.to(bus_pairs) + vi.from(bus_pairs)*vi.to(bus_pairs));
        
        PAD_LB = vi.from(bus_pairs)*vr.to(bus_pairs) - vr.from(bus_pairs)*vi.to(bus_pairs);
        PAD_LB -= tan_th_min*(vr.from(bus_pairs)*vr.to(bus_pairs) + vi.from(bus_pairs)*vi.to(bus_pairs));
    }
    ACOPF->add(PAD_UB.in(bus_pairs) <= 0);
    ACOPF->add(PAD_LB.in(bus_pairs) >= 0);
    
    
    /*  Thermal Limit Constraints */
    Constraint<> Thermal_Limit_from("Thermal_Limit_from");
    Thermal_Limit_from += pow(Pf_from, 2) + pow(Qf_from, 2);
    Thermal_Limit_from -= pow(S_max, 2);
    ACOPF->add(Thermal_Limit_from.in(arcs) <= 0);
    
    Constraint<> Thermal_Limit_to("Thermal_Limit_to");
    Thermal_Limit_to += pow(Pf_to, 2) + pow(Qf_to, 2);
    Thermal_Limit_to -= pow(S_max,2);
    ACOPF->add(Thermal_Limit_to.in(arcs) <= 0);
    return ACOPF;
}



/** Return the vector of arcs of the chordal completion ignoring parallel lines **/
indices PowerNet::get_bus_pairs_chord(){
    set<pair<Node*,Node*>> unique_pairs;
    indices bpairs("bus_pairs");
    for (auto a: arcs) {
        if (!a->_parallel) {
            unique_pairs.insert({a->_src,a->_dest});
            bpairs.insert(a->_src->_name+","+a->_dest->_name);
        }
    }
    string key;
    double cos_max_, cos_min_, sin_max_, sin_min_;
    double wr_max_, wr_min_, wi_max_, wi_min_, w_max_, w_min_;
    if (m_theta_lb < -3.14 && m_theta_ub > 3.14) {
        cos_max_ = 1;
        cos_min_ = -1;
    } else if (m_theta_lb < 0 && m_theta_ub > 0){
        cos_max_ = 1;
        cos_min_ = gravity::min(cos(m_theta_lb), cos(m_theta_ub));
    } else{
        cos_max_ = gravity::max(cos(m_theta_lb),cos(m_theta_ub));
        cos_min_ = gravity::min(cos(m_theta_lb), cos(m_theta_ub));
    }
    if(m_theta_lb < -1.57 && m_theta_ub > 1.57){
        sin_max_ = 1;
        sin_min_ = -1;
    } else{
        sin_max_ = sin(m_theta_ub);
        sin_min_ = sin(m_theta_lb);
    }
    for (auto &bag: _bags) {
        for (size_t i = 0; i< bag.size()-1; i++) {
            if (unique_pairs.insert({bag[i],bag[i+1]}).second) {
                auto bus_s = (Bus*)bag[i];
                auto bus_d = (Bus*)bag[i+1];
                w_max_ = bus_s->vbound.max*bus_d->vbound.max;
                w_min_ = bus_s->vbound.min*bus_d->vbound.min;
                wr_max_ = cos_max_*w_max_;
                if(cos_min_ < 0) wr_min_ = cos_min_*w_max_;
                else wr_min_ = cos_min_*w_min_;
                if(sin_max_ > 0) wi_max_ = sin_max_*w_max_;
                else wi_max_ = sin_max_*w_min_;
                if(sin_min_ > 0) wi_min_ = sin_min_*w_min_;
                else wi_min_ = sin_min_*w_max_;
                auto name = bag[i]->_name + "," + bag[i+1]->_name;
                wr_max.add_val(name,wr_max_);
                wr_min.add_val(name,wr_min_);
                wi_max.add_val(name,wi_max_);
                wi_min.add_val(name,wi_min_);
                bpairs.insert(name);
            }
        }
        /* Loop back pair */
        if (unique_pairs.insert({bag[0],bag[bag.size()-1]}).second) {
            auto name = bag[0]->_name + "," + bag[bag.size()-1]->_name;
            auto bus_s = (Bus*)bag[0];
            auto bus_d = (Bus*)bag[bag.size()-1];
            w_max_ = bus_s->vbound.max*bus_d->vbound.max;
            w_min_ = bus_s->vbound.min*bus_d->vbound.min;
            wr_max_ = cos_max_*w_max_;
            if(cos_min_ < 0) wr_min_ = cos_min_*w_max_;
            else wr_min_ = cos_min_*w_min_;
            if(sin_max_ > 0) wi_max_ = sin_max_*w_max_;
            else wi_max_ = sin_max_*w_min_;
            if(sin_min_ > 0) wi_min_ = sin_min_*w_min_;
            else wi_min_ = sin_min_*w_max_;
            wr_max.add_val(name,wr_max_);
            wr_min.add_val(name,wr_min_);
            wi_max.add_val(name,wi_max_);
            wi_min.add_val(name,wi_min_);
            bpairs.insert(name);
        }
    }
    return bpairs;
}

double PowerNet::solve_acopf(PowerModelType pmt, int output, double tol){
    
    auto ACOPF = build_ACOPF(*this,pmt,output,tol);
    bool relax;
    solver<> OPF(ACOPF,ipopt);
//    auto mipgap = 1e-6;
    OPF.run(output, tol);
    return ACOPF->_obj->get_val();
}


void PowerNet::fill_wbnds(){
    double cos_max_, cos_min_, w_max_, w_min_, wr_max_, wr_min_, sin_max_, sin_min_, wi_max_, wi_min_;
    for(int i = 0; i < nodes.size()-1; i++) {
        for(int j = i+1; j < nodes.size(); j++) {
            Bus *bus_s = (Bus *) (nodes[i]);
            Bus *bus_d = (Bus *) (nodes[j]);
            
            if(get_arc(bus_s, bus_d)) continue;
            
            string name = bus_s->_name + "," + bus_d->_name;
//            _bus_pairs_chord._keys.push_back(new index_pair(index_(bus_s->_name), index_(bus_d->_name)));
            
            if (m_theta_lb < -3.14 && m_theta_ub > 3.14) {
                cos_max_ = 1;
                cos_min_ = -1;
            } else if (m_theta_lb < 0 && m_theta_ub > 0) {
                cos_max_ = 1;
                cos_min_ = gravity::min(cos(m_theta_lb), cos(m_theta_ub));
            } else {
                cos_max_ = gravity::max(cos(m_theta_lb), cos(m_theta_ub));
                cos_min_ = gravity::min(cos(m_theta_lb), cos(m_theta_ub));
            }
            w_max_ = bus_s->vbound.max * bus_d->vbound.max;
            w_min_ = bus_s->vbound.min * bus_d->vbound.min;
            
            wr_max_ = cos_max_ * w_max_;
            if (cos_min_ < 0) wr_min_ = cos_min_ * w_max_;
            else wr_min_ = cos_min_ * w_min_;
            
            if (m_theta_lb < -1.57 && m_theta_ub > 1.57) {
                sin_max_ = 1;
                sin_min_ = -1;
            } else {
                sin_max_ = sin(m_theta_ub);
                sin_min_ = sin(m_theta_lb);
            }
            
            if (sin_max_ > 0) wi_max_ = sin_max_ * w_max_;
            else wi_max_ = sin_max_ * w_min_;
            if (sin_min_ > 0) wi_min_ = sin_min_ * w_min_;
            else wi_min_ = sin_min_ * w_max_;
            
            //            cout << "\nImaginary line, bounds: (" << wr_min_ << "," << wr_max_ << "); (" << wi_min_ << "," << wi_max_ << ")";
            
            wr_max.add_val(name, wr_max_);
            wr_min.add_val(name, wr_min_);
            wi_max.add_val(name, wi_max_);
            wi_min.add_val(name, wi_min_);
        }
    }
}



