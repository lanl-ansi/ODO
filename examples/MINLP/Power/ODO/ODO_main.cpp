//
//  OD&O Implementation in Gravity
//
//  Created by Hassan Hijazi Dec 2018
//
//
#include <stdio.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <cstring>
#include <fstream>
#include "../PowerNet.h"
#include <gravity/solver.h>
#include <stdio.h>
#include <stdlib.h>
#include <optionParser.hpp>

using namespace std;
using namespace gravity;


int main (int argc, char * argv[])
{
    string fname = string(prj_dir)+"/data_sets/Power/ODO_INPUT.xlsx", mtype = "ACPOL";
    string solver_str="ipopt";
    int output = 0;
    bool relax = false, use_cplex = false, use_gurobi = false;
    double tol = 1e-6;
    string mehrotra = "no", log_level="0", nb_hours="1";
    
    /** create a OptionParser with options */
    op::OptionParser opt;
    opt.add_option("h", "help", "shows option help"); // no default value means boolean options, which default value is false
    opt.add_option("f", "file", "Input file name (def. ../data_sets/Power/nesta_case5_pjm.m)", fname );
    opt.add_option("l", "log", "Log level (def. 0)", log_level );
    opt.add_option("t", "time", "time in hours (def. 1)", nb_hours );
    opt.add_option("m", "model", "power flow model: ACPOL/ACRECT/DISTFLOW/LINDISTFLOW (def. ACPOL)", mtype );
    opt.add_option("s", "solver", "Solvers: ipopt/cplex/gurobi, default = ipopt", solver_str);
    
    /** parse the options and verify that all went well. If not, errors and help will be shown */
    bool correct_parsing = opt.parse_options(argc, argv);
    
    if(!correct_parsing){
        return EXIT_FAILURE;
    }
    
    fname = opt["f"];
    mtype = opt["m"];
    solver_str = opt["s"];
    if (solver_str.compare("gurobi")==0) {
        use_gurobi = true;
    }
    else if(solver_str.compare("cplex")==0) {
        use_cplex = true;
    }
    output = op::str2int(opt["l"]);
    auto max_nb_hours = op::str2int(opt["t"]);
    bool has_help = op::str2bool(opt["h"]);
    /** show help */
    if(has_help) {
        opt.show_help();
        exit(0);
    }
    double solver_time_end, total_time_end, solve_time, cont_solve_start, cont_solve_end, cont_solve_time, model_build_time = -1, total_time = -1;
    double total_time_start = get_wall_time();
    PowerNet grid;
    PowerModelType pmt = LDISTF;
    if(mtype.compare("DISTF")==0) pmt = DISTF;
    else if(mtype.compare("CDISTF")==0) pmt = CDISTF;
    if (pmt==LDISTF) {
        DebugOn("Using Linear Distflow model\n");
    }
    else if(pmt==DISTF) {
        DebugOn("Using Nonconvex Distflow model\n");
    }
    else if(pmt==CDISTF) {
        DebugOn("Using Convex Distflow model\n");
    }
    
    grid.readJSON(string(prj_dir)+"/data_sets/Power/IEEE13.json");
    auto stat = grid.readODO(fname);
    if (stat==-1) {
        cerr << "Error reading Excel File, Exising" << endl;
        return -1;
    }
    
    /* Grid Parameters */
    auto bus_pairs = grid.get_bus_pairs();
    auto Ngen = grid.gens.size();
    auto Nbat = grid._battery_inverters.size();
    auto Nline = grid.get_nb_active_arcs();
    auto Nbus = grid.get_nb_active_nodes();
    DebugOn("nb generators = " << Ngen << endl);
    DebugOn("nb of potential generators = " << grid._potential_diesel_gens.size() << endl);
    DebugOn("nb batteries = " << Nbat << endl);
    DebugOn("nb of potential batteries = " << grid._potential_battery_inverters.size() << endl);
    DebugOn("nb of existing PV generators = " << grid._existing_PV_gens.size() << endl);
    DebugOn("nb of potential PV generators = " << grid._potential_PV_gens.size() << endl);
    DebugOn("nb of existing Wind generators = " << grid._existing_wind_gens.size() << endl);
    DebugOn("nb of potential Wind generators = " << grid._potential_wind_gens.size()<< endl);
    DebugOn("nb installed lines = " << Nline << endl);
    DebugOn("number of buses = " << Nbus << endl);

    auto ROMDST = grid.build_ROMDST_3phase(pmt,output,tol,max_nb_hours);
//    return 0;
    
    
        ROMDST->print_expanded();
    //    return 0;
    
    /* SOLVE */
    double lb = 0, ub = 0;
    int return_status = -1;
    auto solver_time_start = get_wall_time();
    if (use_cplex) {
        solver ACUC(*ROMDST,cplex);
        return_status = ACUC.run(output, relax = true, tol = 1e-6, 0.01);
        if (return_status==100) {
            lb = ROMDST->_obj_val;
            ub = ROMDST->_obj_val;
        }
    }
    else if (use_gurobi) {
        solver ACUC(*ROMDST,gurobi);
        return_status = ACUC.run(output, relax = false, tol = 1e-6, 0.01);
        if (return_status==100) {
            lb = ROMDST->_obj_val;
            ub = ROMDST->_obj_val;
        }
    }
    else {
        solver ACUC(*ROMDST,ipopt);
        return_status = ACUC.run(output=5, relax = false, tol = 1e-6, 1e-6, "mumps");
        ROMDST->print_solution(false);
        return 0;
        if (return_status != 100) {
            clog << "Cannot compute lower bound!\n";
        }
        else {
            lb = ROMDST->_obj_val;
            DebugOn("Lower Bound = " << lb << endl);
            ROMDST->print_solution();
//            DebugOn("Rounding current solution..." << endl);
//            ROMDST->round_solution();
//            if(ROMDST->is_feasible(tol=1e-5)){
            if(false){
                clog << "Feasible integer solution found!\n";
                ub = lb;
                return_status = 100;
            }
            else{
                clog << "Rounded integer solution is not feasible!\n Trying to enforce integrality in the objective\n";
                auto old_obj = ROMDST->_obj;
                ROMDST->add_round_solution_obj();
                //                ROMDST->add_integrality();
                //                ROMDST->add_round_solution_cuts();
//                ROMDST->_first_run = true;
//                solver ACUC_CPX(*ROMDST,ipopt);
//                return_status = ACUC_CPX.run(output, relax = false, tol = 1e-6);
                return_status = ACUC.run(output, relax = false, tol = 1e-6);
                if (return_status != 100) {
                    clog << "Cannot compute lower bound!\n";
                }
                else {
                    //                ROMDST->print_solution();
                    ROMDST->round_solution();
                    if(ROMDST->is_feasible(tol=1e-4)){
                        clog << "Feasible integer solution found!\n";
                        ub = old_obj.eval();
                    }
                    else{
                        clog << "Current integer solution is infeasible!\n Trying to solve with pure integer objective\n";
                        ROMDST->add_round_solution_obj(false);
                        //                        ROMDST->_first_run = true;
                        //                        solver ACUC_CPX(*ROMDST,cplex);
                        return_status = ACUC.run(output, relax = false, tol = 1e-6);
                        if (return_status != 100) {
                            clog << "Cannot compute lower bound!\n";
                        }
                        else {
                            ROMDST->round_solution();
                            if(ROMDST->is_feasible(tol=1e-4)){
                                clog << "Feasible integer solution found!\n";
                                ub = old_obj.eval();
                            }
                            else{
                                return_status = -1;
                                clog << "Current integer solution is infeasible!\n";
                            }
                        }
                    }
                }
            }
        }
    }
    solver_time_end = get_wall_time();
    solve_time = solver_time_end - solver_time_start;
    DebugOn("Lower Bound = " << lb << endl);
    DebugOn("Upper Bound = " << ub << endl);
    DebugOn("Optimality gap = " << 100*(ub-lb)/ub << "%" << endl);
    DebugOn("Solve time = " << solve_time << endl);
    DebugOn("Total time = " << total_time << endl);
    string out = "DATA_ROMDST, lb = " + to_string(lb) + ", ub = " + to_string(ub) + ", solve_time = " + to_string(solve_time);
    if(return_status==100){
        if(ROMDST->is_convex() && (ub-lb)/ub < 1e-4){
            out += ", GlobalOpimal, ";
        }
        else{
            out += ", LocalOptimal, ";
        }
    }
    else {
        out += ", NoSolution, ";
    }
    out += to_string(total_time);
    DebugOn(out <<endl);
//        return 0;
    auto nb_threads = 3;
    double start_built_time = get_wall_time();
    cont_solve_start = get_wall_time();
    vector<shared_ptr<Model>> conting_mods;
    if(return_status==100){
        bool all_feasible = false, added_all = false;
        unsigned nb_added = 0, nb_total = 0, nb_exist = grid._existing_diesel_gens.size(), nb_new = 0;
        grid.fix_investment();
        auto vals = grid.w_g.get_vals();
        clog << "Building models for new generators." << endl;
        for (auto i = 0; i<vals->size(); i++) {
            if (vals->at(i)) {//This generator was built
                clog << "Deactivating Newly Built Generator: " << grid._potential_diesel_gens[i]->_name << endl;
                grid._potential_diesel_gens[i]->_active = false;
                if (i>0) {
                    grid._potential_diesel_gens[i-1]->_active = true;
                }
                auto contingency_model = grid.build_ROMDST_contingency(grid._potential_diesel_gens[i]->_name);
//                contingency_model->print_expanded();
                conting_mods.push_back(contingency_model);
                nb_new++;
            }
        }
        
        clog << "Building models for existing generators." << endl;
        for (auto i = 0; i<nb_exist; i++) {
            clog << "Deactivating Existing Generator: " << grid._existing_diesel_gens[i]->_name << endl;
            grid._existing_diesel_gens[i]->_active = false;
            if (i>0) {
                grid._existing_diesel_gens[i-1]->_active = true;
            }
            auto contingency_model = grid.build_ROMDST_contingency(grid._existing_diesel_gens[i]->_name);
//            contingency_model->print_expanded();
//            return 0;
            conting_mods.push_back(contingency_model);
        }
        nb_total = nb_new + nb_exist;
        double end_build_time = get_wall_time();
        model_build_time = end_build_time - start_built_time;
        DebugOn("Total time for building continency models = " << model_build_time << endl);
//        return 0;
        while (!all_feasible && !added_all) {
            all_feasible = true;
            nb_total = 0;
            clog << "Running Feasibility Models in Parallel" << endl;
            run_parallel(conting_mods,ipopt,tol=1e-6,nb_threads=5);
            clog << "Checking N-1 Scenarios" << endl;
            /* Check Feasibility */
            for (auto &mod:conting_mods) {
                auto pshed_max = (var<>*)mod->get_var("P_shed_max");
                auto pshed = (var<>*)mod->get_var("P_shed");
                
//                DebugOn(pshed->to_str(true) << endl);
//                DebugOn(pshed_max->to_str(true) << endl);
//                auto pb = (var<>*)mod->get_var("Pb");
//                DebugOn(pb->to_str(true) << endl);
//                auto pb_ = (var<>*)mod->get_var("Pb_");
//                DebugOn(pb_->to_str(true) << endl);
//                auto sc = (var<>*)mod->get_var("Sc");
//                DebugOn(sc->to_str(true) << endl);
//                mod->get_constraint("BatteryEfficiency")->print_expanded();
//                mod->get_constraint("KCL_P")->print_expanded();
//                mod->get_constraint("KCL_Q")->print_expanded();
//                mod->get_constraint("State_Of_Charge")->print_expanded();
                
                if (mod->_status!=100) {
                    clog << "ERROR WHEN SOLVING N-1 CONTINGENCY WITH GEN:" << mod->get_name() << endl;
                    return -1;
                }
                
                auto max_shed = pshed_max->eval()*100;
                if (max_shed>2) {
                    auto Pshed_cstr = mod->get_constraint("max_load_shed");
                    for (auto inst = 0; inst<Pshed_cstr->_nb_instances; inst++) {
                        if (pshed->eval(inst)>1e-4 && Pshed_cstr->is_active(inst,1e-5)) {
                            DebugOn("Max load shed active constraint: ");
                            //                        Pshed_cstr->print(inst);
                            DebugOn("P_shed[" << pshed->get_rev_indices()->at(inst) << "] = " << pshed->eval(inst) << endl);
                        }
                    }
                    clog << "VIOLATED N-1 CONTINGENCY WITH GEN:" << mod->get_name() << endl;
                    clog << "Percentage of load shed = " << max_shed << "%" << endl;
//                    auto pshed = (var<>*)mod->get_var("P_shed");
//                    DebugOn(pshed->to_str(true) << endl);
//                    auto Pshed_cstr = mod->get_constraint("max_load_shed");
//                    for (auto inst = 0; inst<Pshed_cstr->get_nb_instances(); inst++) {
//                        if (Pshed_cstr->is_active(inst)) {
//                            DebugOn("Max load shed active constraint: " << Pshed_cstr->to_str(inst) << endl);
//                        }
//                    }
                    return_status = -1;
//                    return -1;
                    /* Add cuts */
                }
                else{
                    clog << "N-1 CONTINGENCY SATISFIED FOR GEN:" << mod->get_name() << endl;
                }
            }
            
            added_all = (nb_total==nb_added);
        }

    }
    cont_solve_end = get_wall_time();
    cont_solve_time = cont_solve_end - cont_solve_start;
    out = "DATA_ROMDST, lb = " + to_string(lb) + ", ub = " + to_string(ub) + ", gap = " + to_string(100*(ub-lb)/ub) + "%, base case solve_time = " + to_string(solve_time) + ", contingency cases solve_time = " + to_string(cont_solve_time) + " (inlcuding model build time = " + to_string(model_build_time) + ") ";
    if(return_status==100){
        clog << "Current Solution Satisfies N-1 Constraints." << endl;
        if(ROMDST->is_convex() && (ub-lb)/ub < 1e-4){
            out += ", GlobalOpimal, ";
        }
        else{
            out += ", LocalOptimal, ";
        }
        ROMDST->print_solution();
    }
    else {
        clog << "WARNING: Current Solution Violates N-1 Constraints." << endl;
        out += ", NoSolution, ";
    }
    total_time_end = get_wall_time();
    total_time = total_time_end - total_time_start;
    out += "total_time = " + to_string(total_time);
    DebugOn(out <<endl);
    return 0;
}
