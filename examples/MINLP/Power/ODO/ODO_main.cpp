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
#include <cstdlib>
#include <iostream>

using namespace std;
using namespace gravity;

bool file_exists (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}


int main (int argc, char * argv[])
{
    
    string downl_json_cmd, downl_xls_cmd;
    string Json_str, Xls_str;
    string Json, Invest, mtype = "ACRECT";
    string solver_str="ipopt", default_str="no";
    int output = 0;
    bool relax = false, use_cplex = false, use_gurobi = false, default_args=false;
    double tol = 1e-6;
    string mehrotra = "no", log_level="0", nb_hours="1";
    double solver_time_end, total_time_end, solve_time, cont_solve_start, cont_solve_end, cont_solve_time, model_build_time = -1, total_time = -1;
    double total_time_start = get_wall_time();

    /** create a OptionParser with options */
    op::OptionParser opt;
    opt.add_option("h", "help", "shows option help"); // no default value means boolean options, which default value is false
    opt.add_option("i", "invest", "Investment options input file name (def. Invest.xlsx)", Invest );
    opt.add_option("j", "json", "Json input file name (def. Net.json)", Json );
    opt.add_option("l", "log", "Log level (def. 0)", log_level );
    opt.add_option("t", "time", "time in hours (def. 1)", nb_hours );
    opt.add_option("m", "model", "power flow model: ACPOL/ACRECT/DISTFLOW/LINDISTFLOW (def. ACRECT)", mtype );
    opt.add_option("s", "solver", "Solvers: ipopt/cplex/gurobi, default = ipopt", solver_str);
    opt.add_option("d", "default", "Use default arguments for input files: yes/no, default = no", default_str);
    
    /** parse the options and verify that all went well. If not, errors and help will be shown */
    bool correct_parsing = opt.parse_options(argc, argv);
    
    if(!correct_parsing){
        return EXIT_FAILURE;
    }
    bool changed_input = false;
    Invest = opt["i"];
    Json = opt["j"];
    mtype = opt["m"];
    solver_str = opt["s"];
    default_str = opt["d"];
    if (opt["j"].empty() && default_str.compare("yes")==0) {
        default_args = true;
        cout << "Using default arguments for Json input file: https://raw.githubusercontent.com/lanl-ansi/ODO/master/data_sets/Power/IEEE13.json" << endl;
        Json_str = "https://raw.githubusercontent.com/lanl-ansi/ODO/master/data_sets/Power/IEEE13.json";
    }
    if (opt["i"].empty() && default_str.compare("yes")==0) {
        cout << "Using default arguments for Excel input file: https://raw.githubusercontent.com/lanl-ansi/ODO/master/data_sets/Power/ODO_INPUT.xlsx" << endl;
        Xls_str = "https://raw.githubusercontent.com/lanl-ansi/ODO/master/data_sets/Power/ODO_INPUT.xlsx";
    }
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
    if (!default_args) {
        if (opt["j"].empty()){
            cout << " Please enter the url for the Json file (hit enter to use the default url https://raw.githubusercontent.com/lanl-ansi/ODO/master/data_sets/Power/IEEE13.json)\n";
            getline(cin, Json_str);
            if (Json_str.empty()) {
                Json_str = "https://raw.githubusercontent.com/lanl-ansi/ODO/master/data_sets/Power/IEEE13.json";
            }
            else {
                changed_input = true;
            }
        }
        if (opt["i"].empty()){
            cout << " Please enter the url for the Excel file (hit enter to use the default url https://raw.githubusercontent.com/lanl-ansi/ODO/master/data_sets/Power/ODO_INPUT.xlsx)\n";
            getline(cin, Xls_str);
            if (Xls_str.empty()) {
                Xls_str = "https://raw.githubusercontent.com/lanl-ansi/ODO/master/data_sets/Power/ODO_INPUT.xlsx";
            }
            else {
                changed_input = true;
            }
        }
    }
#if defined(_WIN32)
    downl_json_cmd = string("wget -O Net.json" + Json_str);
    downl_xls_cmd = string("wget -O Invest.xlsx \"https://github.com/lanl-ansi/ODO/raw/master/data_sets/Power/ODO_INPUT.xlsx\"");
#elif defined(__APPLE__)
    downl_json_cmd = string("curl \"" + Json_str + "\" > Net.json");
    downl_xls_cmd = string("curl \"" + Xls_str + "\" > Invest.xlsx");
#elif defined(__linux__)
    downl_json_cmd = string("wget -O Net.json \"" + Json_str + "\"");
    downl_xls_cmd = string("wget -O Invest.xlsx \"" + Xls_str + "\"");
#endif
    if (opt["j"].empty() && (changed_input || !file_exists("Net.json"))) {
        system(downl_json_cmd.c_str());
    }
    if (opt["i"].empty() && (changed_input || !file_exists("Invest.xlsx"))) {
        system(downl_xls_cmd.c_str());
    }
    if(Json.empty()){
        Json = "Net.json";
    }
    if(Invest.empty()){
        Invest = "Invest.xlsx";
    }
    PowerNet grid;
    PowerModelType pmt = ACRECT;
    if(mtype.compare("ACPOL")==0) pmt = ACPOL;
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
    
    grid.readJSON(Json);
    auto stat = grid.readODO(Invest);
    if (stat==-1) {
        cerr << "Error reading Excel File, Exiting" << endl;
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

    auto ODO = grid.build_ODO_model(pmt,output,tol,max_nb_hours);
    ODO->print();
//    return 0;
    
    
    
    //    return 0;
    
    /* SOLVE */
    double lb = 0, ub = 0;
    int return_status = -1;
    auto solver_time_start = get_wall_time();
    if (use_cplex) {
        solver<> ACUC(*ODO,cplex);
        return_status = ACUC.run(output, tol = 1e-6);
        if (return_status==100) {
            lb = ODO->get_obj_val();
            ub = ODO->get_obj_val();
        }
    }
    else if (use_gurobi) {
        solver<> ACUC(*ODO,gurobi);
        return_status = ACUC.run(output, tol = 1e-6);
        if (return_status==100) {
            lb = ODO->get_obj_val();
            ub = ODO->get_obj_val();
        }
    }
    else {
        solver<> ACUC(ODO,ipopt);
        return_status = ACUC.run(output=5, tol = 1e-5, "ma97");
        auto pg = ODO->get_var<double>("Pg");
        pg.print_vals(10);
        solver_time_end = get_wall_time();
        solve_time = solver_time_end - solver_time_start;
        DebugOn("Solve time = " << solve_time << endl);
        DebugOn("Optimal Objective = " << ODO->get_obj_val() << endl);
        return 0;
        if (return_status != 100) {
            clog << "Cannot compute lower bound!\n";
        }
        else {
            lb = ODO->get_obj_val();
            DebugOn("Lower Bound = " << lb << endl);
            ODO->print_solution(10);
//            DebugOn("Rounding current solution..." << endl);
//            ODO->round_solution();
//            if(ODO->is_feasible(tol=1e-5)){
            if(false){
                clog << "Feasible integer solution found!\n";
                ub = lb;
                return_status = 100;
            }
            else{
                clog << "Rounded integer solution is not feasible!\n Trying to enforce integrality in the objective\n";
                auto old_obj = ODO->_obj;
//                ODO->add_round_solution_obj();
                //                ODO->add_integrality();
                //                ODO->add_round_solution_cuts();
//                ODO->_first_run = true;
//                solver ACUC_CPX(*ODO,ipopt);
//                return_status = ACUC_CPX.run(output, relax = false, tol = 1e-6);
                return_status = ACUC.run(output, relax = false, tol = 1e-6);
                if (return_status != 100) {
                    clog << "Cannot compute lower bound!\n";
                }
                else {
                    //                ODO->print_solution();
                    ODO->round_solution();
                    if(ODO->is_feasible(tol=1e-4)){
                        clog << "Feasible integer solution found!\n";
                        ub = old_obj->eval();
                    }
                    else{
                        clog << "Current integer solution is infeasible!\n Trying to solve with pure integer objective\n";
//                        ODO->add_round_solution_obj(false);
                        //                        ODO->_first_run = true;
                        //                        solver ACUC_CPX(*ODO,cplex);
                        return_status = ACUC.run(output, relax = false, tol = 1e-6);
                        if (return_status != 100) {
                            clog << "Cannot compute lower bound!\n";
                        }
                        else {
                            ODO->round_solution();
                            if(ODO->is_feasible(tol=1e-4)){
                                clog << "Feasible integer solution found!\n";
                                ub = old_obj->eval();
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
    string out = "DATA_ODO, lb = " + to_string(lb) + ", ub = " + to_string(ub) + ", solve_time = " + to_string(solve_time);
    if(return_status==100){
        if(ODO->is_convex() && (ub-lb)/ub < 1e-4){
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
    return 0;
}
