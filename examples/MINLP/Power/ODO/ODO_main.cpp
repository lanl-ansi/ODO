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
    string Json_str = string(prj_dir)+"/data_sets/Power/Net.json", Xls_str = string(prj_dir)+"/data_sets/Power/ExtraData.xlsx";
    string Json = string(prj_dir)+"/data_sets/Power/Net.json", ExtraData = string(prj_dir)+"/data_sets/Power/ExtraData.xlsx", mtype = "ACRECT";
    string solver_str="ipopt", default_str="no", networked_str="no";
    int output = 0;
    bool relax = false, use_cplex = false, use_gurobi = false, default_args=false, run_networked = false;
    double tol = 1e-6;
    string mehrotra = "no", log_level="0", nb_hours="24";
    double solver_time_end, total_time_end, solve_time, cont_solve_start, cont_solve_end, cont_solve_time, model_build_time = -1, total_time = -1;
    double total_time_start = get_wall_time();

    /** create a OptionParser with options */
    op::OptionParser opt;
    opt.add_option("h", "help", "shows option help"); // no default value means boolean options, which default value is false
    opt.add_option("e", "ExtraData", "ExtraData options input file name (def. ExtraData.xlsx)", ExtraData);
    opt.add_option("j", "json", "Json input file name (def. Net.json)", Json );
    opt.add_option("l", "log", "Log level (def. 0)", log_level );
    opt.add_option("d", "default", "Use default arguments for input files: yes/no, default = no", default_str);
    
    /** parse the options and verify that all went well. If not, errors and help will be shown */
    bool correct_parsing = opt.parse_options(argc, argv);
    
    if(!correct_parsing){
        return EXIT_FAILURE;
    }
    bool changed_input = false;
    ExtraData = opt["e"];
    Json = opt["j"];
    default_str = opt["d"];
    if (networked_str.compare("yes")==0) {
        run_networked = true;
    }
    if (opt["j"].empty() && default_str.compare("yes")==0) {
        default_args = true;
        cout << "Using default arguments for Json input file: the default file found under data_sets/Power/Net.json" << endl;
    }
    if (opt["e"].empty() && default_str.compare("yes")==0) {
        cout << "Using default arguments for Excel input file: the default file found under data_sets/Power/ExtraData.xlsx" << endl;
    }
    if (solver_str.compare("gurobi")==0) {
        use_gurobi = true;
    }
    else if(solver_str.compare("cplex")==0) {
        use_cplex = true;
    }
    output = op::str2int(opt["l"]);
    bool has_help = op::str2bool(opt["h"]);
    /** show help */
    if(has_help) {
        opt.show_help();
        exit(0);
    }
    if (!default_args) {
        if (opt["j"].empty()){
            cout << " Please enter the url for the Json file (hit enter to use the default file found under data_sets/Power/Net.json)\n";
            getline(cin, Json_str);
            if (!Json_str.empty()) {
                changed_input = true;
            }
        }
        if (opt["e"].empty()){
            cout << " Please enter the url for the Excel file (hit enter to use the default file found under data_sets/Power/ExtraData.xlsx)\n";
            getline(cin, Xls_str);
            if (!Xls_str.empty()) {
                changed_input = true;
            }
        }
    }
#if defined(_WIN32)
    downl_json_cmd = string("wget -O Net.json" + Json_str);
    downl_xls_cmd = string("wget -O ExtraData.xlsx \"" + Xls_str + "\"");
#elif defined(__APPLE__)
    downl_json_cmd = string("curl \"" + Json_str + "\" > Net.json");
    downl_xls_cmd = string("curl \"" + Xls_str + "\" > ExtraData.xlsx");
#elif defined(__linux__)
    downl_json_cmd = string("wget -O Net.json \"" + Json_str + "\"");
    downl_xls_cmd = string("wget -O ExtraData.xlsx \"" + Xls_str + "\"");
#endif
    if (opt["j"].empty() && changed_input) {
        system(downl_json_cmd.c_str());
    }
    if (opt["e"].empty() && changed_input) {
        system(downl_xls_cmd.c_str());
    }
    if(Json.empty()){
        Json = "Net.json";
    }
    if(ExtraData.empty()){
        ExtraData = "ExtraData.xlsx";
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
    auto stat = grid.readODO(ExtraData);
    if (stat==-1) {
        cerr << "Error reading Excel File, Exiting" << endl;
        return -1;
    }
//    auto grids = grid.get_separate_microgrids();
    
    /* Grid Parameters */
    auto bus_pairs = grid.get_bus_pairs();
    auto Ngen = grid.gens.size();
    auto Nbat = grid._battery_inverters.size();
    auto Nline = grid.get_nb_active_arcs();
    auto Nbus = grid.get_nb_active_nodes();
    DebugOn("nb generators = " << Ngen << endl);
    DebugOn("nb batteries = " << Nbat << endl);
    DebugOn("nb of existing PV generators = " << grid._existing_PV_gens.size() << endl);
    DebugOn("nb of existing Wind generators = " << grid._existing_wind_gens.size() << endl);
    DebugOn("nb installed lines = " << Nline << endl);
    DebugOn("number of buses = " << Nbus << endl);

    grid._networked = run_networked;
    auto ODO = grid.build_ODO_model(pmt,output,tol,1);
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
        int precision;
        solver<> ACUC(ODO,ipopt);
        return_status = ACUC.run(output=5, tol = 1e-6);
//        ODO->print_solution();
        //ODO->print_nnz_solution(precision=5,tol=1e-4);
//        auto pg = ODO->get_var<double>("Pg");
//        pg.print_vals(10);
//        ODO->write_var_solution("Pg");
        ODO->write_solution();
//        auto pg_ = ODO->get_var<double>("Pg_");
//        pg_.print_vals(10);
//        auto pw = ODO->get_var<double>("Pw");
//        pw.print_vals(10);
//        auto pv = ODO->get_var<double>("Pv");
//        pv.print_vals(10);
//        auto pb = ODO->get_var<double>("Pb");
//        pb.print_vals(10);
//        auto wb = ODO->get_var<double>("w_b");
//        wb.print_vals(10);
//        auto wpv = ODO->get_var<double>("w_pv");
//        wpv.print_vals(10);
//        auto we = ODO->get_var<double>("w_e");
//        we.print_vals(10);
//        auto wg = ODO->get_var<double>("w_g");
//        wg.print_vals(10);
//        auto pls = ODO->get_var<double>("pls");
//        pls.print_vals(6);
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
            ODO->print_nnz_solution();
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
                return_status = ACUC.run(output, tol = 1e-6, "ma97");
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
