#include <gravity/utils.h>
#include <gravity/types.h>
#include <time.h>
using namespace std;
using namespace gravity;

bool gravity::is_weekend(const tuple<int,int,int,int>& ymdh){
    time_t t = time(nullptr);
    tm timeinfo = *localtime(&t);
    timeinfo.tm_year = get<0>(ymdh) - 1900;
    timeinfo.tm_mon = get<1>(ymdh)-1;
    timeinfo.tm_mday = get<2>(ymdh);
    mktime ( &timeinfo );
    return (timeinfo.tm_wday==0 || timeinfo.tm_wday==6);
}


int gravity::get_nb_days_in_month(const tm& timeinfo){
    int numberOfDays;
    if (timeinfo.tm_mon == 3 || timeinfo.tm_mon == 5 || timeinfo.tm_mon == 8 || timeinfo.tm_mon == 10)
        numberOfDays = 30;
    else if (timeinfo.tm_mon == 1)
    { bool isLeapYear = ((timeinfo.tm_year+1900) % 4 == 0 && (timeinfo.tm_year+1900) % 100 != 0) || ((timeinfo.tm_year+1900) % 400 == 0);
        if (isLeapYear)
            numberOfDays = 29;
        else
            numberOfDays = 28;
    }
    else
        numberOfDays = 31;
    return numberOfDays;
}

set<int> gravity::get_phases(string phases){
    set<int> _phases;
    if(phases.empty())
        return _phases;
    auto pos = phases.find_first_of(",");
    auto ph = phases.substr(0,pos);
    _phases.insert(stoi(ph));
    phases = phases.substr(pos+1);
    if(phases.empty())
        return _phases;
    pos = phases.find_first_of(",");
    ph = phases.substr(0,pos);
    _phases.insert(stoi(ph));
    phases = phases.substr(pos+1);
    if(phases.empty())
        return _phases;
    pos = phases.find_first_of(",");
    ph = phases.substr(0,pos);
    _phases.insert(stoi(ph));
    return _phases;
}

gravity::indices time(unsigned p1 ,unsigned p2){
    gravity::indices res(p1,p2);
    res._time_extended = true;
    return res;
}

gravity::indices gravity::range(size_t i, size_t j){
    gravity::indices res(to_string(i)+":"+to_string(j));
    for(auto idx = i; idx <=j; idx++){
        res.add(to_string(idx));
    }
    return res;
}

gravity::indices gravity::operator-(const gravity::indices& s1, const gravity::indices& s2){
    gravity::indices res;
    for(auto &key: *s1._keys){
        if(s2._keys_map->count(key)==0)
            res.add(key);
    }
    return res;
}

#ifdef _WIN32
#include <Windows.h>

double get_wall_time() {
    LARGE_INTEGER time,freq;
    if (!QueryPerformanceFrequency(&freq)) {
        return 0;
    }
    if (!QueryPerformanceCounter(&time)) {
        return 0;
    }
    return (double)time.QuadPart / freq.QuadPart;
}
double get_cpu_time() {
    FILETIME a,b,c,d;
    if (GetProcessTimes(GetCurrentProcess(),&a,&b,&c,&d) != 0) {
        return
        (double)(d.dwLowDateTime |
                 ((unsigned long long)d.dwHighDateTime << 32)) * 0.0000001;
    } else {
        return 0;
    }
}
#else
#include <time.h>
#include <sys/time.h>
double get_wall_time() {
    struct timeval time;
    if (gettimeofday(&time,NULL)) {
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}
double get_cpu_time() {
    return (double)clock() / CLOCKS_PER_SEC;
}
#endif


string clean_print(bool pos, const string& v, bool brackets){
    if(pos){
        if (v=="-1" || v==" - 1" || v=="(-1,0)") {
            return " - ";
        }
        else if (v.front()=='-'){
            return " - " + v.substr(1);
        }
        else if(v=="1" || v==" + 1" || v=="(1,0)") {
            return " + ";
        }
        else if(brackets){
            return " + ("+v+")";
        }
        else{
            return " + " + v;
        }
    }
    else {
        if (v == "-1" || v==" - 1" || v=="(-1,0)") {
            return " + ";
        }
        else if (v.front()=='-'){
            return " + " + v.substr(1);
        }
        else if (v=="1" || v==" + 1" || v=="(1,0)"){
            return " - ";
        }
        else if(brackets){
            return " - ("+v+")";
        }
        else{
            return " - " + v;
        }
    }
}

int nthOccurrence(const std::string& str, const std::string& findMe, int nth)
{
    size_t  pos = 0;
    int     cnt = 0;
    
    while( cnt != nth )
    {
        pos+=1;
        pos = str.find(findMe, pos);
        if ( pos == std::string::npos )
            return -1;
        cnt++;
    }
    return pos;
}

op::OptionParser readOptions(int argc, char * argv[]){
    string log_level ="0";
    op::OptionParser opt;
    opt.add_option("h", "help", "shows option help"); // no default value means boolean options, which default value is false
    opt.add_option("l", "log", "Log level (def. 0)", log_level );
    
    return opt;
}

bool operator <(const Cpx& lhs, const Cpx& rhs){
    return lhs.real()<rhs.real() && lhs.imag()<rhs.imag();
}

bool operator >(const Cpx& lhs, const Cpx& rhs){
    return lhs.real()>rhs.real() && lhs.imag()>rhs.imag();
}

bool operator <=(const Cpx& lhs, const Cpx& rhs){
    return lhs.real()<=rhs.real() && lhs.imag()<=rhs.imag();
}

bool operator >=(const Cpx& lhs, const Cpx& rhs){
    return lhs.real()>=rhs.real() && lhs.imag()>=rhs.imag();
}




//Cpx gravity::min (const Cpx& a, const Cpx& b){
//    Cpx res(a);
//    if (res.real()>b.real()) {
//        res.real(b.real());
//    }
//    if (res.imag()>b.imag()) {
//        res.imag(b.imag());
//    }
//    return res;
//}
//
//Cpx gravity::max (const Cpx& a, const Cpx& b)
//{
//    Cpx res(a);
//    if (res.real()<b.real()) {
//        res.real(b.real());
//    }
//    if (res.imag()<b.imag()) {
//        res.imag(b.imag());
//    }
//    return res;
//}

Sign reverse(Sign s) {
    if(s==unknown_){
        return unknown_;
    }
    return Sign(-1*s);
}

Sign sign_add(Sign s1, Sign s2){
    if (s1==unknown_ || s2==unknown_) {
        return unknown_;
    }
    else if((s1==non_neg_ || s1==pos_) && (s2==neg_ || s2==non_pos_)){
        return unknown_;
    }
    else if((s1==non_pos_ || s1==neg_) && (s2==pos_ || s2==non_neg_)){
        return unknown_;
    }
    else if(s1==zero_ || s1==pos_ || s1==neg_){// take weaker sign
        return s2;
    }
    else{
        return s1;
    }
}

Sign sign_product(Sign s1, Sign s2){
    if (s1==unknown_ || s2==unknown_) {
        return unknown_;
    }
    else if(s1==pos_ && (s2==neg_ || s2==non_pos_)){
        return s2;
    }
    else if(s1==non_neg_ && (s2==neg_ || s2==non_pos_)){
        return non_pos_;
    }
    else if(s1==neg_ && s2==neg_){
        return pos_;
    }
    else if(s1==neg_ && s2==non_pos_){
        return non_neg_;
    }
    return s1;
}

/*Split "mem" into "parts", e.g. if mem = 10 and parts = 4 you will have: 0,2,4,6,10, i.e., [0,2], [2,4], [4,6], [6,10] if possible the function will split mem into equal chuncks, if not the last chunck will be slightly larger */
std::vector<size_t> bounds(unsigned parts, size_t mem) {
    std::vector<size_t>bnd;
    size_t delta = mem / parts;
    size_t reminder = mem % parts;
    size_t N1 = 0, N2 = 0;
    bnd.push_back(N1);
    for (size_t i = 0; i < parts; ++i) {
        N2 = N1 + delta;
        if (i == parts - 1)
            N2 += reminder;
        bnd.push_back(N2);
        N1 = N2;
    }
    return bnd;
}

