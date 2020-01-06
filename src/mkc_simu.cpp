#include<iostream>   
#include<fstream>    
#include<array>
#include<vector>
#include<boost/numeric/odeint.hpp>
#include"matplotlibcpp.h"

using namespace std;
using namespace boost::numeric::odeint;
namespace plt = matplotlibcpp;


typedef vector< double > container_type;
using state = array<double, 2>;


    const double tstart = 0;
    const double tend = 5;
    const double dt = 0.05;
    const int length = (int)(tend/dt)+1;
    double tout[length];
    double xout[length][2];
    int i=0;

struct mkc_system {
    public:
        using state = std::array<double, 2>;
    public:
        double m_m;
        double m_k;
        double m_c;
    public:
        mkc_system(double m,double k,double c)
            :m_m(m),m_k(k),m_c(c){}

        void operator()(const state& x, state& dxdt, double t){
        //mx'' + cx' + kx = 0
        //dx/dt = x' 
        //d(x')/dt =- c/m x' - k/m x
        // x=x[0], x'=x[1]
        dxdt[0] = x[1];              // dx/dt
        dxdt[1] = -(m_c/m_m)*x[1] -(m_k/m_m)*x[0] ;   // d(x')/dt
        }
};

/*
//csv形式で軌道を記録するobserver
struct csv_observer{
    using state = mkc_system::state;
    std::ofstream fout;
    csv_observer(const std::string& FileName) :fout(FileName){};
    //ここで記録方法を定義する
    void operator()(const state& x, double t){
        fout << t << "," << x[0] << "," << x[1] << std::endl;
    }
};
*/

int main(){
    int m=1;
    int k = 1;
    int c = 1;
    mkc_system System(m, k, c);
    mkc_system::state State = {10.0,0.};

    std::cout << "time=0 " << "x=" << State[0] << " " << "y=" << State[1] << std::endl;

    //オイラー法を使ってみる
    boost::numeric::odeint::euler<mkc_system::state> Stepper;
    //4次のルンゲクッタ法を使ってみる
    //boost::numeric::odeint::runge_kutta4<mkc_system::state> Stepper;
    


    boost::numeric::odeint::integrate_const(Stepper, System, State, tstart,tend, dt, 
    [&](const state &x, const double t) //ステップ毎に実行される関数.
    {
        //標準出力へ
        tout[i] = t;
        xout[i][0] = x[0];
        xout[i][1] = x[1];
        i++;
    });


    cout<<"matplotlib-cpp sample start"<<endl;

    vector<double> x(length), y(length);
    for(int i=0; i<length; ++i) {
    x.at(i) = tout[i];
    y.at(i) = xout[i][0];
    }

    plt::plot(x, y, "--r");
    plt::show();
    std::cout << "time=10 " << "x=" << State[0] << " " << "y=" << State[1] << std::endl;

    return 0;
}