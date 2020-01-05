#include<iostream>   
#include<fstream>    
#include<array>
#include<boost/numeric/odeint.hpp>
#include"matplotlibcpp.h"

using namespace boost::numeric::odeint;



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
    
    //observerを用意する
    csv_observer Observer("result1.csv");

    //time = 0 -> 5まで、時間発展を計算してみる
    boost::numeric::odeint::integrate_const(
           Stepper, System, State, 0.0, 5.0, 0.05, std::ref(Observer)
        );

    std::cout << "time=10 " << "x=" << State[0] << " " << "y=" << State[1] << std::endl;
}