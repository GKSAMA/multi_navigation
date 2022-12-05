#include <iostream>
#include <string>
#include <vector>

using namespace std;

enum FuncType{
    Trimf,
    Gaussmf,
    Trapmf,
    Segmf
};

class MyFuzzyController
{
public:
    // 量化论域子集个数
    int N_;
private:
    int inputNum_;
    // int outputNum_;
    int ruleNum_;

    vector<vector<int>> subSet_;
    vector<double> rules_;
    vector<double> input_;
    // vector<double> output_;
    vector<int> mFunc_;

public:
    MyFuzzyController(){}
    MyFuzzyController(int subsetNum, int inputNum, int ruleNum);
    ~MyFuzzyController(){}
    double trimf(double x,double a,double b,double c);           //三角隶属度函数
    double gaussmf(double x,double ave,double sigma);           //正态隶属度函数
    double trapmf(double x,double a,double b,double c,double d);  //梯形隶属度函数
    double Segmf(double x, double k);                          //分段隶属度函数

    void setSubSetNum(int num) { N_ = num; }
    void setInputNum(int num) { inputNum_ = num;}
    void setRuleNum(int num) { ruleNum_ = num; }

    void setSubSet(vector<vector<int>> subset);
    void setRules(vector<double> rules);
    void setSetMFunc(vector<int> mFunc);

    double realize(vector<double> inputData, vector<double> k = {0,0,0});
    double realize2(vector<double> inputData, vector<double> k = {0,0,0});
    double realize3(vector<double> inputData, vector<double> k = {0,0,0,0});
};