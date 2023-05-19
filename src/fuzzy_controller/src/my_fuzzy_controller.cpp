#include "fuzzy_controller/my_fuzzy_controller.h"
#include <cmath>

MyFuzzyController::MyFuzzyController(int subsetNum, int inputNum, int ruleNum)
    : N_(subsetNum)
    , inputNum_(inputNum)
    , ruleNum_(ruleNum)
{
    // subSet_.resize(N_);
    rules_.resize(ruleNum_);
    input_.resize(inputNum_);
    mFunc_.resize(inputNum_);
}

//三角隶属度函数
double MyFuzzyController::trimf(double x,double a,double b,double c)
{
   double u;
   if(x>=a&&x<=b)
       u=(x-a)/(b-a);
   else if(x>b&&x<=c)
       u=(c-x)/(c-b);
   else
       u=0.0;
   return u;

}
//正态隶属度函数
double MyFuzzyController::gaussmf(double x,double ave,double sigma) 
{
    double u;
    if(sigma<0)
    {
       cout<<"In gaussmf, sigma must larger than 0"<<endl;
    }
    u=exp(-pow(((x-ave)/sigma),2));
    return u;
}
//梯形隶属度函数
double MyFuzzyController::trapmf(double x,double a,double b,double c,double d)
{
    double u;
    if(x>=a&&x<b)
        u=(x-a)/(b-a);
    else if(x>=b&&x<c)
        u=1;
    else if(x>=c&&x<=d)
        u=(d-x)/(d-c);
    else
        u=0;
    return u;
}
// 分段隶属度函数
double MyFuzzyController::Segmf(double x, double k)
{
    if(x <= 0){
        return 0;
    } else if(x <= k){
        return pow(2,3) * pow((x / (k * 2)), 4);
    } else if(x <= 2 * k){
        return 1 - pow(2, 3) * pow((2 * k - x) / (2 * k), 4);
    } else {
        return 1;
    }
}

void MyFuzzyController::setSubSet(vector<vector<int>> subset)
{
    int n = subset[0].size();
    vector<int> temp(n,0);
    subSet_.resize(N_, temp);
    for(int i = 0; i < N_; ++i){
        for(int j = 0; j < n; ++j){
            subSet_[i][j] = subset[i][j];
        }
    }
}

void MyFuzzyController::setRules(vector<double> rules)
{
    for(int i = 0; i < rules.size(); ++i){
        rules_[i] = rules[i];
    }
}

void MyFuzzyController::setSetMFunc(vector<int> mFunc)
{
    for(int i = 0; i < mFunc.size(); ++i){
        mFunc_[i] = mFunc[i];
    }
}

double MyFuzzyController::realize(vector<double> inputData, vector<double> k)
{
    vector<double> miuN(inputNum_, 0);
    vector<double> miuM(inputNum_, 0);
    for(int i = 0; i < inputNum_; ++i){
        miuN[i] = Segmf(inputData[i], k[i]);
        miuM[i] = 1 - miuN[i];
    }
    int tableRow = pow(2, inputNum_);
    vector<vector<double>> table(tableRow, vector<double>(inputNum_ + 1, 0));

    // fill tables
    table[0][0] = miuM[0];
    table[0][1] = miuN[1];
    table[0][2] = miuM[2];
    table[0][3] = rules_[0];

    table[1][0] = miuN[0];
    table[1][1] = miuN[1];
    table[1][2] = miuM[2];
    table[1][3] = rules_[0];
    
    table[2][0] = miuM[0];
    table[2][1] = miuN[1];
    table[2][2] = miuN[2];
    table[2][3] = rules_[1];
    
    table[3][0] = miuN[0];
    table[3][1] = miuN[1];
    table[3][2] = miuN[2];
    table[3][3] = rules_[2];

    table[4][0] = miuM[0];
    table[4][1] = miuM[1];
    table[4][2] = miuM[2];
    table[4][3] = rules_[3];

    table[5][0] = miuN[0];
    table[5][1] = miuM[1];
    table[5][2] = miuM[2];
    table[5][3] = rules_[4];

    table[6][0] = miuM[0];
    table[6][1] = miuM[1];
    table[6][2] = miuN[2];
    table[6][3] = rules_[5];

    table[7][0] = miuN[0];
    table[7][1] = miuM[1];
    table[7][2] = miuN[2];
    table[7][3] = rules_[5];

    double u = 0;
    double subU = 0;
    for(int i = 0; i < tableRow; ++i){
        // u += table[i][0] * table[i][3] + table[i][1] * table[i][3] + table[i][2] * table[i][3];
        // subU += table[i][0] + table[i][1] + table[i][2];
        
        // mu_R_i

        //TEST
        // int minIndex = 0;
        // double tmp = min(table[i][1], table[i][2]);
        // minIndex = table[i][0] < table[i][1]? 0 : 1;
        // tmp = min(table[i][2], tmp);
        // minIndex = table[i][2] < tmp? 2 : minIndex;
        // std::cout<< minIndex << " ";
        
        double tmp = min(table[i][0], min(table[i][1], table[i][2]));
        u += tmp * table[i][3];
        subU += tmp;
    }
    return u / subU;
}

double MyFuzzyController::realize2(vector<double> inputData, vector<double> k)
{
    vector<double> miuN(inputNum_, 0);
    vector<double> miuM(inputNum_, 0);
    for(int i = 0; i < inputNum_; ++i){
        miuN[i] = Segmf(inputData[i], k[i]);
        miuM[i] = 1 - miuN[i];
    }
    int tableRow = pow(2, inputNum_);
    vector<vector<double>> table(tableRow, vector<double>(inputNum_ + 1, 0));

    // fill tables
    table[0][0] = miuM[0];
    table[0][1] = miuM[1];
    table[0][2] = miuN[2];
    table[0][3] = rules_[1];

    table[1][0] = miuM[0];
    table[1][1] = miuM[1];
    table[1][2] = miuM[2];
    table[1][3] = rules_[1];
    
    table[2][0] = miuN[0];
    table[2][1] = miuN[1];
    table[2][2] = miuN[2];
    table[2][3] = rules_[2];
    
    table[3][0] = miuN[0];
    table[3][1] = miuN[1];
    table[3][2] = miuM[2];
    table[3][3] = rules_[2];

    table[4][0] = miuN[0];
    table[4][1] = miuM[1];
    table[4][2] = miuN[2];
    table[4][3] = rules_[3];

    table[5][0] = miuN[0];
    table[5][1] = miuM[1];
    table[5][2] = miuM[2];
    table[5][3] = rules_[3];

    table[6][0] = miuM[0];
    table[6][1] = miuN[1];
    table[6][2] = miuN[2];
    table[6][3] = rules_[4];

    table[7][0] = miuM[0];
    table[7][1] = miuN[1];
    table[7][2] = miuM[2];
    table[7][3] = rules_[4];

    double u = 0;
    double subU = 0;
    for(int i = 0; i < tableRow; ++i){
        u += table[i][0] * table[i][3] + table[i][1] * table[i][3] + table[i][2] * table[i][3];
        subU += table[i][0] + table[i][1] + table[i][2];
    }
    return u / subU;
}

double MyFuzzyController::realize3(vector<double> inputData, vector<double> k)
{
    vector<double> miuN(inputNum_, 0);
    vector<double> miuM(inputNum_, 0);
    for(int i = 0; i < inputNum_; ++i){
        miuN[i] = Segmf(inputData[i], k[i]);
        miuM[i] = 1 - miuN[i];
    }
    int tableRow = pow(2, inputNum_);
    vector<vector<double>> table(tableRow, vector<double>(inputNum_ + 1, 0));

    table[0][0] = miuM[0];
    table[0][1] = miuN[1];
    table[0][2] = miuM[2];
    table[0][3] = miuN[3];
    table[0][4] = rules_[0];

    table[1][0] = miuM[0];
    table[1][1] = miuN[1];
    table[1][2] = miuN[2];
    table[1][3] = miuN[3];
    table[1][4] = rules_[0];

    table[2][0] = miuN[0];
    table[2][1] = miuN[1];
    table[2][2] = miuM[2];
    table[2][3] = miuN[3];
    table[2][4] = rules_[0];

    table[3][0] = miuN[0];
    table[3][1] = miuN[1];
    table[3][2] = miuN[2];
    table[3][3] = miuN[3];
    table[3][4] = rules_[0];

    table[4][0] = miuM[0];
    table[4][1] = miuN[1];
    table[4][2] = miuM[2];
    table[4][3] = miuM[3];
    table[4][4] = rules_[1];

    table[5][0] = miuM[0];
    table[5][1] = miuN[1];
    table[5][2] = miuN[2];
    table[5][3] = miuM[3];
    table[5][4] = rules_[1];

    table[6][0] = miuN[0];
    table[6][1] = miuN[1];
    table[6][2] = miuM[2];
    table[6][3] = miuM[3];
    table[6][4] = rules_[1];

    table[7][0] = miuN[0];
    table[7][1] = miuN[1];
    table[7][2] = miuN[2];
    table[7][3] = miuM[3];
    table[7][4] = rules_[1];

    table[8][0] = miuM[0];
    table[8][1] = miuM[1];
    table[8][2] = miuN[2];
    table[8][3] = miuM[3];
    table[8][4] = rules_[2];

    table[9][0] = miuM[0];
    table[9][1] = miuM[1];
    table[9][2] = miuN[2];
    table[9][3] = miuN[3];
    table[9][4] = rules_[3];

    table[10][0] = miuN[0];
    table[10][1] = miuM[1];
    table[10][2] = miuM[2];
    table[10][3] = miuM[3];
    table[10][4] = rules_[4];

    table[11][0] = miuN[0];
    table[11][1] = miuM[1];
    table[11][2] = miuM[2];
    table[11][3] = miuN[3];
    table[11][4] = rules_[4];

    table[12][0] = miuN[0];
    table[12][1] = miuM[1];
    table[12][2] = miuN[2];
    table[12][3] = miuM[3];
    table[12][4] = rules_[4];

    table[13][0] = miuN[0];
    table[13][1] = miuM[1];
    table[13][2] = miuN[2];
    table[13][3] = miuN[3];
    table[13][4] = rules_[4];

    table[14][0] = miuM[0];
    table[14][1] = miuM[1];
    table[14][2] = miuM[2];
    table[14][3] = miuM[3];
    table[14][4] = rules_[5];

    table[15][0] = miuM[0];
    table[15][1] = miuM[1];
    table[15][2] = miuM[2];
    table[15][3] = miuN[3];
    table[15][4] = rules_[6];

    double subU = 0.0;
    double u = 0.0;
    for(int i = 0; i < tableRow; ++i){
        // double tmp = min(table[i][0], min(table[i][1], min(table[i][2], table[i][3])));
        // u += tmp * table[i][4];
        // subU += tmp;
        subU += table[i][0] * table[i][1] * table[i][2] * table[i][3] * table[i][4];
        u += table[i][0] * table[i][1] * table[i][2] * table[i][3];
    }
    return subU / u;
}
