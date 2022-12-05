#include"fuzzy_controller/my_fuzzy_controller.h"
#include <iostream>

using namespace std;
#define M 0
#define N 1
#define n1 0.01
#define n2 0.05
#define n3 0.5

int main()
{
    int subsetNum = 2;
    int inputNum = 3;
    int ruleNum = 6;
    vector<double> inputData = {1, 0.2, 19};
    vector<double> k = {15, 1.57, 10};
    vector<double> rules = {n1, n2, n3, 1 / n3, 1 / n2, 1 / n1};

    MyFuzzyController fzctl(subsetNum, inputNum, ruleNum);
    fzctl.setRules(rules);
    double result = fzctl.realize(inputData, k);
    cout << result << endl;
    return 0;
}