#ifndef SMOOTH_GLOBAL_PLANNER_UTILS
#define SMOOTH_GLOBAL_PLANNER_UTILS

#include <iostream>
#include <vector>
#include <string>

typedef std::string str;

namespace plot{

namespace line
{
    //shape of line
    str SOLID = "-";
    str DASHED = "--";
    str DOTTED = ":";
    str DASHDOT = "-.";
}

namespace color
{
    // color for plot
    str RED = "r";
    str GREEN = "g";
    str BLUE = "b";
    str CYAN = "c";
    str MAGENTA = "m";
    str YELLOW = "y";
    str BLACK = "k";
    str WHITE = "w";
}

namespace point
{
    // shape of point
    str PLUS = "+";
    str CIRCLE = "o";
    str ASTERISK = "*";
    str POINT = ".";
    str CROSS = "x";
}

class Figure{
public:
    Figure();

    Figure(int h, int w);

    void DrawLine(const std::vector<std::pair<double, double>> &points, const std::string &name, const std::string style);

    void DrawLine(const std::vector<double> &x, const std::vector<double> &y, const std::string &name, std::string style);

    void DrawCompare(const std::vector<std::pair<double, double>> &points, 
                    std::string style1,
                    const std::vector<double> &x2, const std::vector<double> &y2, 
                    std::string style2);

    void PlotShow();
private:

    int high_;
    int width_;
};
}

#endif //SMOOTH_GLOBAL_PLANNER_UTILS