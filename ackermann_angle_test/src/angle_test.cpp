#include <iostream>
#include <math.h>
using namespace std;

double lf_p = 0.0, rf_p = 0.0, lb_p = 0.0, rb_p = 0.0;
double lf_v = 0.0, rf_v = 0.0, lb_v = 0.0, rb_v = 0.0;

// 机器人长的一半
const double half_len = 0.268;
// 机器人宽的一半
const double half_wid = 0.159;

double rad2deg(double angle)
{
    return angle * 180 / M_PI;
}

double deg2rad(double angle)
{
    return angle * M_PI / 180;
}

void computeWheelVel_ackermannSteer(double linear_vel, double front, double rear)
{
    double d_vertical = 0.0, l_up = half_len;
    if (front != 0)
    {
        d_vertical = 2 * half_len * cos(front) * cos(rear) / sin(front - rear);
        l_up = 2 * half_len * sin(front) * cos(rear) / sin(front - rear);
        cout << "d_vertical: " << d_vertical << "\tl_up: " << l_up << endl;
        lf_p = atan(2 * half_len * sin(front) * cos(rear) / (2 * half_len * cos(front) * cos(rear) - half_wid * sin(front - rear)));
        rf_p = atan(2 * half_len * sin(front) * cos(rear) / (2 * half_len * cos(front) * cos(rear) + half_wid * sin(front - rear)));
        lb_p = atan(2 * half_len * cos(front) * sin(rear) / (2 * half_len * cos(front) * cos(rear) - half_wid * sin(front - rear)));
        rb_p = atan(2 * half_len * cos(front) * sin(rear) / (2 * half_len * cos(front) * cos(rear) + half_wid * sin(front - rear)));
    }

    double Rc = sqrt(pow(half_len - l_up, 2) + pow(d_vertical, 2));
    if (Rc == 0)
    {
        lf_v = linear_vel;
        rf_v = linear_vel;
        lb_v = linear_vel;
        rb_v = linear_vel;
    }
    else
    {
        double Rlf = sqrt(pow(l_up, 2) + pow(d_vertical - half_wid, 2));
        double Rrf = sqrt(pow(l_up, 2) + pow(d_vertical + half_wid, 2));
        double Rlr = sqrt(pow(2 * half_len - l_up, 2) + pow(d_vertical - half_wid, 2));
        double Rrr = sqrt(pow(2 * half_len - l_up, 2) + pow(d_vertical + half_wid, 2));
        double angular_vel = linear_vel / Rc;
        lf_v = angular_vel * Rlf;
        rf_v = angular_vel * Rrf;
        lb_v = angular_vel * Rlr;
        rb_v = angular_vel * Rrr;
    }
}

int main()
{
    double linear_vel = 0.1;
    for (int i = 1; i <= 45; ++i)
    {
        computeWheelVel_ackermannSteer(linear_vel, deg2rad(i), deg2rad(-i));
        cout << "front_angle: " << i << "\t rear_angle: " << -i << endl;
        cout << "lf_p: " << rad2deg(lf_p) << "; rf_p: " << rad2deg(rf_p)
             << "; lb_p: " << rad2deg(lb_p) << "; rb_p: " << rad2deg(rb_p) << endl
             << endl;
    }
    return 0;
}